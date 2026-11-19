"""benchkit: lab-bench parts for yam_arena task files: Tools of several boxes with sprung sliders and hinges, and
per-step hooks.

A :class:`Tool` is one loose body of box colliders that can carry :class:`Slider` parts (boxes on prismatic joints)
and :class:`Hinge` parts (rigid groups of boxes on revolute joints; a Hinge can hang from another, and a Slider can
ride on a Hinge). Importing this module hooks ``yam_arena.scene`` and the ``yam_arena`` embodiment's step and reset,
without changing yam_arena itself:

- ``success()`` and ``score()`` receive a :class:`ToolState` for each Tool (body pose, bounds, slider positions, hinge
  angles, hinged parts' frames and named points), and an ordinary ``ObjectState`` for everything else;
- :func:`on_step` and :func:`on_reset` register callbacks that get the object states after every control step
  (before scoring) and after every reset;
- :func:`fingertips` gives the world position of each gripper's pad tips.

This task's copy (each task keeps its own) adds to the shared kit: a box may carry an opacity and a ``ghost`` flag
(``(centre, size, colour, turn, name, opacity, ghost)``; a ghost box is drawn with no collider, and a box of opacity 0,
a Slider's own box included, is a collider that isn't drawn: a collision proxy). A box's ``turn`` is a yaw (rad) or a
full rotation as a quaternion ``(w, x, y, z)``, so a part can lie at any angle (tweezer legs splayed open, a slanted
tip, a splinter at 25 degrees). A Slider or Hinge may carry a joint ``frame`` (quaternion ``(w, x, y, z)``): its axis is
the frame's X, Y or Z, so a part can slide or turn along any direction. A Tool may start at any orientation (``quat``,
x y z w). A Slider may ride on an earlier Slider (``parent``), a Hinge may be weightless, and :func:`fingertips` gives
the pad tips. Tilted grips come from ``tiltgrip.py``, not from here. It leaves out the kit's colour changes, set poses
and joint resets, which it doesn't use.

Everything that needs Isaac Sim is in ``benchkit_sim.py``, imported only when the simulator builds the scene, so the task
file and its tests load without a GPU.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

Color = tuple[float, float, float]
# centre, size (tool frame, m), sRGB, and optionally a yaw (rad) and the box's own prim name
BoxSpec = tuple
AXES = {"X": np.array([1.0, 0.0, 0.0]), "Y": np.array([0.0, 1.0, 0.0]), "Z": np.array([0.0, 0.0, 1.0])}


def box_parts(spec) -> tuple[tuple, tuple, Color, object, str]:
    """(centre, size, colour, turn, name) of a box given as 3 to 7 items. ``turn`` is a yaw (rad) or a quaternion
    (w, x, y, z)."""
    centre, size, color = spec[0], spec[1], spec[2]
    turn = spec[3] if len(spec) > 3 else 0.0
    turn = tuple(float(v) for v in turn) if isinstance(turn, tuple | list) else float(turn)
    name = str(spec[4]) if len(spec) > 4 else ""
    return tuple(centre), tuple(size), tuple(color), turn, name


def quat_matrix(q_wxyz) -> np.ndarray:
    """Rotation matrix of a quaternion (w, x, y, z)."""
    w, x, y, z = (float(v) for v in q_wxyz)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def matrix_quat(r) -> tuple[float, float, float, float]:
    """Quaternion (w, x, y, z) of a rotation matrix."""
    r = np.asarray(r, dtype=float)
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = (0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
        v = [0.0, 0.0, 0.0]
        v[i] = 0.25 * s
        v[j] = (r[j, i] + r[i, j]) / s
        v[k] = (r[k, i] + r[i, k]) / s
        q = ((r[k, j] - r[j, k]) / s, *v)
    q = np.array(q)
    q = q if q[0] >= 0 else -q
    return tuple(float(v) for v in q / np.linalg.norm(q))


def turn_matrix(turn) -> np.ndarray:
    """A box's rotation in its parent's frame: a yaw about z, or a quaternion (w, x, y, z)."""
    if isinstance(turn, tuple | list):
        return quat_matrix(turn)
    return axis_rotation("Z", float(turn))


def box_look(spec) -> tuple[float, bool]:
    """(opacity, ghost) of a box: items 6 and 7, default opaque and colliding. A ghost box has no collider; a box of
    opacity 0 is a collider that isn't drawn."""
    opacity = float(spec[5]) if len(spec) > 5 else 1.0
    ghost = bool(spec[6]) if len(spec) > 6 else False
    return opacity, ghost


@dataclass(frozen=True)
class Slider:
    """A box on a prismatic joint along ``axis`` of its parent's frame (the tool's, or a Hinge's named by ``parent``),
    resting at ``rest`` (tool frame). ``parent`` may also name a Slider listed before it, which it then rides. It moves
    between ``lower`` and ``upper`` (m, 0 the rest stop); a spring of ``rate`` N/m returns it to 0, and with ``rate`` 0
    it stays where it is left, held by ``friction`` (N). ``opacity`` 0 makes its own box a collider that isn't drawn.
    ``frame`` (w, x, y, z) turns the joint frame, so ``axis`` may point any way; ``box_turn`` turns its own box."""

    name: str
    rest: tuple[float, float, float]
    size: tuple[float, float, float]
    axis: str = "Z"
    lower: float = -0.010
    upper: float = 0.0
    color: Color = (0.85, 0.85, 0.88)
    rate: float = 600.0
    damping: float = 8.0
    mass: float = 0.004
    friction: float = 0.0
    boxes: tuple = ()
    opacity: float = 1.0
    weightless: bool = False
    parent: str = ""
    box_name: str = ""
    frame: tuple = (1.0, 0.0, 0.0, 0.0)
    box_turn: tuple = (1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Hinge:
    """A rigid part of ``boxes`` on a revolute joint about ``axis`` of its parent's frame, anchored at ``rest`` (tool
    frame, at rest). It turns between ``lower`` and ``upper`` (rad, 0 where its boxes are drawn), starts at ``init``,
    and has a spring ``rate`` (N m/rad), ``damping`` and joint ``friction`` (N m). ``frame`` (w, x, y, z) turns the
    joint frame, so ``axis`` may point any way."""

    name: str
    rest: tuple[float, float, float]
    boxes: tuple
    axis: str = "Z"
    lower: float = -3.0
    upper: float = 3.0
    color: Color = (0.85, 0.85, 0.88)
    rate: float = 0.0
    damping: float = 0.15
    friction: float = 0.03
    mass: float = 0.3
    parent: str = ""
    init: float = 0.0
    frame: tuple = (1.0, 0.0, 0.0, 0.0)
    weightless: bool = False


@dataclass(frozen=True)
class Tool(Box):
    """A loose rigid tool made of ``boxes`` (tool frame). It subclasses Box so yam_arena's task-file checks accept
    it: ``pos`` is the body origin and ``size`` its bounding box. ``points`` names tool-frame points (a tip, say)
    that :meth:`ToolState.point` puts in the world; ``link_points`` names points on hinged parts, as
    ``(name, hinge, local point)``. ``weightless`` turns gravity off for the body; ``quat`` (x, y, z, w) is its
    starting orientation. ``self_collide`` lets the Tool's
    parts collide with each other (PhysX still skips each joint's two bodies)."""

    boxes: tuple = ()
    sliders: tuple[Slider, ...] = ()
    friction: float = 1.0
    points: tuple[tuple[str, tuple[float, float, float]], ...] = ()
    weightless: bool = False
    hinges: tuple[Hinge, ...] = ()
    self_collide: bool = False
    link_points: tuple = ()
    quat: tuple = (0.0, 0.0, 0.0, 1.0)

    def _check(self) -> None:
        super()._check()
        if not self.boxes:
            raise ValueError(f"{self.name}: a Tool needs at least one box")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Tool must be loose (give it a mass)")
        hinge_names = [h.name for h in self.hinges]
        if len(set(hinge_names)) != len(hinge_names):
            raise ValueError(f"{self.name}: hinge names must be unique")
        slider_names = [s.name for s in self.sliders]
        for i, s in enumerate(self.sliders):
            if not (s.rate >= 0 and s.damping >= 0 and s.friction >= 0 and s.mass > 0 and 0.0 <= s.opacity <= 1.0
                    and s.lower <= 0.0 <= s.upper and s.axis in AXES):
                raise ValueError(f"{self.name}.{s.name}: a Slider needs a positive mass, no negative gains, an opacity "
                                 "in [0, 1] and limits either side of 0")
            if s.parent and s.parent not in hinge_names and s.parent not in slider_names[:i]:
                raise ValueError(f"{self.name}.{s.name}: parent {s.parent!r} is not a hinge or an earlier slider")
        for i, h in enumerate(self.hinges):
            if not (h.boxes and -2 * math.pi < h.lower < 0.0 < h.upper < 2 * math.pi and h.axis in AXES):
                raise ValueError(f"{self.name}.{h.name}: a Hinge needs boxes, an axis and limits either side of 0, "
                                 "inside a full turn")
            if not (h.mass > 0 and h.rate >= 0 and h.damping >= 0 and h.friction >= 0):
                raise ValueError(f"{self.name}.{h.name}: a Hinge needs a positive mass and no negative gains")
            if not h.lower <= h.init <= h.upper:
                raise ValueError(f"{self.name}.{h.name}: init must lie within the hinge's limits")
            if h.parent and h.parent not in hinge_names[:i]:
                raise ValueError(f"{self.name}.{h.name}: parent {h.parent!r} must be a hinge listed before it")
        for _, part, _ in self.link_points:
            if part not in hinge_names and part not in slider_names:
                raise ValueError(f"{self.name}: link point on unknown part {part!r}")


def axis_rotation(axis: str, angle: float) -> np.ndarray:
    """Rotation by ``angle`` (rad) about the unit axis "X", "Y" or "Z"."""
    x, y, z = AXES[axis]
    c, s = math.cos(angle), math.sin(angle)
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + s * k + (1 - c) * (k @ k)


def _flat_reach(size, turn) -> tuple[float, float, float]:
    """Half-extents along x, y and z of a box turned by ``turn`` (a yaw or a quaternion)."""
    return tuple(float(v) for v in np.abs(turn_matrix(turn)) @ (np.asarray(size, dtype=float) / 2))


def tool(name, pos, boxes, mass, *, sliders=(), yaw=0.0, color=(0.5, 0.5, 0.5), friction=1.0, points=(),
         weightless=False, hinges=(), self_collide=False, link_points=(), quat=(0.0, 0.0, 0.0, 1.0)) -> Tool:
    """Build a Tool; its bounding box is taken from the body's boxes and the sliders and hinges at rest."""
    shapes = []
    for b in boxes:
        c, s, tint, turn, _ = box_parts(b)
        shapes.append((c, tuple(2 * r for r in _flat_reach(s, turn)), tint))
    for s in sliders:
        shapes.append((s.rest, s.size, s.color))
        shapes += [(tuple(float(a + b) for a, b in zip(s.rest, box_parts(x)[0])), box_parts(x)[1], box_parts(x)[2])
                   for x in s.boxes]
    for h in hinges:
        for b in h.boxes:
            c, s, tint, turn, _ = box_parts(b)
            middle = tuple(float(a + b) for a, b in zip(h.rest, c))
            shapes.append((middle, tuple(2 * r for r in _flat_reach(s, turn)), tint))
    lo = np.min([np.array(c) - np.array(s) / 2 for c, s, _ in shapes], axis=0)
    hi = np.max([np.array(c) + np.array(s) / 2 for c, s, _ in shapes], axis=0)
    return Tool(name=name, pos=tuple(pos), size=tuple((hi - lo).tolist()), mass=mass, color=color, yaw=yaw,
                boxes=tuple(boxes), sliders=tuple(sliders), friction=friction, points=tuple(points),
                weightless=weightless, hinges=tuple(hinges), self_collide=self_collide, link_points=tuple(link_points),
                quat=tuple(float(v) for v in quat))


def _rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = quat_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def link_frames(spec: Tool, hinges: dict, sliders: dict | None = None) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Each hinge's and slider's frame in the tool frame, as (rotation, origin), from the joint positions ``hinges``
    (rad) and ``sliders`` (m). A joint's anchor is the child's rest point; at 0 every frame is the tool frame moved to
    that rest point. A joint's axis is its ``frame`` rotation applied to X, Y or Z."""
    sliders = sliders or {}
    parts = {p.name: p for p in (*spec.hinges, *spec.sliders)}
    out: dict[str, tuple[np.ndarray, np.ndarray]] = {}

    def place(p):
        if p.name in out:
            return out[p.name]
        if p.parent:
            prot, porg = place(parts[p.parent])
            prest = np.array(parts[p.parent].rest)
        else:
            prot, porg, prest = np.eye(3), np.zeros(3), np.zeros(3)
        anchor = porg + prot @ (np.array(p.rest) - prest)
        axis = quat_matrix(p.frame) @ AXES[p.axis]
        if isinstance(p, Hinge):
            out[p.name] = (prot @ axis_rotation_vec(axis, float(hinges.get(p.name, p.init))), anchor)
        else:
            out[p.name] = (prot, anchor + prot @ axis * float(sliders.get(p.name, 0.0)))
        return out[p.name]

    for p in (*spec.hinges, *spec.sliders):
        place(p)
    return out


def axis_rotation_vec(axis, angle: float) -> np.ndarray:
    """Rotation by ``angle`` (rad) about a unit vector."""
    x, y, z = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    c, s = math.cos(angle), math.sin(angle)
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + s * k + (1 - c) * (k @ k)


@dataclass(frozen=True)
class ToolState(ObjectState):
    """A tool's body pose (``pos`` is the body origin, not the bounding-box centre), velocities, world bounds, each
    slider's position (m from its rest stop), its named points in the world, each hinge's angle (rad from rest) and
    each hinged part's frame in the world (``links``: name -> (rotation, origin))."""

    sliders: dict = field(default_factory=dict)
    named: dict = field(default_factory=dict)
    hinges: dict = field(default_factory=dict)
    links: dict = field(default_factory=dict)

    def rotation(self) -> np.ndarray:
        return _rotation(self.quat_xyzw)

    def to_world(self, local) -> np.ndarray:
        return self.pos + self.rotation() @ np.asarray(local, dtype=float)

    def to_local(self, world) -> np.ndarray:
        return self.rotation().T @ (np.asarray(world, dtype=float) - self.pos)

    def point(self, name: str) -> np.ndarray:
        return self.named[name]

    def tilt_deg(self) -> float:
        """Angle between the tool's z axis and the vertical."""
        return float(np.degrees(np.arccos(np.clip(self.rotation()[2, 2], -1.0, 1.0))))


def tool_state(spec: Tool, pos, quat_xyzw, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0), sliders=None,
               hinges=None) -> ToolState:
    """A ToolState from a body pose and joint positions: what the simulator reports, and what tests build by hand."""
    pos, quat = np.asarray(pos, dtype=float), np.asarray(quat_xyzw, dtype=float)
    rot = _rotation(quat)
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    pts = []
    for b in spec.boxes:
        c, s, _, turn, _ = box_parts(b)
        reach = np.array(_flat_reach(s, turn))
        pts += [pos + rot @ (np.array(c) + k * reach) for k in corners]
    pts = np.array(pts)
    angles = dict(hinges or {h.name: h.init for h in spec.hinges})
    travel = dict(sliders or {s.name: 0.0 for s in spec.sliders})
    frames = link_frames(spec, angles, travel)
    links = {n: (rot @ r, pos + rot @ o) for n, (r, o) in frames.items()}
    named = {n: pos + rot @ np.asarray(p, dtype=float) for n, p in spec.points}
    for n, part, local in spec.link_points:
        r, o = frames[part]
        named[n] = pos + rot @ (o + r @ np.asarray(local, dtype=float))
    return ToolState(pos=pos, quat_xyzw=quat, lin_vel=np.asarray(lin_vel, dtype=float),
                     ang_vel=np.asarray(ang_vel, dtype=float), low=pts.min(0), high=pts.max(0),
                     sliders=travel, named=named,
                     hinges=angles, links=links)


# -- spawning (Isaac Sim, imported lazily) ------------------------------------------------------------------------

_TOOLS: dict[str, Tool] = {}


def joint_name(part: Slider | Hinge) -> str:
    return f"{part.name}_{'turn' if isinstance(part, Hinge) else 'slide'}"


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


# -- hooks into yam_arena -----------------------------------------------------------------------------------------

_STEP: dict = {}
_RESET: dict = {}
_ENV: dict = {"env": None}


def on_step(fn):
    """Register ``fn(states)``: called with every object's state after each control step, before scoring. A task
    file loaded again replaces its earlier hooks rather than adding to them."""
    _STEP[(fn.__module__, fn.__qualname__)] = fn
    return fn


def on_reset(fn):
    """Register ``fn(states)``: called after every reset, once objects have settled."""
    _RESET[(fn.__module__, fn.__qualname__)] = fn
    return fn


def fingertips() -> dict:
    """World position (m, env frame) of the point between each gripper's pad tips, by arm. Empty outside the
    simulator. Errors go through: a scorer that needs the fingers must not run blind."""
    env = _ENV["env"]
    if env is None:
        return {}
    from yam_arena.runtime import pad_tips_world

    base = env.unwrapped
    origin = _np(base.scene.env_origins[0])
    return {arm: np.asarray(pad_tips_world(base.scene[f"{arm}_robot"]), dtype=float) - origin
            for arm in ("left", "right")}


def finger_frames() -> dict:
    """Per arm: (TCP position, TCP rotation with columns closing axis, palm normal, approach, and each finger's pad
    face as an offset from the TCP along the closing axis (m)). Empty outside the simulator."""
    env = _ENV["env"]
    if env is None:
        return {}
    from yam_arena.constants import FINGER_JOINTS
    from yam_arena.runtime import link_poses, tcp_world

    base = env.unwrapped
    origin = _np(base.scene.env_origins[0])
    out = {}
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        rot = link_poses(robot)["gripper"][:3, :3]
        ids = robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        q = robot.data.joint_pos.torch[0, ids].cpu().numpy()
        out[arm] = (np.asarray(tcp_world(robot), dtype=float) - origin, np.array(rot, dtype=float),
                    (abs(float(q[0])), -abs(float(q[1]))))
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_benchkit", False):
        return
    plain_object, plain_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Tool):
            _TOOLS[spec.name] = spec
            from benchkit_sim import _arena_object

            return _arena_object(spec)
        return plain_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _TOOLS]
        out = plain_states(environment, plain) if plain else {}
        from benchkit_sim import _read_tool

        for n in names:
            if n in _TOOLS:
                out[n] = _read_tool(environment, _TOOLS[n])
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = task_object, object_states
    scene._benchkit = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_benchkit", False):
        return
    plain_step, plain_reset = cls._task_step, cls.reset

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self._env, self.task.names)

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        if _STEP:
            now = states(self)
            for fn in list(_STEP.values()):
                fn(now)
        return plain_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = plain_reset(self, *a, **k)
        _ENV["env"] = self._env
        if _RESET:
            now = states(self)
            for fn in list(_RESET.values()):
                fn(now)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._benchkit = True


class _Finder(importlib.abc.MetaPathFinder):
    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, patch = spec.loader, self.targets[fullname]
        plain_exec = loader.exec_module

        def exec_module(module):
            plain_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena: now for modules already imported, otherwise when they are. Safe to call twice."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
