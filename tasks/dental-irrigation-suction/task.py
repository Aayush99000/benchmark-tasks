"""Rinse three lower teeth inside a real-size mouth while a saliva ejector keeps the mouth clear, then put both tools back.

A reclined phantom head (an adult typodont in a printed shell, with lips, cheeks, a tongue, the floor of the mouth and a
throat) lies on a stand, its crown toward the robot. A blue gel dot sits on the occlusal surface of each of the lower
left second premolar and first and second molars (FDI 35, 36, 37). A 2 oz bulb syringe stands in its holder, filled with
water; a saliva ejector lies in its cradle. Squeezing the bulb sends a stream from its nozzle; water runs off the teeth
into the cheek vestibules and back along the floor of the mouth, where it collects against the throat sill either side of
the tongue, and over the sill into the throat when that is full. The ejector draws water when its tip is in it. Scored by mouth.py, following task.json's `scoring`.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import mechkit as m  # noqa: E402
import mouth as M  # noqa: E402
import parts as P  # noqa: E402
import steptime  # noqa: E402

SCENE = "station"
FULL_HOLD_S = 1.0  # the full-score state must hold this long (checked in Trial.step)
SUCCESS_HOLD_S = 0.05  # the harness's own hold on success(), on top: one step
INSTRUCTION = "Rinse the blue gel off the three marked lower teeth while keeping the mouth clear with the saliva ejector, then put both tools back in their holders."

OBJECTS = [P.head(), P.bulb_holder(), P.ejector_cradle(), P.bulb(), P.ejector(), P.water_marks()]

HOME = {"bulb": np.array(P.BULB_START), "ejector": np.array(P.EJ_START)}
HOME_DIST, HOME_DEG, RELEASE = 0.010, 10.0, 0.030  # task.json: within 1 cm, upright within 10 deg; both grippers 3 cm or more from both tools and both holders


def _seg_dist(pts, a, b) -> np.ndarray:
    """Distance from each point to the segment a-b."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    ab = b - a
    t = np.clip((pts - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(pts - (a + t[:, None] * ab), axis=1)


def _ring(centre, r, z, n=12):
    return [np.asarray(centre, float) + [r * math.cos(t), r * math.sin(t), z] for t in np.linspace(0, 2 * math.pi, n, endpoint=False)]


def clearance(fingers, bulb, ej) -> float:
    """The least distance (m) from any point of either gripper's fingers to the surface of the bulb, the ejector, the bulb's
    holder or the ejector's cradle. Each is taken as round segments a little larger than the part (never smaller)."""
    shapes = []  # (a, b, radius)
    Rb, Re = bulb.rotation(), ej.rotation()
    for z in (-0.015, 0.0, 0.015):  # the bulb, as spheres of its full radius
        c = bulb.pos + Rb @ np.array([0, 0, z])
        shapes.append((c, c, P.BULB_SEMI[0]))
    shapes.append((bulb.pos + Rb @ np.array([0, 0, -P.BULB_SEMI[2]]), bulb.pos + Rb @ np.array([0, 0, P.TIP_Z]), 0.0045))
    hy = P.EJ_HANDLE / 2
    shapes.append((ej.pos + Re @ np.array([0, -hy, 0]), ej.pos + Re @ np.array([0, hy, 0]), 0.007))
    for a, b in (((0, -hy, 0), P.EJ_BEND), (P.EJ_BEND, P.EJ_TIP_LOCAL)):
        shapes.append((ej.pos + Re @ np.asarray(a, float), ej.pos + Re @ np.asarray(b, float), 0.0045))
    hb = np.array([*P.HOLDER_XY, M.TOP])
    shapes += [(p, p, 0.003) for p in _ring(hb, P.RING_R + 0.003, P.RING_Z - M.TOP)]
    shapes += [(p, p, 0.001) for p in _ring(hb, 0.032, 0.020)]
    shapes.append((hb + [0, -0.042, 0], hb + [0, -0.042, P.RING_Z - M.TOP + 0.003], 0.0075))
    shapes.append((hb + [0, -0.042, P.RING_Z - M.TOP], hb + [0, -0.019, P.RING_Z - M.TOP], 0.007))
    hc = np.array([*P.CRADLE_XY, M.TOP])
    for dy in (-0.035, 0.035):
        shapes.append((hc + [0, dy, 0], hc + [0, dy, P.CRADLE_Z - M.TOP + 0.011], 0.0145))
    return float(min(np.min(_seg_dist(fingers, a, b)) - r for a, b, r in shapes))


def _tilt_deg(R) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))


class Trial:
    def reset(self, env=None) -> None:
        self.water = M.Water()
        self.report = {}
        self.ops = {}
        self.value = 0.0
        self.ok = False
        self.squeeze = 0.0
        self.held_bulb = False
        self.full_s = 0.0

    # -- what is seen of the water --------------------------------------------------------------
    def _pose(self, path, pos, scale=(1, 1, 1)) -> None:
        if path not in self.ops:
            import isaaclab.sim as sim_utils
            from pxr import UsdGeom

            xf = UsdGeom.Xformable(sim_utils.get_current_stage().GetPrimAtPath(path))
            ops = xf.GetOrderedXformOps()
            tr = next((o for o in ops if o.GetOpType() == UsdGeom.XformOp.TypeTranslate), None) or xf.AddTranslateOp()
            sc = next((o for o in ops if o.GetOpType() == UsdGeom.XformOp.TypeScale), None) or xf.AddScaleOp()
            self.ops[path] = (tr, sc)
        self.ops[path][0].Set(tuple(float(v) for v in pos))
        self.ops[path][1].Set(tuple(float(v) for v in scale))

    def draw(self, tip, ej_tip) -> None:
        head = f"{m.ENV_PATH}/head/visual/"
        marks = f"{m.ENV_PATH}/water_marks/visual/"
        w = self.water
        for i, done in enumerate(w.rinsed()):
            m.set_visible(head + f"dot{i}", not done)
        for side, sgn in (("left", 1.0), ("right", -1.0)):
            f = math.sqrt(min(w.sill[side], M.SILL_CAP) / M.SILL_CAP)  # the wedge of water grows alike in depth and reach
            lc = sgn * sum(M.SILL_L) / 2
            front = M.world(M.SILL_P - M.SILL_REACH * f, lc, M.FLOOR_C)  # where its level surface meets the floor
            back = M.world(M.SILL_P, lc, M.FLOOR_C + M.SILL_H * f)  # and the sill
            m.set_visible(head + "sill_water_" + side, w.sill[side] > 0.01)
            self._pose(head + "sill_water_" + side, (front + back) / 2 - M.O, (max(f, 0.05), 1, 1))
        for side, s in (("left", "L"), ("right", "R")):
            m.set_visible(head + "vestibule_water" + s, w.vestibule[side] > 0.02)
        on = w.flow > 0 and w.last not in ("", "away")
        end = np.asarray(w.last_point, float)
        for j in range(14):
            m.set_visible(marks + f"jet{j}", bool(on))
            if on:
                ph = (j / 14 + steps_phase(self)) % 1.0
                self._pose(marks + f"jet{j}", tip * (1 - ph) + end * ph)
        for j in range(8):
            m.set_visible(marks + f"draw{j}", bool(w.sucking))
            if w.sucking:
                pt = M.mouth(ej_tip)
                src = M.world(pt[0] + 3 * math.sin(j), pt[1] + 3 * math.cos(j), M.FLOOR_C + 0.5)
                ph = (j / 8 + steps_phase(self)) % 1.0
                self._pose(marks + f"draw{j}", src * (1 - ph) + ej_tip * ph)

    def step(self, env) -> None:
        dt = float(env.unwrapped.step_dt)
        bulb, ej = m.state(env, "bulb"), m.state(env, "ejector")
        Rb, Re = bulb.rotation(), ej.rotation()
        tip = bulb.pos + Rb @ np.array([0, 0, P.TIP_Z])
        axis = Rb @ np.array([0, 0, -1.0])
        ej_tip = ej.pos + Re @ P.EJ_TIP_LOCAL
        fingers = m.fingertips(env)
        # the bulb's squeeze: the right fingers' gap across it, while it is between them
        base = env.unwrapped
        robot = base.scene["right_robot"]
        opening = abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
        mid = fingers[12:].mean(axis=0)
        self.held_bulb = bool(np.linalg.norm(mid - bulb.pos) < 0.02)
        self.squeeze = max(0.0, (M.BULB_D - 2 * (opening + P.PAD_OFFSET)) * 1000) if self.held_bulb else 0.0
        far = np.linalg.norm(tip - M.O) > 0.15  # over its holder's drip cup, not over the patient or the towel
        self.water.step(dt, self.squeeze, tip, axis, ej_tip, away=far)
        self.draw(tip, ej_tip)
        # where the tools are
        out = all(float((p - M.LIP_CENTRE) @ M.LIP_NORMAL) > 0.010 or np.linalg.norm(p - M.LIP_CENTRE) > 0.08 for p in (tip, ej_tip))
        home = {}
        for name, st in (("bulb", bulb), ("ejector", ej)):
            d = st.pos - HOME[name]
            home[name] = bool(np.linalg.norm(d) <= HOME_DIST and _tilt_deg(st.rotation()) <= HOME_DEG)
        clear = clearance(fingers, bulb, ej)  # both grippers' fingers from both tools and both holders
        raw = M.score(self.water, out, all(home.values()) and clear >= RELEASE)
        self.full_s = self.full_s + dt if raw == 1.0 else 0.0
        self.value = 1.0 if self.full_s >= FULL_HOLD_S - 1e-6 else min(raw, 0.75)
        w = self.water
        self.report = dict(score=self.value, held_bulb=self.held_bulb, squeeze_mm=round(self.squeeze, 2), dispensed=round(w.dispensed, 3), landing=w.last, dose=[round(d, 3) for d in w.dose],
                           film={k: round(v, 3) for k, v in w.film.items()}, vestibule={k: round(v, 3) for k, v in w.vestibule.items()},
                           sill={k: round(v, 3) for k, v in w.sill.items()}, throat=round(w.throat, 3), spill=round(w.spill, 3), dripped=round(w.dripped, 3),
                           trap=round(w.trap, 3), in_mouth=round(w.in_mouth(), 3), sucking=w.sucking, tools_out=bool(out), home=home, clear_mm=round(clear * 1000, 1),
                           tip=np.round(tip, 4).tolist(), opening_mm=round(opening * 1000, 2), target_mm=round(abs(float(m._np(robot.data.joint_pos_target)[0, robot.find_joints('joint7')[0][0]])) * 1000, 2), bulb_pos=np.round(bulb.pos, 4).tolist(), ej_pos=np.round(ej.pos, 4).tolist(), bulb_hand=np.round(mid, 4).tolist(), ejector_hand=np.round(fingers[:12].mean(axis=0), 4).tolist(), ejector_tip=np.round(ej_tip, 4).tolist(), balance=round(w.balance(), 6))
        if os.environ.get("DENTAL_TRACE") and int(round(w.t / dt)) % 10 == 0:
            print("[dental]", self.report, flush=True)


MOUTH_SHOT = os.environ.get("DENTAL_MOUTH_SHOT")  # a path: write a close-up still of the open mouth there (not a policy camera)


class MouthShot:
    """The README's close-up still of the open mouth (run_demo.py --mouth-shot, on a run of its own that does nothing else).
    On that run only, the overhead camera is moved in front of the face to look into the mouth, and one picture is written
    once the renderer has settled. Scored runs never move a camera."""

    def __init__(self, path: str, at_step: int = 30):
        self.path, self.at_step, self.n = path, at_step, 0

    def step(self, env) -> None:
        if self.n < 0:
            return
        import torch

        self.n += 1
        scene = env.unwrapped.scene
        cam = scene["top_cam"]
        if self.n == 1:
            origin = m._np(scene.env_origins[0])
            dots = M.mouth(M.SPOTS).mean(axis=0)  # the three dots, seen from above the lower lip through the opening's middle
            eye = M.world(*(dots + 3.3 * (np.array([-3.0, 4.0, 22.0]) - dots))) + origin
            target = M.world(*dots) + origin
            cam.set_world_poses_from_view(torch.tensor([eye], dtype=torch.float32), torch.tensor([target], dtype=torch.float32))
        if self.n < self.at_step:
            return
        img = m._np(cam.data.output["rgb"][0])[..., :3].astype(np.uint8)
        if img.std() < 5:
            return  # not rendered yet
        from PIL import Image

        h, w = img.shape[:2]  # the middle half of the picture, where the dots are, at 1280 x 720
        Image.fromarray(img[h // 4: h - h // 4, w // 4: w - w // 4]).resize((1280, 720), Image.LANCZOS).save(self.path)
        print(f"[dental] mouth close-up written to {self.path}", flush=True)
        self.n = -1


def steps_phase(trial) -> float:
    return (trial.water.t * 3.0) % 1.0


TRIAL = Trial()
TRIAL.reset()
m.on_reset(TRIAL.reset)
m.on_step(TRIAL.step)
if MOUTH_SHOT:
    m.on_step(MouthShot(MOUTH_SHOT).step)


def score(objects) -> float:
    return TRIAL.value


def success(objects) -> bool:
    steptime.tick()
    return TRIAL.value == 1.0  # already held for FULL_HOLD_S inside Trial.step


def demo_script(ctx):
    import dentaldemo

    yield from dentaldemo.script(ctx)
