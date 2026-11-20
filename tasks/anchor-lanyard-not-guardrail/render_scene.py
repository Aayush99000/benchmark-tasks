#!/usr/bin/env python3
"""Render stills of the start layout: the three policy cameras, plus close-ups from an extra still camera.

    python render_scene.py OUT_DIR [--width 1280 --height 720]

No policy runs and nothing is scored: the arms hold their ready pose while the scene settles. The extra camera is
for people looking at the scene; the policy only ever sees top_cam, left_cam and right_cam.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

SETTLE_STEPS = 40
HERE = Path(__file__).resolve().parent


class StillCamera:
    def __init__(self, emb, width: int, height: int):
        import omni.replicator.core as rep
        from pxr import Gf, Sdf, UsdGeom

        self.emb, self.shape = emb, (height, width, 3)
        self.camera = UsdGeom.Camera.Define(emb._base.sim.stage, "/World/StillCamera")
        self.transform = self.camera.AddTransformOp()
        self.camera.CreateFocalLengthAttr(24.0)
        self.camera.CreateHorizontalApertureAttr(20.955)
        self.camera.CreateVerticalApertureAttr(20.955 * height / width)
        self.camera.CreateClippingRangeAttr(Gf.Vec2f(0.005, 30.0))
        self.camera.GetPrim().CreateAttribute("omni:scenePartition", Sdf.ValueTypeNames.Token).Set("env_0")
        self.product = rep.create.render_product(str(self.camera.GetPath()), (width, height))
        self.rgb = rep.AnnotatorRegistry.get_annotator("rgb", device="cpu")
        self.rgb.attach([self.product])

    def shot(self, eye, target, focal: float) -> np.ndarray:
        from isaaclab_physx.renderers.isaac_rtx_renderer_utils import ensure_isaac_rtx_render_update
        from pxr import Gf

        self.transform.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse())
        self.camera.GetFocalLengthAttr().Set(focal)
        for _ in range(12):
            self.emb._base.sim.render()
            ensure_isaac_rtx_render_update(force=True)
        frame = np.ascontiguousarray(self.rgb.get_data()[..., :3])
        if frame.shape != self.shape:
            raise RuntimeError(f"bad still shape {frame.shape}")
        return frame


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("out", type=Path)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(HERE))

    import cv2
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    emb = yam_arena_embodiment(task=str(HERE / "task.py"), object_jitter=0, control_hz=20, cam_width=args.width,
                               cam_height=args.height)
    obs = emb.reset(Scene(id="scene", instruction="scene"), seed=0)
    T = emb.task.module
    hold = np.array(obs.state["joint_pos"], float)
    for _ in range(SETTLE_STEPS):
        obs = emb.step(Action(data=hold)).observation
    images = {k: np.asarray(v) for k, v in obs.images.items()}

    import rigkit

    env = emb._env
    report = {}
    for name in ("hook", "hook_gate", "hook_lock", "pack"):
        p, q, v = rigkit.pose(env, name)
        report[name] = {"pos": p.round(4).tolist(), "quat_xyzw": q.round(4).tolist(), "speed": float(np.linalg.norm(v))}
    for name in ("hook_gate", "hook_lock"):
        report[name]["hinge_deg"] = round(rigkit.hinge_angle(env, name), 2)
    report["start"] = {"hook": T.HOOK_POS.round(4).tolist(), "anchor_seat": T.ANCHOR_SEAT.round(4).tolist(),
                       "worker_dring": T.WORKER_DRING.round(4).tolist()}

    cam = StillCamera(emb, args.width, args.height)
    zd = T.ZD
    shots = {
        "overview": ((-0.75, -0.20, zd + 1.25), (0.45, 0.10, zd + 0.40), 18.0),
        "worker-side": ((1.55, -1.95, zd + 1.25), (0.45, -0.35, zd + 0.72), 18.0),
        "hook-and-post": ((0.30, -0.56, zd + 0.42), (0.54, -0.17, zd + 0.03), 32.0),
        "anchor": ((0.16, 1.00, zd + 0.48), (0.41, 0.53, zd + 0.16), 45.0),
        "plan": ((0.37, 0.0, zd + 2.2), (0.3701, 0.0, zd), 32.0),
    }
    for name, (eye, target, focal) in shots.items():
        images[name] = cam.shot(eye, target, focal)
    for name, frame in images.items():
        cv2.imwrite(str(args.out / f"{name}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    (args.out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report), flush=True)
    print("WROTE", sorted(images), "to", args.out, flush=True)
    emb.close()


if __name__ == "__main__":
    main()
