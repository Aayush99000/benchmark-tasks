"""Run the scripted reference demo through Inspect Robots' own CLI, as any policy runs.

    python run_demo.py OUT_DIR [--width 640 --height 360]

The demo is closed loop (dentaldemo.py): each control step it reads the poses of the bulb syringe and the saliva
ejector, which the cameras see, and its own arms' joints. It is registered as the policy
``dental_rinse_demo`` and run with the same embodiment, task file, scorer and step limit as any other policy
(``-E object_jitter=0``, 20 Hz), through the same guards: its commands stay within the bounds and per-step change
that the guards allow, so they reach the arms unchanged. A learned policy gets only the cameras and joints.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def register(mode: str) -> str:
    from inspect_robots.registry import register as reg
    from inspect_robots.types import Action, ActionChunk
    from yam_arena.scripted import YamScriptedPolicy

    import demokit
    import mechkit

    class _Emb:
        """What demokit's Ctx reads from an embodiment: the Isaac Lab env and the task's object names."""

        def __init__(self, names):
            self.task = type("T", (), {"names": names})()

        @property
        def _env(self):
            return mechkit.env()

    class DentalRinseDemo(YamScriptedPolicy):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            self.info = replace(self.info, name="dental_rinse_demo")

        def reset(self, scene):
            os.environ["TASK_DEMO"] = mode
            module = self._task.module
            names = tuple(o.name for o in module.OBJECTS)
            self.ctx = None
            self.module, self.names = module, names
            self.done = False

        def act(self, observation):
            if self.ctx is None:
                self.ctx = demokit.Ctx(_Emb(self.names), observation, mode)
                self.script = self.module.demo_script(self.ctx)
            self.ctx.obs = observation
            self.ctx.t = float(observation.state_time)
            if not self.done:
                try:
                    next(self.script)
                except StopIteration:
                    self.done = True
            meta = {"request_stop": True} if self.done else {}
            return ActionChunk(actions=[Action(data=self.ctx.command(), meta=meta)], control_hz=20.0)

    reg("policy", "dental_rinse_demo")(DentalRinseDemo)
    return "dental_rinse_demo"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", type=Path)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--mode", default="full")
    ap.add_argument("--no-frames", action="store_true", help="score only, without storing camera frames")
    ap.add_argument("--mouth-shot", type=Path, help="write a close-up still of the open mouth here (README): on this run only, the overhead camera is moved in front of the mouth; use with --mode none")
    a = ap.parse_args()
    task_json = json.loads((HERE / "task.json").read_text())
    steps = int(re.search(r"[\d,]+", task_json["run_limit"]).group().replace(",", ""))
    a.out.mkdir(parents=True, exist_ok=True)
    os.environ["TASK_DEMO"] = a.mode
    if a.mouth_shot:
        os.environ["DENTAL_MOUTH_SHOT"] = str(a.mouth_shot.resolve())
        import demokit

        demokit._render_at(a.width, a.height)  # the cameras really render at --width x --height on this run
    policy = register(a.mode)
    from inspect_robots.cli import main as cli

    args = ["run", "--embodiment", "yam_arena", "--policy", policy, "-E", f"task={HERE / 'task.py'}", "-E", "object_jitter=0",
            "-E", "control_hz=20", "-E", f"cam_width={a.width}", "-E", f"cam_height={a.height}",
            "--instruction", task_json["instruction"], "--scorer", "task_score", "--max-steps", str(steps),
            "--log-dir", str(a.out)] + ([] if a.no_frames else ["--store-frames"])
    result = cli(args)
    return 0 if result in (None, 0) else int(result)


if __name__ == "__main__":
    raise SystemExit(main())
