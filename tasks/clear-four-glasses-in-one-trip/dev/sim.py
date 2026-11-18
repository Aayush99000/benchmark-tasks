"""Dev only: several episodes in one simulator session, to try the demo and the shortcuts and to read off the arms' give.

    python dev/sim.py demo demo none half single high held     (under the same environment as an inspect-robots run)

Each word is one episode of the scripted plan, played open loop by the stock yam_scripted policy on the yam_bus
embodiment: "demo" is the full demo, any other word a task.set_variant name. With BUS_SNAPS=<folder> a camera frame is
saved every BUS_SNAP_S seconds (default 2), and the top camera's first frame as the setup image (BUS_CAM_W and BUS_CAM_H
set its size). With BUS_DEBUG=1 every step's measurements are printed. The demo itself reads
nothing from the simulator."""
import json
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
from glass_bussing.embodiment import BusEmbodiment  # noqa: E402
from inspect_robots import Scene  # noqa: E402
from yam_arena.scripted import YamScriptedPolicy  # noqa: E402

INSTR = json.load(open(os.path.join(HERE, "task.json")))["instruction"] if os.path.exists(os.path.join(HERE, "task.json")) else "clear the glasses"
MAX_STEPS = int(os.environ.get("BUS_STEPS", "1800"))
SNAPS = os.environ.get("BUS_SNAPS")
SNAP_S = float(os.environ.get("BUS_SNAP_S", "2"))
CAM = (int(os.environ.get("BUS_CAM_W", "320")), int(os.environ.get("BUS_CAM_H", "176")))


def snap(obs, n, word, t):
    import cv2
    import numpy as np

    os.makedirs(SNAPS, exist_ok=True)
    row = np.concatenate([obs.images[c] for c in ("top_cam", "left_cam", "right_cam")], axis=1)
    cv2.imwrite(os.path.join(SNAPS, f"ep{n}_{word}_{t:05.1f}.jpg"), cv2.cvtColor(row, cv2.COLOR_RGB2BGR))
    if t <= 0.051:  # the start, top camera alone: the setup image
        cv2.imwrite(os.path.join(SNAPS, f"ep{n}_{word}_setup.png"), cv2.cvtColor(obs.images["top_cam"], cv2.COLOR_RGB2BGR))


def main():
    emb = BusEmbodiment(task=os.path.join(HERE, "task.py"), control_hz=20, cam_width=CAM[0], cam_height=CAM[1])
    pol = YamScriptedPolicy()
    pol.bind(emb.info)
    m = emb.task.module
    dt = 1.0 / emb.info.control_hz
    for n, word in enumerate(sys.argv[1:]):
        m.set_variant("" if word == "demo" else word)
        scene = Scene(id=f"ep{n}", instruction=INSTR)
        print(f"EPISODE {n} {word} start", flush=True)
        obs = emb.reset(scene, seed=0)
        pol.reset(scene)
        info, steps, next_snap = {}, 0, 0.0
        for steps in range(1, MAX_STEPS + 1):
            if SNAPS and steps * dt >= next_snap:
                snap(obs, n, word, steps * dt)
                next_snap += SNAP_S
            action = pol.act(obs).actions[0]
            res = emb.step(action)
            obs, info = res.observation, res.info
            if action.meta.get("request_stop") or res.terminated:
                break
        if SNAPS:
            snap(obs, n, word, steps * dt)
        tr = m._trial
        print(f"EPISODE {n} {word} end: steps {steps} score {info.get('score')} success {info.get('success')} carries {m.carries()} "
              f"tipped {tr['tipped']} nested {tr['nested']} off {tr['off']}", flush=True)
    os._exit(0)


main()
