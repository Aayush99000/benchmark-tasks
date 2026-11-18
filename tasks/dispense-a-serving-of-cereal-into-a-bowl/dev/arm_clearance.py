"""Offline: how close each arm's collision hulls come to the dispenser during the scripted demo (kinematic replay,
no Isaac Sim). Created 11_16_26 (America/New_York). Run from the task folder in the simulator's venv."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import task  # noqa: E402
from cereal_dispenser import geometry as G  # noqa: E402
from yam_arena import analysis as A  # noqa: E402
from yam_arena.constants import ARM_JOINTS  # noqa: E402


def dispenser_gap(pts_d, skip_knob=False):
    """Smallest clearance (m, negative = inside) of D-frame points to the canister, funnel, housing, board."""
    x, y, z = pts_d[:, 0], pts_d[:, 1], pts_d[:, 2]
    out = []
    # canister and funnel as one vertical cylinder of the canister's outer radius above the funnel's bottom
    r = np.hypot(x, y)
    inz = (z > G.FUNNEL_Z0) & (z < G.CANISTER_TOP + G.LID_T)
    out.append(np.where(inz, r - (G.CANISTER_R + 0.006), 1.0))
    # housing and spout box
    hx, hy = G.X_END + G.T_SHELL, G.R_BORE + G.T_SHELL
    inside = np.maximum.reduce([np.abs(x) - hx, np.abs(y) - hy, np.maximum(G.SPOUT_Z - z, z - (G.WHEEL_Z + hy))])
    out.append(inside)
    if not skip_knob:
        k = np.maximum.reduce([np.maximum(G.KNOB_X0 - x, x - (G.BAR_X + G.BAR["h"] / 2)), np.hypot(y, z - G.WHEEL_Z) - G.KNOB["r"]])
        out.append(k)
    bx, bh = (G.BOARD["x0"] + G.BOARD["x1"]) / 2, (G.BOARD["x1"] - G.BOARD["x0"]) / 2
    out.append(np.maximum.reduce([np.abs(x - bx) - bh, np.abs(y) - G.BOARD["w"] / 2, z - G.BOARD["h"]]))
    names = ["canister", "housing"] + ([] if skip_knob else ["knob"]) + ["board"]
    arr = np.array(out)
    dispenser_gap.last = names[int(np.unravel_index(arr.argmin(), arr.shape)[0])]
    return np.min(out, axis=0)


plans = task.demo()
for arm, base in (("left", task.LEFT_BASE), ("right", task.RIGHT_BASE)):
    plan = plans[arm][0]
    worst = (1.0, None, None)
    for k in range(0, len(plan.q), 2):
        poses = A.fk_link_poses_world(dict(zip(ARM_JOINTS, plan.q[k])), base)
        for link, hull in A.world_hulls(poses):
            if arm == "right" and link in ("tip_left", "tip_right", "gripper") and plan.labels[k] in ("in", "grip", "turn", "open"):
                continue  # the fingers are meant to touch the knob's bar
            g = dispenser_gap(task.to_dispenser(hull.vertices)).min()
            if g < worst[0]:
                worst = (g, link, plan.labels[k] + " / " + dispenser_gap.last + f" / step {k}")
    print(f"{arm}: closest approach {worst[0] * 1000:6.1f} mm ({worst[1]}, during '{worst[2]}')")
