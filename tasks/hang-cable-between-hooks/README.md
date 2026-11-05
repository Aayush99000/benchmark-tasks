# Hang a cable between two hooks with a set sag

Two grey hook posts stand 30 cm apart in front of the robot, each topped by an open U cradle 15 cm above the table;
orange bands on the posts mark 4 cm and 6 cm below the hooks. A free 50 cm black cable with a small plug (end
weight) on each end lies straight on the table in front of the posts. The robot must hang the cable across both
hooks, each end hanging down on the outer side of its hook, with the middle sagging so its lowest point lies
between the bands (4-6 cm below the hook line), and let go. The sag is a global shape target: it is set only by how
much cable is left between the hooks, so the robot has to pull the right amount of cable over each hook. It is
naturally bimanual: each arm takes one end and lays it over its own hook, and the two arms set the slack together.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Hang the black cable across the two hooks: lay it in both cradles with each end hanging down on the outer side of its hook, so that the middle sags between the hooks with its lowest point between the two orange bands on the posts (4 to 6 cm below the hooks). Then let go of the cable."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Closed-loop (reads the live cable state): each arm takes one end plug, lifts the cable, carries the plug from
the inner side of its hook to 9 cm outside it (so the cable runs over the cradle) and lowers it to the hook line,
which drops the cable into both cradles. Holding the plugs, both arms then move outwards or inwards together
until the measured sag is ~6 cm (the cable settles ~1 cm when the plugs are let go and swing down), release, and
move clear. Success holds for 1 s and ends the episode at step 256 (~13 s); task_score 1.0. `demo/` holds its
videos, `setup.png` and `result.json`.

Cheat / failure variants (test only, `HANG_CHEAT=<name> python cabledemo.py task.py out`), all score exactly 0:
`none` (do nothing: the cable stays on the table), `hold` (does the full demo, then the left arm re-grips its plug
where it hangs and keeps holding it: both hooks hung, sag 4.1 cm, would be 1.0 if released), `short` (drapes the cable across both cradles with the
plugs left on top of the cradles, so the ends do not hang past the hooks; the middle drops to the table), `taut`
(pulls the cable straight across the hooks, then lets go: sag ends at ~1.7 cm, well below the 3 cm minimum).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** unless the cable is released: both grippers' centres at least 3 cm from every part of the cable and plugs
  (real trial: no gripper touching the cable).
- A hook is **hung** when part of the cable lies in its cradle slot (between the walls, at most 12 mm above the
  floor) and a cable end hangs on that hook's outer side with its plug centre at least 5 cm below the hook line.
- One hook hung: **0.2**. Both hooks hung: **0.4** plus a sag bonus. Sag = hook line (cradle floor top) minus the
  lowest point of the cable's underside between the two cradles: 4-6 cm **+0.6** (1.0, success); 3-4 or 6-8 cm
  **+0.3** (0.7); more than 8 cm **+0** (0.4). Both hung but sag under 3 cm (pulled taut / laid straight across) **0**.
- Nothing hung (e.g. doing nothing): 0.
- `success()` additionally requires the cable to be at rest. `distance()`: |sag - 5 cm| once both hooks are hung,
  else 1 m.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). One trial takes ~5-10 min wall time on one GPU including the ~1-2 min
  Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight), so every trial
  starts identically.
