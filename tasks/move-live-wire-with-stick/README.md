# Move a live wire off a walkway with an insulated stick

A red "live" cable (8 mm PVC cord, 30 cm, black plug on its free end) leaves a grey junction box beside a yellow
taped walkway (15 cm wide) and lies diagonally across it, its plug just past the far edge. A blue insulated stick
(30 x 2 x 2 cm hollow fibreglass rod) lies to the right. The robot must use only the stick to push the whole cable
off the walkway so that the plug ends up in a green taped parking zone on the near side, **without the grippers or
any other part of the arms ever touching the cable or its plug**. This mirrors how electricians and line workers
clear energised conductors with a hot stick: pick the right tool, keep the hands clear, and manipulate a deformable
cable non-prehensilely. Real trials use an unpowered cable, so they are safe.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "The red cable is live. Without touching the cable or its plug with the grippers or any part of the arms, use the blue insulated stick to push the whole cable off the yellow walkway so that its plug ends up inside the green parking zone."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop (reads the live cable and stick states): the right arm picks the stick up by its far end, lays it on
the table alongside the cable on its far side with the gripper beyond the plug, and sweeps it clockwise round the
junction box in 4-degree steps so that the stick's side pushes the cable off the walkway; the plug lands in the
green zone. Success holds for 1 s and ends the episode at step 222 (~11 s); task_score 1.0.
`demo/` holds its videos, `setup.png` and `result.json`.

Shortcut checks (each scores exactly 0): `LIVEWIRE_CHEAT=none` (do nothing), `LIVEWIRE_CHEAT=hand` (grasp the
plug and carry it into the zone by hand), `LIVEWIRE_CHEAT=sweep_hand` (the same clockwise sweep, pushing the cable's middle with the bare closed
gripper instead of the stick).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if any robot link (fingers, gripper housing or arm) ever touched the cable or plug. In sim this is read
  from PhysX contact reports at every 240 Hz physics substep: any robot-link/cable-or-plug contact point with a
  separation of 1 mm or less marks the episode as touched (stick-finger and stick-cable contacts do not count).
  The flag resets at every episode start.
- **1.0** if no part of the cable surface or plug overlaps the walkway tape in top view, the plug centre is inside
  the green zone, and the cable lies on the table (every segment at most 2 cm above it).
- Otherwise partial credit: with L = cable length over the walkway (top view, height ignored; 20.6 cm at the start),
  if at least 3 cm has been cleared, **0.8 x (20.6 - L) / 20.6**; else **0** (so doing nothing scores 0).
- `distance()`: cable length still over the walkway (m); 1.0 once the robot has touched the cable.

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz). One trial takes ~3-5 min wall time on one GPU including Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight), so every trial
  starts identically.
