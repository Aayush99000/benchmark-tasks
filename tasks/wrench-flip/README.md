# Tighten a locknut where the wrench swings only 44 degrees: turn the open-end wrench over for each new bite

A nylon-insert locknut sits on a level bolt at the bottom of a narrow slot in a steel bracket, half a turn short of
seated. A 10 x 13 mm double open-end wrench stands in a rack nearby. The instruction is only "Tighten the nut with the
wrench." The slot's walls leave the handle about 44 degrees of swing. A hex nut's flats repeat every 60 degrees, so
after one stroke the jaw no longer fits the flats anywhere in the swing, unless the wrench is turned over: its head is
angled 15 degrees to the handle, so turned over it fits 30 degrees further round. Each stroke after the first needs
the wrench off, turned over and back on, six strokes of about 30 degrees in all.

## The real step
Tekton, [24 mm Combination Wrench](https://www.tekton.com/24-mm-combination-wrench-18295): to reduce the working swing
arc in tight spaces, the open end is angled 15 degrees "so the wrench can be flipped over with each rotation". Done
by: automotive service technicians and mechanics (BLS: about 805,600 jobs in 2024), and maintenance workers.

What goes wrong in real life: a nut left short of seated. A nylon-insert nut can't be spun on by hand, so a nut the
wrench can't reach is left loose. Here the paint line on the nut shows it: it points at the red mark below the nut
only once the nut is seated.

What we simplified: the bracket and the rack are fixed. The jaw on the nut is a model (below), as in the accepted
compression-joint task, rather than contact between a rigid jaw and a rigid hex. The nut doesn't travel along the
thread.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=wrench_flip.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Tighten the nut with the wrench." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/wrench-flip-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=wrench_flip.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Tighten the nut with the wrench." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/wrench-flip-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/wrench-flip-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the nut has been seated for 1 s; otherwise it
scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| bracket | grey steel, fixed. An M8 bolt sticks out of its back plate level, 4.5 cm above the table, pointing towards the left arm (along (-0.87, 0.5)), at the bottom of an upright slot: two walls 1 cm thick, their inner faces 3.27 cm either side of the bolt, from the slot's floor (2 cm up) to 11.5 cm, reaching 2 cm out past the nut | (28, 12) |
| washer | steel, 16 mm across, 1.6 mm thick, against the back plate | on the bolt |
| nut | zinc-plated steel M8 nylon-insert locknut (DIN 985), 13 mm across the flats, 8 mm long; a white paint line from its centre to one flat points up; half a turn short of seated; 0.5 N m of drag | on the bolt, against the washer |
| seated mark | a red line on the back plate below the nut: where the paint line points once seated | on the bracket |
| wrench | chrome steel double open-end wrench, 13 mm and 10 mm, both heads angled 15 degrees to the handle, 16 cm long, handle 14 x 5 mm, 70 g; head down, handle straight up, faces square to the bolt | in the rack |
| rack | wood, fixed, 5 x 7 cm, 4.2 cm tall, with a 7 x 37 mm pocket 3.2 cm deep holding the 13 mm head, and a 6 mm step in it under one prong so the wrench stands straight | (20, 26) |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
Measured on the nut: how far it has been turned clockwise (tightening, seen from the bolt's end) from its start, in
degrees, as the jaw model turned it; turning it back counts against it.

| Score | When, at the end |
|---|---|
| 1.0 | the nut seated (at least 178 of the half turn to its stop), held for 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.75 | turned at least 150 degrees |
| 0.5 | at least 100 degrees |
| 0.25 | at least 60 degrees |
| 0 | anything else |

**Why credit starts at 60 degrees:** one stroke without turning the wrench over turns the nut at most about 51
degrees: the 44 degree swing, plus the jaw's 7 degree fit tolerance. So any credit needs the wrench turned over at
least once. `test_wrench_flip.py` plays every one-face strategy (on wherever the jaw fits, to the wall, repeated) and
checks that it stays under 55 degrees.

The nut turns only with the 13 mm jaw on it, so nothing else (the fingers, the 10 mm end, pushing the nut) scores.

## The jaw model
`WrenchOnNut` in `wrench_flip.py` runs before every physics substep (240 Hz). The 13 mm jaw is on the nut when all of
these hold:
- the wrench lies in the nut's plane (its face within 12 degrees of the bolt's axis, either face out);
- the nut is between its prongs: within 3 mm of the jaw's middle across the mouth, and from 1.5 mm past the jaw's seat
  to 8 mm out towards the mouth (the prongs are 18.6 mm deep), and within 4.5 mm of the jaw along the bolt;
- the jaw is lined up with a pair of flats within 7 degrees.

The jaw is turned 15 degrees to the handle: one way with one face out, the other way with the wrench turned over. So
the handle angle at which it fits moves 30 degrees when the wrench is turned over.

While on, the jaw is held on the nut across its mouth and drawn lightly (at most 1.5 N) onto it. The nut turns with
the wrench against 0.5 N m of drag, and stops when seated. Slid 10 mm out of its mouth, or 6.75 mm off along the bolt,
the jaw comes off. The nut
is drawn as a hexagon but collides as a round core inside its flats, so the held jaw never fights the contact solver
(the accepted compression-joint task does the same).

## The scripted demo
The left arm does all of it, open loop:
1. It takes the wrench out of its rack by the handle, its fingers closing across the handle's 5 mm thickness.
2. For each stroke, it brings the wrench in front of the slot, lines it up, and brings the jaw in along the bolt beside
   the nut, 16 mm out from it; then it slides the jaw on across the flats and swings the handle clockwise to 1 degree
   inside the slot's wall.
3. It backs the jaw off along the bolt, over the nut's end, and out of the slot. It turns the wrench over by turning its
   straight-down grip half round about the vertical (yaw -120 to +60 degrees), and goes in again.

Six strokes of about 30 degrees seat the nut. The grip tilts with the handle about the bolt's axis: the stock waypoints
only point straight down, so `wrench_flip.py` adds `TiltWaypoint` and has `yam_arena.demo.ArmPlan` honour its tilt.
The stroke plan allows for the arm as the official sim measured it (each `WHY` comment in `wrench_flip.py` gives the
run): the jaw goes on about 1.3 degrees below its aim, and a stroke stops about 1.2 degrees short of it. The right arm
stays clear.

## Part kit
`fittingkit.py` is copied unchanged from the accepted robocurve/benchmark-tasks task `tasks/compression-joint` (commit
de817b9), under the MIT licence its authors state in that task's README. It builds rigid bodies from boxes, poses
bodies and applies forces from a model every physics substep, and hooks yam_arena's scene and reset.

## Runtime and seeds
- Official runs (2026-11-16, the Robocurve Brev launchable, 2,400 steps, `object_jitter=0`): the scripted demo scores
  1.0: six strokes seat the nut, and the trial ends at 62.6 s (about 1,250 steps) once it has stayed seated for 1 s.
  A second run of the demo (a state trace) also ended seated at 62.6 s. Dream-1 scores 0.0 over the full 2,400 steps.
- The scripted demo's plan is 66.6 s of simulated time.
- The scoring, jaw model, reach and step-guard checks run without the simulator:
  `python -m pytest -q test_wrench_flip.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
