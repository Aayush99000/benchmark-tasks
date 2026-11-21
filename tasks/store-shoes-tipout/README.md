# Put a pair of clogs away in a tip-out shoe cabinet

A slim tip-out shoe cabinet stands on the table, and a pair of foam clogs lies in front of it. The instruction asks for
both clogs put away and the door shut. Its whole front is one door, hinged along its bottom edge. Pulled by its lip, it
tips forward to open a bin that is shallow at the bottom and deep at the top. A clog's toe is thin and its heel thick,
so a clog stood in toe first sinks to the bin's floor, its heel 4 cm below the lip. Stood in heel first, its thick heel
catches halfway down, its toe sticks 7 cm above the lip, and the door stops against it. The robot must open the door,
turn each clog toe-down in the hand, lower it in, and push the door back past its balance point so it falls shut.

## The real step
From a flip-down shoe cabinet's manual (VASAGLE MLBS101LB01, troubleshooting, "Flip-down doors do not close
properly"): "Ensure no shoes are blocking the door mechanism." Done by: nearly everyone with a hallway shoe cabinet.
What goes wrong: a shoe put in the wrong way, or lying flat, holds the door open; forced, a real door cracks its hinge
pins. A photo of the cabinet shows the door ajar and the shoe sticking out.

## The cabinet, the clogs, and what was simplified
| What | Value used | Source |
|---|---|---|
| cabinet | IKEA TRONES shoe/storage cabinet, white, 52.1 x 18.1 x 39.1 cm ($39.99 per 2-pack) | IKEA US product page (20 1/2 x 7 1/8 x 15 3/8 in) |
| tray on top | 2 cm deep | IKEA's dimension drawing |
| bin | 47 cm wide inside; 15 cm deep at the top | IKEA's dimension drawing |
| bin's depth at the bottom | 7 cm | **estimate**, to measure |
| door | tips out to 40 degrees, 0.7 kg with the bin; a lip 2 cm proud along its top edge | **estimates** from IKEA's photo and drawing (its top edge curls outward), to measure |
| clogs | Crocs Classic Clog, men's US 9, 198 g each | weight: Zappos listing, "based on a single item" |
| clog outside | 29 x 10 x 9.5 cm; toe 6.5 cm thick, heel 9.5 cm; heel cup 8.5 cm wide, open on top, walls 8 mm | **estimate** (Crocs' charts give 263 mm heel to toe for the foot), to measure |

How the sim models it:
- **The cabinet** is a fixed articulation (`mechanism.py`, MIT, from robocurve#223, copied unchanged): its sides, back,
  bottom and tray are fixed boxes.
- **The door and bin** are one link on a revolute hinge, 0 to 40 degrees. The bin's back is a staircase of eight steps
  from 7 to 15 cm deep, because the module's boxes can't be tilted.
- **The door's weight** does the rest: pulled past about 20-25 degrees (its middle over the hinge), it falls open to its
  stop and stays; pushed back below that, it falls shut.
- **Each clog** is one rigid body of six boxes: sole, toe box, middle, and a heel cup of three walls, open on top. Its
  outside box (the object the task-file format sees) has collisions and drawing turned off.
- **Stillness** is judged from each clog's position over the last 6 control steps (moved under 1 mm, turned under 1
  degree). A clog asleep in the bin kept reporting a stale 87 mm/s, so the reported velocity isn't used.
- **Friction:** the cabinet's plastic has friction 0.35 / 0.30 (an estimate), combined with "min", so a clog slides
  down the tipped bin while the grippers keep 1.2 / 1.0 on it.

What we simplified:
- The clogs are rigid; real Croslite foam gives a little.
- They have no foot opening or strap.
- The bin's back is stepped, not smooth.
- The door has no catch.
- The cabinet stands on the table; IKEA says to fix it to a wall.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| cabinet | white tip-out shoe cabinet, 52.1 wide x 18.1 deep x 39.1 tall, front towards the arms, door shut (fixed) | front face at x 45, centred on y 0 |
| left clog | red foam clog, 29 x 10 x 9.5 cm, 198 g, sole down, toe towards the arms, heel towards the cabinet's front | middle at (25, 33), just left of the cabinet |
| right clog | the same | middle at (25, -33), just right of the cabinet |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
A clog is **in** when every corner of its parts is inside the bin: behind the door, in front of the bin's back at
that height, between its sides, above its floor, and no more than 5 mm above its lip. Taken in the door's own frame,
so it works at any door angle. **Hands off:** no point along either gripper's finger pads, and neither fingertip's
end, within 3 cm of the cabinet.

| Score | When, at the end |
|---|---|
| 1.0 | both clogs in, the door within 2 degrees of shut, both hands off, the clogs still (under 1 mm and 1 degree over 0.3 s); held for 1 s, which ends the trial |
| 0.75 | both in, the door within 15 degrees of shut (or shut with a hand still near) |
| 0.5 | both in, the door open more than 15 degrees |
| 0.25 | one in |
| 0 | neither in |

In a real trial: open the door after the trial; a clog is in if it lies wholly inside the bin with nothing above the
lip. A photo of the shut cabinet shows the door's gap.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder (it holds two Python files: the task file and
`mechanism.py`, which the task file imports):

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=store_shoes_tipout.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put both clogs away in the shoe cabinet: tip its door open by the top lip, stand each clog in the bin toe first, then tip the door shut." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/store-shoes-tipout-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=store_shoes_tipout.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put both clogs away in the shoe cabinet: tip its door open by the top lip, stand each clog in the bin toe first, then tip the door shut." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/store-shoes-tipout-demo

# A wrong way: each clog dropped in as it was picked up, lying flat
SHOES_DEMO=flat inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=store_shoes_tipout.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put both clogs away in the shoe cabinet: tip its door open by the top lip, stand each clog in the bin toe first, then tip the door shut." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/store-shoes-tipout-flat

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/store-shoes-tipout-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. With `SHOES_DEBUG=<file>` set, each control step adds a line to that file: the
grade, the door's angle, each clog's place and whether it is in, its top and bottom in the door's frame, and the
grippers' tips. `TOP_CAM_VIDEO=<file.mp4>` records the top camera at full size (and its first frame as the setup
screenshot).

## The scripted demo
It plays a fixed plan from the fixed layout with the arm's own inverse kinematics, and reads nothing from the
simulator.

1. The left hand pinches the door's lip near its left end, the fingers closing vertically. It carries the lip round
   the hinge to 24 degrees, turning the grip with the door. Then it lets go and slips sideways off the lip's end; the
   door falls open to its 40-degree stop.
2. Both hands pinch their clogs across the heel cup from above. Each lifts its clog and turns it toe-down in three
   steps (straight down, 45 degrees, level), because the straight-down grip reaches only up to about 20 cm above the
   table here.
3. Each hand carries its clog high, crosses in front of the open lip, and moves in over the opening. It lowers the
   clog until the toe meets the tipped door, then moves in and down along the bin. The toe slides down the door, and
   the clog, free to swing in the pinch, comes into line with the bin. The hand lets go and lifts away; the clog
   slides to the bin's floor.
4. The right hand pushes the lip back round the arc; past the door's balance point it falls shut. Both hands move away.

Tried and dropped along the way (official sim, 2026-11-20):
- fingertips hooked under a flat lip: they slid off it;
- the clogs in front of the cabinet: the open door hangs over them;
- a pinch on the heel's 8 mm back wall: the fingers closed beside it;
- the clog dropped from a tilted grip: it swings straight down in the pinch, lands on the lip and falls out.

## Runtime and seeds
- **Official runs, 2026-11-21, this version:**
  - The scripted demo scored 1.0 twice, each time ending on success at step 629 (31 s): both clogs in the bin, the door
    shut at 0.0 degrees, the hands clear.
  - The wrong way (`SHOES_DEMO=flat`, each clog dropped in as it was picked up, lying flat) scored 0: both clogs fell back
    out.
  - Dream-1 scored 0.0 over the full 1,200 steps: its arms reached up to the cabinet's top corners and never touched the
    door or the clogs.
- The scoring, geometry and reach checks run without the simulator: `python -m pytest -q test_store_shoes_tipout.py`
  (21 tests). They include the trap from geometry alone: stood toe first, a clog's top is 4 cm below the lip; heel
  first, it stands 7 cm above it.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- `tools/check_reach.py` is the offline reach scan used for the layout.
