# Change a jigsaw's blade in its tool-free clamp

A corded jigsaw, unplugged, lies on its side in its open case with its blade clamp facing the arms, and a new wood
blade stands in a holder beside it, its teeth facing the wrong way. The instruction asks for the blade to be fitted the
way the maker describes: hold the clamp's spring-loaded cap pushed in, push the blade's shank into the clamp with its
teeth towards the saw's nose until it clicks, then let go of the cap and the blade. One hand holds the cap in while the
other turns the blade the right way round and pushes it home.

## The real step
Carpenters and DIYers change jigsaw blades for each material and when one dulls. Bosch,
[Changing the jigsaw blade](https://www.bosch-diy.com/gb/en/all-about-diy/step-by-step-project/changing-the-stitch-saw-blade):
"Pull the red cap of the SDS quick-change system up again with two fingers", "push the saw blade from below into the
slot provided", "As soon as it clicks, it is engaged and firmly seated", "Release the red cap". Done by: carpenters (BLS:
about 959,000 jobs in 2024) and DIYers.

What goes wrong in real life: a blade put in backwards doesn't cut, and one not clicked in can fly out when the saw
starts.

What we simplified: the saw is fixed and unplugged (the maker's first step, taking the battery out or unplugging, is
done); the clamp is a model (below); the shoe's open front is wider than a real one's so the clamp can be reached.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=jigsaw_blade.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the new blade in the jigsaw. Hold the clamp's cap pushed in, push the blade's shank into the clamp with its teeth towards the saw's nose until it clicks, then let go of the cap and the blade." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/jigsaw-blade-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=jigsaw_blade.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the new blade in the jigsaw. Hold the clamp's cap pushed in, push the blade's shank into the clamp with its teeth towards the saw's nose until it clicks, then let go of the cap and the blade." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/jigsaw-blade-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/jigsaw-blade-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the finished state has held for 1 s; otherwise it
scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| jigsaw | corded, unplugged, lying on its right side on its open case's foam insert (5.5 cm thick), fixed; its plunger points at the arms, its nose to the robot's left | case centre (42, 0) |
| clamp | the blade clamp's slot, 10 x 3.5 mm and 14 mm deep, opening towards the arms | the slot's mouth at (34, 0), 9 cm above the table |
| cap | the clamp's red spring-loaded cap, on the nose side of the slot: pushed 3.5 mm in (towards the saw, about 4.4 N) it unlocks the clamp; 5 mm of travel | (34.5, 2.0), 9 cm up |
| shoe | the saw's footplate, standing on edge 2 cm in front of the clamp, 3 cm to the slot's other side | (32, -6.5) |
| blade | a new T-shank wood blade (Bosch T 144 D size: 100 mm long; 7.5 mm wide, 1.5 mm thick), standing shank down in a holder, its teeth facing the arms | (20, -16) |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
From the clamp's model and the parts' poses.

| Score | When, at the end |
|---|---|
| 1.0 | the blade locked in the clamp with its teeth towards the saw's nose, the cap let go, and both grippers more than 2 cm from the blade, held 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.5 | the blade locked with its teeth backwards |
| 0.25 | the blade 10 mm or more into the slot but not locked (the cap still pushed in) |
| 0 | anything else |

- The blade is in the slot when its shank's end is within 2.5 mm of the slot's middle and it lies within 15 degrees of
  the slot's line.
- With the cap not pushed in, the clamp's catch stops the blade 3 mm in (a stiff stop, up to 40 N).
- With the cap pushed 3.5 mm in, the blade goes on; 10 mm in, it clicks.
- Once the cap comes back past its unlock point, a clicked blade is locked where it is.

## The clamp model
`Clamp` in `jigsaw_blade.py` runs before every physics substep (240 Hz). It reads the cap's travel and where the
blade's shank is; until the blade has clicked it applies the catch's stop force when the cap isn't pushed in; it notes
the click, and on the lock it holds the blade where it is (its pose is written back every substep). The cap is a rigid
body on a sprung PhysX prismatic joint in the saw (`fittingkit.Slider` with a drive: 1,250 N/m).

## The scripted demo
Open loop, both arms, on one shared timeline:
1. The right hand comes down from high over the blade and pinches its flats 18 mm from its tip.
2. It lifts the blade out, turns it a quarter turn about the vertical so its teeth face the saw's nose, then tips it
   over (by way of a pose turned 30 degrees) to lie level, in line with the slot, its shank 3 cm short of the mouth.
3. The left hand, fingers closed, reaches the cap from the nose side (turned 35 degrees, pointing 20 degrees down,
   rolled a quarter turn) and pushes it in, and holds it.
4. The right hand pushes the shank into the slot to its bottom; the clamp clicks.
5. The left hand lets the cap go (the blade locks), the right lets go of the blade, and both move clear.

The poses were checked offline against the YAM's gripper and wrist collision hulls (URDF forward kinematics,
`plan_check.py`): neither hand touches the saw, the cap's housing or the holder, the hands stay at least 17 mm apart, and
the blade stays in line with the slot as it goes in. Every pose that touches something is reached within 0.5 mm and
0.1 degree; while the blade is tipped over in mid-air the right hand lags its orientation by up to 6 degrees.

## Part kit
`fittingkit.py` is copied from the accepted robocurve/benchmark-tasks task `tasks/compression-joint` (commit de817b9),
under the MIT licence its authors state in that task's README, with one addition noted in its docstring: `Slider`
takes an optional spring drive (`stiffness`, `damping`, `target`), used here for the cap. `plan_check.py` is the
offline check of the demo described above.

## Runtime and seeds
- Official runs (2026-11-17, the Robocurve Brev launchable, 1,200 steps, `object_jitter=0`): the scripted demo scores
  1.0: the clamp clicks at 19.8 s with the blade 10 mm in, teeth forward, locks when the cap is let go at 21.2 s,
  and the trial ends at 24.0 s (480 steps) once both hands are clear. Two earlier runs of the same demo (before the
  saw's drawn-only details were added) gave the same click and lock times. Dream-1 scores 0.0 over the full 1,200
  steps: its right hand hovers by the blade's holder without picking the blade up, its left hand goes to the saw,
  and the clamp never clicks.
- The scripted demo's plan is 25.3 s of simulated time.
- The clamp model, the in-the-slot and teeth tests, the scoring, the sizes, reach and step-guard checks, and the demo
  against the real collision hulls run without the simulator: `python -m pytest -q test_jigsaw_blade.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
