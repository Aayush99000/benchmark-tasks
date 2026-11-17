# Give a child manikin one puff of an inhaler through a spacer

A child care-training manikin's head faces the arms across the table. A pressurised metered-dose inhaler (cap off)
lies on its side, and a spacer the size of an AeroChamber Plus lies beside it. The instruction asks for one puff of the
inhaler through the spacer: shake the inhaler, fit it into the back of the spacer, put the spacer's mouthpiece in the
manikin's mouth, press the inhaler once, and keep the spacer in its mouth for its five breaths. One hand keeps the
spacer in the mouth while the other holds the inhaler end to end, as a thumb and index finger do, and squeezes the
canister in with the real firing force.

## The real step
This is how parents and carers give a child an asthma reliever. Hull University Teaching Hospitals NHS Trust,
[Using your Inhaler with a Spacer](https://www.hey.nhs.uk/patient-leaflet/using-your-inhaler-with-a-spacer/): "Shake the
inhaler well", "Attach the inhaler onto the end of the spacer", "Ensure there is a good seal around the mask or
mouthpiece", "Press the inhaler once and allow your child to take 5 slow breaths", "Only put one puff of medicine into
the spacer at a time". Done by: parents and carers of children with asthma (Asthma + Lung UK: "An estimated 1 in 8
children have asthma", from the Health Survey for England 2018), and nurses.

What goes wrong in real life: a puff fired with the mouthpiece out of the mouth, or two puffs at once, and the dose
coats the spacer's walls instead of reaching the lungs.

What we simplified: the manikin's head is rigid and its lips are a rigid opening, so the seal is judged from where the
spacer's tip is, not from contact; no spray is simulated (the canister logs a puff past its firing point); the spacer's
soft back socket is a rigid square socket; the inhaler starts with its cap already off.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=inhaler_spacer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Give the child manikin one puff of the inhaler through the spacer. Shake the inhaler, fit it into the back of the spacer, put the spacer's mouthpiece in the manikin's mouth, press the inhaler once, and keep the spacer in its mouth for its five breaths." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/inhaler-spacer-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=inhaler_spacer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Give the child manikin one puff of the inhaler through the spacer. Shake the inhaler, fit it into the back of the spacer, put the spacer's mouthpiece in the manikin's mouth, press the inhaler once, and keep the spacer in its mouth for its five breaths." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/inhaler-spacer-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/inhaler-spacer-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the finished state has held for 1 s; otherwise it
scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| manikin | a child care-training manikin's head (fixed), 15 cm wide and 18 cm tall, on a neck post, facing the arms; its open mouth an opening 40 mm wide and 32 mm tall, 45 mm deep, lips round it | the lips' plane at x 45; the mouth's middle at (45, 0), 14 cm above the table |
| inhaler | a pressurised metered-dose inhaler, cap off: a blue plastic actuator (an upright boot 73 mm tall, 28 x 30 mm, its mouthpiece 24 x 17 mm and 24 mm long at the bottom) and an aluminium canister (21 mm across, 75 mm long) standing 15 mm proud of the boot: 88 mm end to end. Lying on its side, the canister towards the robot's left and the mouthpiece towards the manikin | its base at (26, -16), the canister's end at (26, -7.2) |
| canister | sprung in the actuator: it fires (a puff) when pushed 2.5 mm in, which takes 25 N; 3 mm of travel; ready again once back above 1 mm | |
| spacer | the size of an AeroChamber Plus: a clear barrel 145 mm long and 46 mm across, a rubber back with a socket 32 mm square and 20 mm deep, a mouthpiece 24 mm across. Lying along x, its mouthpiece towards the manikin | its back at (28, 16), its tip at (42.5, 16) |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
From the inhaler's fire log and the parts' poses.

| Score | When, at the end |
|---|---|
| 1.0 | at least 4 shakes, then exactly one puff, fired with the inhaler fitted and the spacer in the mouth, and the spacer kept in the mouth for 5 s after it, held 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.5 | exactly one puff with the inhaler fitted and the spacer in the mouth, but not shaken first or not kept in for 5 s |
| 0.25 | the inhaler fitted and the spacer in the mouth, no puff |
| 0 | anything else: two or more puffs, or a puff with the inhaler not fitted or the spacer not in the mouth |

- A shake is a down-and-up stroke of the inhaler of at least 3 cm.
- A puff is the canister pushed past its firing point (2.5 mm).
- The inhaler is fitted when its mouthpiece's tip is at least 10 mm inside the spacer's socket and within 4 mm of the
  spacer's axis.
- The spacer is in the mouth when its tip is at least 8 mm past the lips, within 6 mm of the mouth's middle, and the
  spacer within 15 degrees of square to the face.

## The inhaler model
`InhalerLog` in `inhaler_spacer.py` runs before every physics substep (240 Hz). It counts shakes from the inhaler's
height, logs each puff (once, until the canister is back above 1 mm) with whether the inhaler was fitted and the spacer
in the mouth at that moment, and times how long the spacer then stays in the mouth. The canister is a rigid body on a
sprung PhysX prismatic joint in the actuator (`fittingkit.Slider` with a drive: 10,000 N/m, so 25 N at the firing
point). Measured pMDI actuation forces are 22 to 31 N (three beclomethasone products, Drug Development and Industrial
Pharmacy, 2008). The YAM's fingers close with up to 50 N each, so a hand holding the inhaler end to end can fire it.

## The scripted demo
Open loop, both arms, on one shared timeline:
1. The right hand comes down from high over the inhaler and holds it end to end (one finger on the canister's end,
   the other on the actuator's base) at about 10 N, short of firing; the left hand holds the spacer's barrel from
   above.
2. Both lift. The right hand turns the inhaler upright (a quarter turn about x); the left brings the spacer up to the
   face, leaning 30 degrees in from its own side (pointing straight down there, the left arm can't reach the pose),
   which rolls the spacer 30 degrees about its axis.
3. The right hand shakes the inhaler 4 times, 5 cm down and up, then brings its mouthpiece behind the socket and
   pushes it 17 mm in.
4. Both hands move the spacer 4 cm forward, its tip 12 mm into the mouth.
5. The right hand closes harder: the canister settles about 2.8 mm in at about 28 N, past its firing point. It eases
   back to the hold, and both hands keep the spacer in the mouth for 6 s.

The demo was checked offline against the YAM's gripper and wrist collision hulls (URDF forward kinematics,
`plan_check.py`): no part of either hand touches the manikin, the spacer or the inhaler except where it holds them,
the hands stay at least 37 mm apart, both arms reach every pose within 0.1 mm, and no step exceeds 82% of the per-step
joint limit.

## Part kit
`fittingkit.py` is copied from the accepted robocurve/benchmark-tasks task `tasks/compression-joint` (commit de817b9),
under the MIT licence its authors state in that task's README, with one addition noted in its docstring: `Slider`
takes an optional spring drive (`stiffness`, `damping`, `target`), used here for the canister. `plan_check.py` is the
offline check of the demo described above.

## Runtime and seeds
- Official runs (2026-11-17, the Robocurve Brev launchable, 1,200 steps, `object_jitter=0`): the scripted demo scores
  1.0: 4 shakes, then one puff at 18.6 s with the mouthpiece 12.9 mm into the socket and the spacer's tip 12.8 mm past
  the lips, 3.7 mm off the mouth's middle; kept in the mouth 5 s; the trial ends at 24.7 s (493 steps). An earlier run of the same demo (aimed 3 mm
  lower) also scored 1.0, with the same puff time. Dream-1 scores 0.0 over the full 1,200 steps: its right hand hovers
  over the inhaler and nudges it without picking it up, its left hand hovers by the spacer and the head, and no puff
  is fired.
- The scripted demo's plan is 25.7 s of simulated time.
- The fire log, the fitted and in-the-mouth tests, the scoring, the sizes, reach and step-guard checks, and the demo
  against the real collision hulls run without the simulator: `python -m pytest -q test_inhaler_spacer.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
