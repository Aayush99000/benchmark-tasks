# Give a child manikin 5 ml of liquid medicine with an oral syringe

A child care-training manikin's head faces the arms across the table, and a filled 5 ml oral syringe lies in a small
plastic tray. The instruction asks for the medicine to be given the way a parent is taught to: the syringe's tip inside
the cheek, between the gums and the cheek, the plunger pushed in small squirts with a pause between them for the child
to swallow, until the syringe is empty, then the syringe taken out. One hand holds the syringe at the corner of the
mouth, angled into the cheek, while the other pushes the plunger in measured strokes against the drag of the liquid.

## The real step
This is how parents and carers give a child liquid medicine. Great Ormond Street Hospital,
[How to give your child liquid medicines](https://www.gosh.nhs.uk/medical-information/medicines-information/how-give-your-child-liquid-medicines):
"Put the tip of the oral syringe inside your child's mouth between the gums and the inside surface of their cheek",
"Gently push the plunger to squirt small amounts of medicine into the side of your child's mouth", "Allow your child to
swallow before you carry on pushing the plunger", "Do not squirt the entire dose into your child's mouth in one go –
they may choke". Done by: parents and carers, and nurses.

What goes wrong in real life: the whole dose squirted at once, or towards the back of the mouth, and the child chokes or
spits it out.

What we simplified: the manikin's head is rigid and fixed; no liquid is simulated: the dose given is the plunger's
travel, and pushing it is damped as a syrup through the tip would be; the syringe starts filled, in a tray that keeps it
from rolling.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=oral_syringe.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Give the child manikin the 5 ml of medicine in the oral syringe. Put the syringe's tip inside its cheek, between the gums and the cheek, and push the plunger in small squirts, pausing between them for it to swallow, until the syringe is empty. Then take the syringe out of its mouth." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/oral-syringe-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=oral_syringe.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Give the child manikin the 5 ml of medicine in the oral syringe. Put the syringe's tip inside its cheek, between the gums and the cheek, and push the plunger in small squirts, pausing between them for it to swallow, until the syringe is empty. Then take the syringe out of its mouth." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/oral-syringe-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/oral-syringe-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the finished state has held for 1 s; otherwise it
scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| manikin | a child care-training manikin's head (fixed), 15 cm wide and 18 cm tall, on a neck post, facing the arms; its open mouth an opening 40 mm wide, 32 mm tall and 45 mm deep, lips round it | the lips' plane at x 45; the mouth's middle at (45, 0), 14 cm above the table |
| syringe | a 5 ml oral syringe, filled: a clear barrel (12.06 mm bore, 14.5 mm outside, 58 mm long) with a 10 mm tip and a finger flange (25 x 8 mm); the plunger drawn back 43.8 mm (5 ml), its thumb rest 17 mm across. Pushing the plunger takes about 2.6 N at 1 ml/s | lying along x, its tip towards the manikin, the flange at (24, 17), 1 cm above the table |
| tray | a fixed plastic tray under the barrel, with two stops beside its front so it can't roll | under the syringe |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
From the plunger's travel (1 ml = 8.75 mm) and the syringe's pose.

| Score | When, at the end |
|---|---|
| 1.0 | the whole 5 ml given (at least 4.8 ml), every bit of it with the tip in the cheek, no squirt over 2 ml, and then the syringe out of the mouth, held 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.5 | the whole dose given with the tip in the cheek, but a squirt over 2 ml, or the syringe not taken out |
| 0.25 | at least 2 ml given, all with the tip in the cheek |
| 0 | anything else: any medicine (over 0.1 ml) pushed out with the tip not in the cheek, or under 2 ml given |

- The tip is in the cheek when it is 8 to 25 mm past the lips, 8 to 18 mm to one side of the mouth's middle and within
  the mouth's height, and the syringe points at least 10 degrees towards that cheek, off straight in.
- The medicine given is the furthest the plunger has gone. A squirt is what is given between pauses; a pause is a full
  second in which the plunger goes less than 0.5 mm further (0.06 ml).
- Out of the mouth: the tip at least 5 mm back from the lips.

## The syringe model
`DoseLog` in `oral_syringe.py` runs before every physics substep (240 Hz). It reads the plunger's travel in the barrel,
counts what is given from the furthest point it has reached, adds what is given with the tip not in the cheek, and
splits it into squirts at pauses. The plunger is a rigid body on a PhysX prismatic joint in the barrel
(`fittingkit.Slider` with a damping drive, 300 N s/m: about 2.6 N at 1 ml/s); only its thumb rest collides, and it stops
2 mm short of the flange with the syringe empty.

## The scripted demo
Open loop, both arms, on one shared timeline:
1. The left hand comes down from high over the syringe and holds its barrel 16 mm in front of the flange.
2. It lifts the syringe and brings it to the face, turned 25 degrees towards the cheek on the robot's left and leaning
   45 degrees in from its own side, then puts the tip 15 mm past the lips and 12 mm to the side.
3. The right hand, fingers closed and pointing along the syringe (turned so they close up and down), comes to the thumb
   rest and pushes it in three squirts of about 1.7 ml, each over 1.8 s, with 3 s between them.
4. It backs off, and the left hand takes the syringe out of the mouth.

The poses were checked offline against the YAM's gripper and wrist collision hulls (URDF forward kinematics,
`plan_check.py`): holding the barrel's middle and leaning 30 degrees, the left hand went 8 mm into the face; holding it
further back and leaning 45 degrees, it clears the face. Both arms reach every pose within 0.1 mm and no step exceeds
76% of the per-step joint limit.

## Part kit
`fittingkit.py` is copied from the accepted robocurve/benchmark-tasks task `tasks/compression-joint` (commit de817b9),
under the MIT licence its authors state in that task's README, with one addition noted in its docstring: `Slider`
takes an optional drive (`stiffness`, `damping`, `target`), used here to damp the plunger. `plan_check.py` is the offline
check of the demo described above.

## Runtime and seeds
- Official runs (2026-11-17, the Robocurve Brev launchable, 1,200 steps, `object_jitter=0`): the scripted demo scores
  1.0: squirts of 1.66, 1.62 and 1.73 ml (ending at 17.1, 21.8 and 26.2 s), 5.01 ml in all, none with the tip out
  of the cheek; the syringe taken out and the trial ends at 31.4 s (628 steps). A run of the same demo before the
  official one logged the same squirts. Dream-1 scores 0.0 over the full 1,200 steps: both hands move to the
  manikin's face and hover there, the syringe is never picked up from its tray, and no medicine is given.
- The scripted demo's plan is 32.5 s of simulated time.
- The dose log, the tip-in-the-cheek test, the scoring, the sizes, reach and step-guard checks, and the demo against
  the real collision hulls run without the simulator: `python -m pytest -q test_oral_syringe.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
