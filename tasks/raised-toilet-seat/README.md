# Fit a front-lock raised toilet seat: rear wings under the bowl's lip, then the front knob turned until tight

A round-front toilet bowl stands in front of the arms, and on it a front-lock raised toilet seat that has been set on
but not fitted: it sits 8 mm too far forward and turned 3 degrees, its knob loose. The instruction asks for it to be
fitted the way the maker describes: pushed back and straight so the two wings under its back go under the bowl's rear
lip, then held there while the knob on its front is turned clockwise until the bracket under the front is tight
against the front of the bowl, and let go. One hand pushes the seat into place and holds it; the other turns the knob
through over three turns, letting go and taking a new grip each time its wrist has turned as far as it goes.

## The real step
After a hip replacement, patients are told not to sit on anything low enough to bend the hip past 90 degrees, and are
given a raised toilet seat to fit at home. Guy's and St Thomas' NHS,
[Precautions after hip surgery](https://guysandstthomas.nhs.uk/health-information/precautions-after-hip-surgery): "The
occupational therapist will give you equipment to raise the height of your toilet, bed and chair, if needed." The
seat is fitted by the patient's family or carer.

Invacare, [raised toilet seat 1391P/1391AP manual](https://www.manualsdir.com/manuals/129030/invacare-1391ap-1391p.html):
"The rear seat wings MUST be positioned under the rear lip of the toilet bowl", "turn the knob clockwise until the
adjustment bracket is securely fastened to the front of the toilet bowl", "The Raised Toilet Seat MUST be level,
straight, and the adjustment bracket MUST be inside the toilet bowl", "Always check to make sure that the Raised Toilet
Seat is properly and securely locked in place before using."

What goes wrong in real life: a seat that is set on but not locked looks fitted, then shifts or tips when someone leans
on it to sit down or stand up, at exactly the time a fall would do the most harm.

What we simplified: only the bowl's top 10 cm stands on the table, at its real size; the bracket's clamp is a model
(below), drawn fixed under the seat; the seat's size and weight are those of a seat of the same front-lock kind
(Invacare publishes none for the 1391); the push stands in for someone leaning on the seat.

**Category: Societal relevance.** Fitting a raised seat is not a regular core duty of an occupation with 500,000 or more
US workers (O*NET's tasks for personal care aides name toileting help, not fitting equipment), so this is framed as a
step a family does at home, not as a job.

### What this answers in robocurve#2034's review
That task (a quick-release seat clicked onto two hinge posts) was not accepted as an "egregious simplification": "A
rectangular ring on a flat block with simulated welded button catches does not represent the geometry and latching
mechanism of an actual quick-release toilet seat", and its catch and materials weren't specified. Here:
- **A real product's catch:** Invacare's 1391 holds by its rear wings under the bowl's lip and a bracket screwed against
  the inside front of the bowl (the manual's words above). The wings are part of the seat's collision shape and go under
  the lip by contact; only the bracket's screw is modelled, and it is described in full below.
- **The bowl's real shape and size:** a round-front bowl's rim, about 16-1/2 in from its seat-bolt holes to the front of
  its rim and about 14 in wide ([Bemis](https://www.homedepot.com/p/questions/BEMIS-Weston-Elongated-Soft-Close-Enameled-Wood-Closed-Front-Toilet-Seat-in-White-Never-Loosens-Brushed-Nickel-Metal-Hinge-1526NISL-000/305423137/1)),
  built as an oval rim with an opening and an undercut beneath the lip, not a block.
- **Materials:** a vitreous-china bowl, a molded-plastic seat, a plastic bracket and knob (setup table).
- **No weld:** nothing holds the seat but its contacts and the bracket's force; a push after the hands let go shows
  whether it holds.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=raised_toilet_seat.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the raised toilet seat to the toilet. Push it back and straight so its rear wings go under the rear lip of the bowl, then hold it there and turn the knob on its front clockwise until the bracket is tight against the front of the bowl. Then let go of it." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/raised-toilet-seat-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=raised_toilet_seat.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the raised toilet seat to the toilet. Push it back and straight so its rear wings go under the rear lip of the bowl, then hold it there and turn the knob on its front clockwise until the bracket is tight against the front of the bowl. Then let go of it." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/raised-toilet-seat-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/raised-toilet-seat-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the finished state has held for 1 s; otherwise it
scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| bowl | the top of a biscuit-coloured vitreous-china round-front toilet bowl (fixed): its rim 43 x 35.6 cm outside, its opening 29 x 24 cm, the rim 2.5 cm thick at its inner edge (the lip), with an 8 mm undercut beneath the lip; its front towards the arms | its middle at (43, 0); the rim's top 10 cm above the table |
| seat | a white molded-plastic front-lock raised toilet seat, 43 x 42 cm and 11.4 cm tall, 1.8 kg, with a hole 26 x 21 cm; set on the rim, not fitted | 8 mm forward of fitted, turned 3 degrees anticlockwise (seen from above) |
| rear wings | two plastic tabs under the seat's back, 6 mm thick, 2 cm wide and 3.35 cm deep, each turned to face the lip's edge where it faces it (35 degrees off straight back), with a 6 mm hook pointing out at its foot, its top 0.5 mm under the lip; fitted, each tab's back is 1 mm inside the lip's edge and its hook 5 mm under the lip | inside the opening, where the lip's edge is 6 cm either side of the middle |
| bracket | a grey plastic plate under the seat's front, inside the opening, screwed all the way back (opened) | 11 mm short of the bowl's front wall when the seat is fitted |
| knob | a grey fluted plastic knob (12-sided, 4 cm across) 2.2 cm long on the seat's front face, 5 cm up, turning about the seat's front-to-back axis; clockwise drives the bracket forwards 5 mm a turn | at the front of the seat, on its middle line |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
From the seat's pose and the clamp model.

| Score | When, at the end |
|---|---|
| 1.0 | fitted, tight, and held under the push with both grippers more than 2 cm from the seat and knob, held 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.5 | fitted and tight, but not yet pushed (a hand still on it) or moved by the push |
| 0.25 | fitted, not tight |
| 0 | anything else: the seat still set on as it started, or pushed off its place |

- **Fitted:** both wings' hooks at least 3 mm under the lip (their ends past the lip's edge, below it), the seat resting
  on the rim (its bottom within 2 mm of the rim's top), level within 2 degrees, straight within 3 degrees and centred
  within 5 mm.
- **Tight:** the bracket pressing the bowl's front wall with 50 N or more.
- **The push:** once both grippers are more than 2 cm from the seat and the knob and the seat is fitted, a push is put
  on the seat 10 cm behind its front edge, at half its height: 1 s forwards, then 1 s to the robot's left, then 1 s to
  its right, each rising smoothly to 40 N and back. It holds if the seat moves under 3 mm and turns under 1 degree all
  the while. A tight seat is held at its front by the bracket and at its back by the wings; a loose one slides until
  the bracket or a wing meets the bowl, and its hooks come out from under the lip.

## The clamp model
`SeatClamp` in `raised_toilet_seat.py` runs before every physics substep (240 Hz):
- **The knob** is a rigid body on a PhysX revolute joint in the seat (`fittingkit.Hinge`, lightly damped). The model
  counts its turns from its pose in the seat's.
- **The bracket** moves forwards 5 mm for each clockwise turn, from 3 mm inside the lip's edge. Once its face is past
  the inside of the bowl's front wall it presses it like a stiff spring (25,000 N/m, damped about critically, bearing at
  most 300 N), which pushes the seat back at the bracket. Across the wall it sticks like static friction: a stiff
  spring from where it first touched, slipping once it would pull more than 0.5 times the bracket's force.
- **The screw** is self-locking: once the bracket presses, turning the knob either way takes the screw's torque (the
  force times 5 mm over 2 pi, over an efficiency of 0.25: 0.16 N m at 50 N, rising no further past 100 N, where its
  thread slips: "DO NOT overtighten knob"). The bracket follows the knob only while a hand is on the seat or the knob;
  with no hand on them it stays where it is (in the sim the loaded knob otherwise crept back by itself).
- **The push** is put on the seat as a force once the hands are clear (above).

Everything else is contact: the seat on the rim, the wings against the lip's edge and under it, the fingers on the knob
and on the seat.

## The scripted demo
Open loop, both arms, on one shared timeline:
1. The left hand, fingers closed and pointing forwards, comes in front of the seat's front face 12 cm to the robot's
   left of its middle and pushes it back with its fingertips, aimed 20 mm past the face's fitted line (the arm gives
   about 550 N/m): the seat turns straight and goes back until both wings are against the lip's edge, their hooks
   under it. Then it eases off to 5 mm past and holds it there. (Pointing down instead, the hand's palm, 10 cm wide
   4 cm above its fingertips, would go into the seat's top.)
2. The right hand, pointing forwards with its fingers closing up and down, comes in front of the knob, opens, goes onto
   it and closes; it turns the knob clockwise 105 degrees, opens a little, turns back and closes again, twelve times
   (1,260 degrees; in the sim the knob turned 3.34 of the 3.5 turns, and the bracket was tight after 2.72).
3. Both hands move clear; the push follows.

The poses were checked offline against the YAM's gripper and wrist collision hulls (URDF forward kinematics,
`plan_check.py`).

## Part kit
`fittingkit.py` is copied from the accepted robocurve/benchmark-tasks task `tasks/compression-joint` (commit de817b9),
under the MIT licence its authors state in that task's README, with one addition noted in its docstring (an optional
`Slider` drive, not used here). `plan_check.py` is the offline check of the demo described above.

## Runtime and seeds
- Official runs (2026-11-18, the Robocurve Brev launchable, 1,800 steps, `object_jitter=0`): the scripted demo scores
  1.0: the seat is pushed home (its hooks under the lip) by 7.0 s, the bracket is tight (50 N) at 52.2 s after 2.72 turns, both hands are clear
  and the push starts at 64.8 s with the bracket at 120 N; the seat moves 0.3 mm and turns 0.23 degrees under it, and
  the trial ends at 68.9 s (1,377 steps). A run of the same demo before the bowl and seam were recoloured gave the same
  numbers. Dream-1 scores 0.0 over the full 1,800 steps: both arms move towards the seat in the first half minute
  without fitting it, then draw back towards their bases; the knob is never turned.
- The same demo with three knob strokes instead of twelve (the seat fitted but loose, its bracket short of the bowl)
  scores 0: the push slides the seat 8 mm forwards, until the bracket meets the bowl, and 10 mm to each side; its hooks
  come out from under the lip (moved 14.2 mm, turned 2.6 degrees).
- The scripted demo's plan is 70.5 s of simulated time; the run limit is 1,800 steps (90 s).
- The fit, the clamp model, the push, the scoring, the sizes, reach and step-guard checks, and the demo against the real
  collision hulls run without the simulator: `python -m pytest -q test_raised_toilet_seat.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
