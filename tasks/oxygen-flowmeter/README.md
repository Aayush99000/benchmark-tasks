# Set up oxygen through a nasal cannula at 2 L/min on a wall flowmeter

A hospital wall oxygen flowmeter (0-15 L/min, a float ball in a clear flow tube) is plugged into the oxygen outlet on
a headwall panel, turned off. The end of a nasal cannula's oxygen tubing lies on the cannula's packet beside it. The
instruction asks for the patient's oxygen to be set up the way nurses are taught: connect the cannula's tubing to the
flowmeter, then set the prescribed flow. One hand pushes the tubing's end fully onto the flowmeter's barbed outlet
nipple, past its last barb; the other turns the flow-control knob until the middle of the float ball sits on the
2 L/min line, then both let go.

## The real step
Setting up oxygen at a prescribed flow is routine nursing care. Open RN, *Nursing Skills* (Ernstmeyer and Christman,
eds., Chippewa Valley Technical College; this copy CC BY-SA 4.0),
[4.6 Checklist for Oxygen Therapy](https://med.libretexts.org/Courses/University_of_South_Carolina_Upstate/Nursing_Skills_(OpenRN)/04%3A_Oxygen_Therapy/4.06%3A_Checklist_for_Oxygen_Therapy):
"Connect flow meter to oxygen supply source.", "Connect nasal cannula tubing to flow meter.", "Set oxygen flow at
prescribed rate." The flowmeter's maker on setting it: Ohio Medical, Medical Gas Flowmeters Instructions for Use
(7700-0631-000 Rev. 12, 2023): "To INCREASE Flow: Turn the know counter clockwise" [sic], "To set the flow, align the
center of the Float Ball to the indicator line on the Flow Tube."

Done by: registered nurses. The BLS counts 3,465,400 registered nurse jobs in 2025 ("about 3.5 million"), whose duties
include "Administer patients' medicines and treatments" and "Operate and monitor medical equipment"
([Occupational Outlook Handbook](https://www.bls.gov/ooh/healthcare/registered-nurses.htm)). Licensed practical nurses
and respiratory therapists do it too.

What goes wrong in real life: tubing pushed only part way onto the nipple leaks or pops off, and the patient gets
less oxygen than the flowmeter shows; a flow read off the ball's top or bottom, or set a line out, is the wrong dose
(too little for a hypoxic patient, too much for one who retains carbon dioxide).

What we simplified:
- The flowmeter is fixed on a headwall panel at table height (on a ward it is on the wall above the bed), and its
  outlet nipple points forward, towards the arms (on many flowmeters it points down, under the body).
- No oxygen is simulated: the flow is set by the knob's angle, proportional up to 15 L/min at 270 degrees, then flush
  (our estimate of the needle valve's travel), and the ball's height by the flow.
- The scale's spacing is our estimate, laid out like Ohio Medical's 6700 series (spec sheet 550038: half-L/min lines
  from 1 to 5, whole ones from 5 to 15, "expanded 1-5 L/min range"): 10 mm per L/min up to 5, 2.6 mm per L/min above.
- The tubing's end is the last 15 cm of the tubing, rigid, 8 mm across and 40 g (real oxygen tubing is about 7 mm
  across and a few grams) so the YAM's fingers hold it firmly; the rest of the tubing and the prongs are not modelled.
- The nipple's barbs are a model (below): they seat the tubing once it is past the last one, but don't resist the push.
  We tried a resisting force (6, 3, then 1.5 N): each time it came on, the tubing pivoted in the YAM's two-finger pinch
  (the simulated pinch barely resists turning about the line between the fingertips) and jammed on the nipple.
- The knob is on the flowmeter's front, at the top.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=oxygen_flowmeter.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set up the patient's oxygen at 2 L/min. Push the nasal cannula's tubing fully onto the flowmeter's outlet nipple, then turn the flowmeter's knob until the middle of the float ball is on the 2 L/min line, and let go of both." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/oxygen-flowmeter-dream

# The scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=oxygen_flowmeter.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set up the patient's oxygen at 2 L/min. Push the nasal cannula's tubing fully onto the flowmeter's outlet nipple, then turn the flowmeter's knob until the middle of the float ball is on the 2 L/min line, and let go of both." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/oxygen-flowmeter-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/oxygen-flowmeter-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the finished state has held for 1 s; otherwise it
scores the partial credit below at the last step. To see the models' events, set `OXY_LOG=<file>` (the moment the
tubing is seated, and how far on); `OXY_TRACE=<file>` adds the tubing's mouth, the knob's angle, the flow and both
TCPs every 0.1 s.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| headwall panel | a 24 cm wide, 30 cm tall panel standing on the table, with a green oxygen wall outlet, fixed | its face at x 38.6 |
| flowmeter | a 0-15 L/min oxygen flowmeter, 152 mm tall, 56 mm wide, 58 mm deep with its flow tube's clear shroud (Ohio Medical 6700 size), its inlet plugged into the wall outlet, fixed; turned off (the knob fully clockwise, the ball at the bottom) | its front face at x 31, centred on y 0, from 7 to 22.2 cm above the table |
| flow tube | a clear tube in a clear shroud on the front, with a float ball 8 mm across; lines at every 0.5 L/min from 1 to 5 and every 1 L/min from 5 to 15, numbered at 1-5, 10 and 15; the ball's middle rests 10.4 cm above the table at 0 and is on the 2 L/min line 2 cm higher | the shroud from x 29.8 to 31, 9.6 to 18.6 cm up |
| knob | the green flow-control knob, 27 mm across and 18 mm deep, fluted, with a white pointer line; on a hinge in the flowmeter, turned 300 degrees from off to its stop; anticlockwise, seen from the front, opens it | its axis 21.2 cm up, its front at x 29 |
| outlet nipple | a green barbed tubing nipple, 20 mm long (three barbs), on a DISS outlet nut, pointing towards the arms | its axis 8.4 cm up; its tip at x 27.5 |
| tubing | the end of a nasal cannula's oxygen tubing: 15 cm, clear green, 8 mm across, its open end towards the flowmeter, lying along x on the packet with its last 55 mm over the packet's edge | its open end at (20, -10), 2.9 cm up |
| packet | the cannula's opened packet, 10 x 7 x 2.5 cm, fixed | centred at (9.5, -10) |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
From the two models and the parts' poses, at the end.

| Score | When, at the end |
|---|---|
| 1.0 | the tubing seated on the nipple, the flow within 0.25 L/min of 2 (the ball's middle within 2.5 mm of the 2 L/min line), and both grippers more than 2 cm from the tubing and the knob, held 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.5 | the tubing seated, the flow on (0.1 to 4 L/min) but off the 2 L/min line |
| 0.25 | the tubing seated with the flow off, or the flow on the 2 L/min line with the tubing not seated (only part way on, or not on) |
| 0 | anything else, and any flow over 4 L/min |

- The tubing is on the nipple when its open end is past the nipple's tip, within 2 mm of its axis, and the tubing is
  within 10 degrees of the nipple's line. Seated: its end 12 mm or more on, past the last barb (the nut stops it at
  20 mm).
- The flow follows the knob's angle from off: 2 L/min at 36 degrees anticlockwise, 0.25 L/min every 4.5 degrees.
- 0.25 L/min is half the scale's 0.5 L/min spacing there, and the flowmeter maker's stated accuracy at 1 L/min
  (spec sheet 550038: "± 1/4 L/min at 1 L/min").

## The models
`Oxygen` in `oxygen_flowmeter.py` runs before every physics substep (240 Hz):
- The nipple: the nipple's core is a 4 mm rod, tapering in three steps over its last 6 mm to 2.2 mm at the tip, and
  the tubing's mouth a 6.8 mm bore 24 mm deep, chamfered out to 8 mm over its last 3 mm (a soft end gives as it
  meets the nipple), so the tubing really goes on over it. Once its mouth is 12 mm on, past the last barb, it is seated and held
  where it is, on the nipple's axis (its pose is written back every substep). Tubing short of that stays where physics
  leaves it.
- The flowmeter: the knob is a rigid body on a PhysX revolute joint in the flowmeter (`fittingkit.Hinge`, damped
  0.03 N m s/rad, from off to 300 degrees). The model reads its angle (unwrapped), sets the flow from it, and writes
  the float ball's pose every substep at the flow's line. The ball sits in a hollow shroud and touches nothing.

## The scripted demo
Open loop, both arms, on one shared timeline (fixed planned poses; nothing is read back from the simulator):
1. The right hand comes down over the tubing, pointing down, its fingers closing across it, and pinches it 38 mm
   behind its open end, where it hangs over the packet's edge.
2. It lifts the tubing, carries it level to the nipple's line, 3 cm short of its tip and 1.4 mm high (held, the
   tubing rides that much low), puts its end on the tip and pushes it on to the nut, aiming 4 mm past so the arm
   presses it home; it lets go, backs 4 cm off the nipple and moves away.
3. The left hand, level and pointing at the knob, rolled 90 degrees clockwise (its pads square on two of the knob's
   12 flats, and the turn within its wrist's reach), closes snugly on the knob (each finger 2.5 mm into it, about
   12 N), pauses, and turns it 37.5 degrees anticlockwise over 4 s (2.08 L/min); it opens its fingers part way,
   backs off, and moves clear.

The poses were checked offline against the YAM's gripper and wrist collision hulls (URDF forward kinematics,
`plan_check.py`): neither hand touches the flowmeter, the packet, or the tubing or knob other than where it holds
them, and the hands stay at least 33.4 mm apart. Both arms reach every pose within 0.5 mm and 0.5 degrees, and no
step exceeds 75% of the per-step joint limit.

How the demo got there (12 official-simulator runs, `oxy1` to `oxy12`, 2026-11-18), each change from what a run's
trace showed:
- The tubing jammed on the nipple's core 5 to 9 mm on in runs 1 to 6. Held, it rode 1.9 mm low (now aimed 1.4 mm
  high); the bore had 1 mm of clearance (now 1.4 mm); and a resisting barb force (6, then 3, then 1.5 N) pivoted it
  in the pinch each time it came on, dropping its mouth onto the core. With no barb force (run 7 on) it seats every
  time, 12 mm on.
- The knob: its corners dragged on the flow tube's top cap (run 1; the knob now sits higher and the cap lower); the
  fingers, opening, caught the cap and spun it (runs 8, 9; now gripped at a 90 degree roll and opened only part way
  before backing off); closing square to its flats instead of 10 degrees off stopped it turning as they closed
  (run 10); and closing snug instead of with each finger's full 50 N stopped the jolt and the arm's spring-back that
  had moved it 1 to 27 degrees as the fingers let go (runs 2 to 11). A round collider pushed the fingers off it
  (run 8) and heavier damping threw the hand off it (run 7); it is a 12-sided collider, damped 0.03 N m s/rad.

## Part kit
`fittingkit.py` is copied from the accepted robocurve/benchmark-tasks task `tasks/compression-joint` (commit de817b9),
under the MIT licence its authors state in that task's README, with one addition noted in its docstring (an optional
spring drive on `Slider`, not used here). This task uses its `Hinge` for the knob. `plan_check.py` is the offline check
of the demo described above.

## Runtime and seeds
- Official runs (2026-11-18, the Robocurve Brev launchable, 1,200 steps, `object_jitter=0`): the scripted demo scores
  1.0 (run `oxy12`): the tubing is seated at 12.7 s, 12.0 mm on, 0.46 mm off the nipple's axis and 0.2 degrees off
  its line; the knob is turned to 37.4 degrees, 2.08 L/min, and doesn't move as the fingers let go; the trial ends at
  32.9 s (658 steps) once both hands are clear. Dream-1 scores 0.0 over the full 1,200 steps (run `oxyD`): its right
  hand reaches towards the packet, then both arms draw back towards their bases; the tubing never leaves the packet
  and the knob is never turned.
- The scripted demo's plan is 35.3 s of simulated time.
- The two models, the scoring, the sizes, reach and step-guard checks, and the demo against the real collision hulls
  run without the simulator: `python -m pytest -q test_oxygen_flowmeter.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
