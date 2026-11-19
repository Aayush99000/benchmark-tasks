# Set up oxygen through a nasal cannula at 2 L/min on a wall flowmeter

A hospital wall oxygen flowmeter (0-15 L/min, a float ball in a clear flow tube) is plugged into the oxygen outlet on
a headwall panel, turned off. The end of a nasal cannula's oxygen tubing lies on the cannula's packet beside it. The
instruction asks for the patient's oxygen to be set up the way nurses are taught: connect the cannula's tubing to the
flowmeter, then set the prescribed flow. One hand pushes the tubing's end fully onto the flowmeter's barbed outlet
nipple, past its last barb, against the barbs' push-back (about 4 N, then 9, then 13 N as each barb stretches the
tubing); the other turns the flow-control knob until the middle of the float ball sits on the 2 L/min line, then both
let go.

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
- The tubing's end is the last 15 cm of the tubing, rigid, 8 mm across with a 5 mm bore, and 40 g (real tubing is a
  few grams) so the YAM's fingers hold it firmly; the rest of the tubing and the prongs are not modelled.
- Rigid tubing can't stretch over a barb, so the nipple collides as its 3.6 mm core and the stretch is a model (below):
  the barbs centre the tubing and push back on it as much as our estimate of what real tubing takes (next section);
  past the last barb it is seated and held.
- The knob is on the flowmeter's front, at the top.

## The barbs' push-back
How hard real tubing pushes back going onto the nipple, estimated from the parts' sizes and the tubing's material
(`barbfit.py`; `python barbfit.py` prints it). We have not measured it on real parts; the real-trial check below does.
- **Sizes.** The nipple is a "6 mm (1/4 inch) barbed fitting" and the tubing has an "interior diameter 5mm" (WHO,
  *COVID-19 Technical specifications for procurement of oxygen therapy and monitoring devices*,
  WHO/2020-nCoV/MedDev/TS/O2T.V1, 17 June 2020: the oxygen concentrator's outlet, and "Tubing (for medical gases)"). The barbs' shape is our estimate: three, each a cone from 3.6 mm across to its 6 mm crest
  over 4 mm, one every 5 mm. The tubing is 8 mm across.
- **Material.** Soft PVC, 70 Shore A [estimate: medical PVC tubing is about 65-80 A], so Young's modulus 5.5 MPa
  (Gent's conversion); friction 0.3 on the nipple's plastic [estimate].
- **Method.** The wall is a thick elastic cylinder (Lame), stretched over the last 1.67 mm of each cone, where the
  cone is wider than the 5 mm bore: 0.5 mm of radius at the crest, 0.45 MPa there. Sliding up a cone of that slope
  takes the radial load times (slope + friction) / (1 - friction x slope): **4.4 N per barb**. Behind the crest the
  wall snaps down the barb's steep back, and every barb already under the tubing goes on costing that as the tubing
  slides on, so the push-back is about 4.4 N over the first barb, 8.8 N over the second and **13.2 N over the last**.
- **Range.** From 65 A and friction 0.2 to 80 A and friction 0.4: 8.5 to 27 N over the last barb.
- **A check.** EN 13544-2 (respiratory therapy tubing and connectors) tests tubing on its nipple by pulling it with
  30 N for 15 minutes (Annex A); tubing that goes on with 13 N and needs more than that to pull off is consistent with
  a barb's shallow front and steep back. In the model, tubing pulled back once past the first crest holds up to 30 N.

## Real trials
The parts, from the hospital supply chain (each is a standard part; the numbers are a known-good choice, not the only
one):

| Part | What | Notes |
|---|---|---|
| flowmeter | Ohio Medical 6700-series oxygen flowmeter, 1-15 L/min, DISS nut-and-gland inlet, tubing-nipple outlet: 6703-1260-921 | built from the part-number table in Ohio Medical's spec sheet 550038 (adapter 03 "DISS Nut & Gland", 1260 "1-15 L/min O2 with O2 DISS fitting", outlet 21 "Tubing Nipple Nut & Gland"); calibrated at 50 psig; 152 x 56 x 58 mm |
| gas | a size E cylinder of medical oxygen USP (CGA 870 pin-index valve), from a medical gas supplier | about 680 L full [estimate: the usual figure for an E cylinder]: over 300 one-minute trials at 2 L/min |
| regulator | a medical oxygen regulator, CGA 870 inlet, preset 50 psig, DISS 1240 outlet (for example Airgas Y11HC870P50-AG) | 50 psi is the flowmeter's supply pressure (spec sheet 550038; WHO's regulator spec: "Outlet pressure 345 kPa (3.5 bar, 50 psi)") |
| hose | a green oxygen DISS 1240 hose, about 1.5 m, hex nut at the regulator end, DISS body at the other | the body goes through a hole in the panel and is clamped there: the panel's oxygen outlet |
| panel | 24 x 30 cm board on a foot, as in the setup | the flowmeter's inlet nut screws onto the hose's body, its back against the panel so it can't swivel |
| tubing | a nasal cannula with 2.1 m (7 ft) of oxygen tubing, its bore 4.8-5 mm | its prongs led off the table, away from the arms |
| force gauge | a push-pull force gauge, 50 N or more, with a flat head | for the push-on check |

Setting up:
1. Stand the cylinder in a cylinder stand or chain it to a fixed support, beside the table and out of both arms'
   reach. Fit the regulator, then the hose, then screw the flowmeter onto the hose's body at the panel.
2. With the flowmeter off, open the cylinder valve slowly; the regulator's gauge shows the cylinder's pressure. Check
   the joints for leaks with leak-detection solution.
3. Check the flow: open the knob to 2 L/min and see the ball's middle on the 2 line, and the flow at the cannula's
   prongs (bubbles through water, or an oxygen flow analyser if there is one). Turn it off.
4. The push-on check, before the first trial and every 10 trials: push a cut-off 15 cm of the same tubing onto the
   nipple with the force gauge's flat head on its end, slowly, and read the peak. The model expects about 13 N (8.5 to
   27 N); if it is far outside that, note it with the results. Cut 1 cm off the trial tubing's end every 10 trials,
   because a stretched end loosens.

Safety, for oxygen:
- Nothing oily or greasy on the grippers, the nipple, the regulator or anyone's hands (oil and oxygen can ignite):
  clean the gripper pads with water or isopropyl alcohol, not a lubricated cleaner.
- No open flame, sparks or smoking near it; keep the prongs' end away from the robot's motors and electronics, in open
  room air. A trial lets out at most about 2 L of oxygen, which a ventilated room takes without any build-up.
- Turn the flowmeter off between trials (the reset). At the end of a session close the cylinder valve, open the
  flowmeter to bleed the regulator, then turn it off.

Scoring a real trial: an overhead camera and a camera level with the flow tube record it.
- Seated: the tubing's end is within 5 mm of the nipple's hex base. The barbs show through the clear tubing, and a
  pen ring on the nipple's nut marks the 5 mm.
- The flow: the ball's middle against the 2 L/min line, from the level camera's last frame.
- Hands clear: from the overhead camera.

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
| outlet nipple | a green barbed tubing nipple, 20 mm long, on a DISS outlet nut, pointing towards the arms: three barbs, each a cone from 3.6 to 6 mm across over 4 mm, one every 5 mm from its tip, then a 4.6 mm shank to its hex base | its axis 8.4 cm up; its tip at x 27.5 |
| tubing | the end of a nasal cannula's oxygen tubing: 15 cm, clear green, 8 mm across with a 5 mm bore (its end flared to 6.2 mm), its open end towards the flowmeter, lying along x on the packet with its last 55 mm over the packet's edge | its open end at (20, -10), 2.9 cm up |
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

- The tubing goes on the nipple when its open end is past the nipple's tip, within 1.5 mm of its axis, and the tubing
  is within 10 degrees of the nipple's line; it stays on until pulled 3 mm or 15 degrees off its line or back off the
  tip. Seated: its end 15 mm or more on, past the last barb's crest (the hex base stops it at 20 mm).
- The flow follows the knob's angle from off: 2 L/min at 36 degrees anticlockwise, 0.25 L/min every 4.5 degrees.
- 0.25 L/min is half the scale's 0.5 L/min spacing there, and the flowmeter maker's stated accuracy at 1 L/min
  (spec sheet 550038: "± 1/4 L/min at 1 L/min").

## The models
`Oxygen` in `oxygen_flowmeter.py` runs before every physics substep (240 Hz):
- The nipple: it collides as its 3.6 mm core, inside the tubing's 5 mm bore (24 mm deep, flared to 6.2 mm over its
  last 3 mm), so the tubing really goes on over it. While the tubing is on the nipple:
  - the stretched wall centres it: springs (1,000 N/m, 2 N s/m, at most 5 N each) at its mouth and where the
    nipple's tip is inside it pull both onto the nipple's axis;
  - the barbs hold it like friction: whatever pushes the tubing along the nipple (fingers, the other hand, the
    table), the model pushes back as much, up to the push-back above for how far on it is, so a weaker push doesn't
    move it and a stronger one moves it on against that. The push is measured from the tubing's own motion: its mass
    times its change of speed along the nipple, less the model's own force the substep before. Pulled back, it holds
    up to 30 N once past the first barb.

  Once its mouth is 15 mm on, past the last barb, it is seated and held where it is, on the nipple's axis (its pose
  is written back every substep).
- The flowmeter: the knob is a rigid body on a PhysX revolute joint in the flowmeter (`fittingkit.Hinge`, damped
  0.03 N m s/rad, from off to 300 degrees). The model reads its angle (unwrapped), sets the flow from it, and writes
  the float ball's pose every substep at the flow's line. The ball sits in a hollow shroud and touches nothing.

## The scripted demo
Open loop, both arms, on one shared timeline (fixed planned poses; nothing is read back from the simulator):
1. The right hand comes down over the tubing, pointing down, its fingers closing across it, and pinches it 38 mm
   behind its open end, where it hangs over the packet's edge.
2. It lifts the tubing, carries it level to the nipple's line, 3 cm short of its tip and 1.4 mm high (held, the
   tubing rides that much low), puts its end on the tip and pushes it on to the hex base in seven steps, harder at
   each barb: each step aims for the barbs' push-back there plus 1.5 N. The arm's joints are springs (80 N m/rad at
   the shoulder, 10 at the wrist, `yam_arena.constants`), so at this pose the pinch gives about 6 mm back, 1 mm
   sideways and 0.6 mm down, and the hand pitches 1.3 degrees, per newton of push-back; each step is aimed that much
   further on (the joints' compliance through the arm's Jacobian, `aim_for_push`), up to 9 cm past where the tubing
   is for the last barb's 14.7 N. It eases off to where the tubing is, lets go, backs 4 cm off the nipple and moves
   away.
3. The left hand, level and pointing at the knob, rolled 90 degrees clockwise (its pads square on two of the knob's
   12 flats, and the turn within its wrist's reach), closes snugly on the knob (each finger 2.5 mm into it, about
   12 N), pauses, and turns it 37.5 degrees anticlockwise over 4 s (2.08 L/min); it opens its fingers part way,
   backs off, and moves clear.

The poses were checked offline against the YAM's gripper and wrist collision hulls (URDF forward kinematics,
`plan_check.py`, the right hand where it is expected to be under the push-back while it pushes): neither hand touches the flowmeter, the packet, or the tubing or knob other than where it holds
them, and the hands stay at least 33.4 mm apart. Both arms reach every pose within 0.5 mm and 0.5 degrees, and no
step exceeds 75% of the per-step joint limit.

How the demo got there (12 official-simulator runs, `oxy1` to `oxy12`, 2026-11-18, then 2 for the revision, `oxr1`
and `oxr2`, 2026-11-19), each change from what a run's trace showed:
- The revision (barbs that push back): aimed for each barb's push-back plus 1.5 N, the hand pushed the tubing over
  the first two barbs and then 13.0 to 13.2 N against the last, and it stopped 0.3 mm short of the crest, not seated
  (run `oxr1`, 0.25: the flow set, the tubing not seated). The arm's joint friction and its give, larger than the
  linear estimate at that deflection, took the rest. Aimed 3.5 N over, it went over the last barb at 13.2 N and seated
  (run `oxr2`). The tubing stayed on the nipple throughout, at most 1.1 mm off its axis (past 0.7 mm the bore rests
  on the core) and tilted up to 7 degrees in the pinch as the hand pitched under the push.
- Before the revision, with a rigid oversized fit: the tubing jammed on the nipple's core 5 to 9 mm on in runs 1 to 6. Held, it rode 1.9 mm low (now aimed 1.4 mm
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
- Official runs (2026-11-19, the Robocurve Brev launchable, 1,200 steps, `object_jitter=0`): the scripted demo scores
  1.0 (run `oxr2`): the tubing goes on the nipple at 11.6 s, 0.35 mm off its axis; the hand pushes it over the three
  barbs, up to 13.2 N; it is seated at 18.0 s, 15.0 mm on; the knob is turned to 37.5 degrees, 2.08 L/min; the trial
  ends at 37.3 s (746 steps) once both hands are clear. Pushed with 13.0 N at most, the same demo leaves the tubing
  short of the last barb and scores 0.25 (run `oxr1`).
  Dream-1 scores 0.0 over the full 1,200 steps (run `oxrD`): both arms draw back towards their bases with their
  grippers closed; the tubing never leaves the packet and the knob is never turned.
- Before the revision (2026-11-18, the rigid oversized fit with no push-back): the demo scored 1.0 (run `oxy12`) and
  Dream-1 0.0 (run `oxyD`).
- The scripted demo's plan is 39.7 s of simulated time.
- The two models, the scoring, the sizes, reach and step-guard checks, and the demo against the real collision hulls
  run without the simulator: `python -m pytest -q test_oxygen_flowmeter.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
