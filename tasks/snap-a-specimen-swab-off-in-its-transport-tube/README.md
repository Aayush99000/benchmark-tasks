# Snap a specimen swab's handle off at its breakpoint in a free-standing transport tube, and bin the handle

A swab that has just been taken stands in its open transport tube: flocked tip down in the medium, handle sticking
6 cm out of the top. The robot has to do what ends every swab collection: bend the handle so the shaft snaps at its
moulded breakpoint, leave the tip in the tube, and throw the handle away, with the tube left standing.

**Instruction:** "Snap the swab's handle off at its breakpoint, leaving the swab's tip in the transport tube, and drop
the handle in the waste cup. Leave the tube standing."

The scene is the common kit for throat, nose and wound swabs: a 16 x 100 mm tube with 3 mL of transport medium and a
regular flocked swab with a breakpoint 80 mm from its tip. The maker's directions for it are: put the swab in the tube,
hold the tube, hold the end of the shaft and bend it to break at the breakpoint (then cap the tube). Three things make
that hard for a robot:

1. **The tube has to be held, and the instruction does not say so.** The open tube with its medium weighs 10 g and
   stands on a base 16 mm across. A sideways push of 0.007 N on the handle is enough to tip it; snapping the shaft takes
   about 1.1 N at the same place, over 100 times more. With nothing holding the tube, a push on the handle knocks the
   tube over and the swab stays whole.
2. **The shaft has to snap in the right place.** The breakpoint sits 8 mm inside the rim only while the tip is down
   on the tube's floor. Bent there, the shaft levers between the rim and the tube's wall and snaps at the breakpoint,
   and the specimen stays in the medium. Lifted first, it snaps with the specimen up in the air.
3. **A thin shaft in a loose tube.** The handle is 2.5 mm thick and only leans on the rim, so it is wherever it was
   last nudged. Fingers that close on the tube first jolt it a millimetre, and the swab flops to the other side of the
   rim. The pinch on the shaft has to let it turn as it bends.

## Why this task is different

**One hand braces what the other loads, by physics.** The holding hand is not named in the instruction; the numbers
above make it necessary, and the control run without it scores 0 (the tube ends on its side with the swab unbroken).

**A part designed to break, and to break in one place only.** The score depends on where the swab was when it
snapped, not only on whether the handle came off. A policy that pulls the swab up to get a better grip, then bends it,
gets the handle off and still scores 0.

**The whole object is small and light.** The swab weighs about a gram and the tube ten. Nothing here can be found by
pushing against it: a touch moves it.

**Nearest tasks on the board, and how this differs.**

- "Snap a circuit board out of its panel over the break edge" (TK-X7G4S2, accepted) also breaks a part at a scored
  line by bending it over an edge. There the edge is a steel block fixed to the bench, which takes the load. Here the
  edge is the rim of a loose 10 g container that falls over unless the other hand holds it, the part is a 2.5 mm
  shaft taken in the fingertips, and the piece that matters has to stay inside the container.
- A pending task wrings a strep-test swab in a soft tube (squeeze, turn five times, draw the swab out). It uses a
  swab and a tube for a different step: nothing is snapped, and the swab leaves the tube.

## Where this comes up

- **Clinics, urgent care, pharmacies and hospital wards.** Throat, nose, wound and screening swabs all end with this
  step. Collecting and preparing specimens is a core task of medical assistants (O*NET 31-9092.00: "Collect blood,
  tissue, or other laboratory specimens, log the specimens, and prepare them for testing"; over 800,000 US workers),
  and a routine one for registered nurses (29-1141.00) and phlebotomists (31-9097.00).
- **Home test kits.** Many mail-in swab kits ask the person at home to snap the swab off in the tube in the same way.
- **The same skill elsewhere.** Break-off parts are common: snap-off ampoule necks, break-off applicator sticks, tear
  bars, sprues. Each needs the base braced and the bend made where the part is meant to give.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: the layout (two tube positions), the instruction, what happened during the trial, scoring |
| `swabkit.py` | Builds the tube, the swab (two lengths joined at the breakpoint by a joint that lets go for good at its break moment) and the waste cup |
| `demo.py` | The scripted demo, and its control runs |
| `test_scoring.py` | Offline tests of the scoring rule, the geometry and the two-hand numbers (no simulator) |
| `task.json` | The submission's details |

## Requirements

The yam_arena embodiment and Isaac Lab-Arena, as set up by the program's GPU instance image (an NVIDIA RTX GPU is
needed). `test_scoring.py` needs only Python with numpy, pytest and the `yam_arena` package.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Snap the swab's handle off at its breakpoint, leaving the swab's tip in the transport tube, and drop the handle in the waste cup. Leave the tube standing." \
  --scorer task_score --max-steps 1200
```

Run limit: 1,200 steps (60 s at 20 Hz). The demo reaches full score at step 532 (27 s) on layout A and step 442
(22 s) on layout B, so the limit leaves room for a slower policy.

The tube stands in one of two places. Layout A is the default; set the environment variable `SWAB_LOT=B` for the
other (2 cm further away and 2 cm to the left). The waste cup does not move.

## Scripted demo

```bash
python demo.py task.py out_dir                  # the demo (SWAB_LOT=B for the second layout)
python demo.py task.py out_dir --mode nohold    # control: the same bend with nothing holding the tube
python demo.py task.py out_dir --mode lifted    # control: lift the swab 3 cm, then bend it
python demo.py task.py out_dir --mode still     # control: the arms do not move
```

1. The left hand comes in from the left, 15 degrees below level, its fingers open either side of the tube 2 cm below
   the rim. The right hand, pointing straight down, comes in from the near side with its fingers either side of the
   handle 3 cm above the rim.
2. The right hand closes on the handle first; then the left hand closes on the tube and holds it for the rest of the
   job.
3. The right hand pushes the handle 22 mm away from the robot. The shaft turns in the fingers and levers between the
   rim and the tube's wall until it snaps at the breakpoint, about half-way through the push (both arms give under
   the load).
4. It lifts the handle clear, carries it over the waste cup and lets it go. The left hand lets go of the tube and
   draws back.

The demo uses only the layout constants in `task.py`, the sizes in `swabkit.py` and each hand's own measured position
(proprioception), which it uses to correct an arm's sag before a grasp. It never reads the tube's or the swab's state.

## Scoring

Tools: the trial video from the side with a ruler in view, a protractor (or the video's frame), a ruler.

- **Snapped:** the handle has come off the rest of the swab. (In the simulator: the handle's lower end is more than
  4 mm from the head's upper end, or the two are more than 12 degrees out of line. A whole swab's joint gives less
  than that under load, see Fidelity.)
- **Snapped in place:** in the last video frame before the handle comes off, the swab's tip is inside the tube and
  no more than 15 mm above the tube's inside floor, and the tube is no more than 45 degrees from upright. On video:
  the red ring at the breakpoint, which sits 8 mm below the rim when the tip is on the floor, is no more than 7 mm
  above the rim.
- **Spilled:** at any time, the tube is more than 60 degrees from upright.
- **Tip exposed:** at any time, the swab's tip is outside the tube.
- **Handle binned:** at the end, the middle of the handle is inside the waste cup (below its rim, within its inside
  diameter) and the cup is within 30 degrees of upright.
- **Left standing:** at the end, the tube stands on the table within 5 degrees of upright, its base no more than
  3 mm off the table, with the tip inside it; the tube, both lengths of the swab and the cup are at rest (under
  2 cm/s); and no part of either gripper is within 3 cm of the tube or of the handle.

| Outcome | Score |
|---|---|
| Not snapped; or not snapped in place; or spilled; or the tip exposed | **0** |
| Otherwise: handle binned, and left standing | **1.0** |
| Otherwise: handle binned | **0.75** |
| Otherwise (snapped in place, handle not in the cup) | **0.5** |

Every outcome falls into exactly one row (the first that applies). In code: `_watch()`, `score_from()` and `score()`
in `task.py`; `joined()` and `apart()` in `swabkit.py`.

## Verification

All runs use the submitted scoring code.

| Run | Score | Notes |
|---|---|---|
| Scripted demo, layout A | 1.0 | Shaft snapped at step 208, during the push, with the tip 1.5 mm above the tube's floor and the tube 0.3 degrees from upright (0.6 at most). Handle in the cup at step 325; full score at step 532 (27 s) |
| Scripted demo, layout B | 1.0 | Snapped at step 214 with the tip 0.3 mm above the floor; the tube tilted 2.2 degrees at most. Full score at step 442 (22 s) |
| No holding hand (control) | 0.0 | The same grasp and a 30 mm push with the left hand kept away: the tube went over onto its side (spilled at step 310) and the swab never snapped (its two lengths were still one piece at the end) |
| Lifted first (control) | 0.0 | The tube held, the swab lifted 3 cm and then bent: it snapped with its tip 31 mm above the tube's floor. The handle went in the cup and the tube was left standing, and it still scores 0 |
| Arms still (control) | 0.0 | Nothing moves; the swab stays whole |
| Dream-1, 1,200 steps, layout A | 0.0 | Both grippers reached toward the tube and the waste cup and hovered near them without closing on either, then drew back and stayed low near their bases. The swab was left whole in the standing tube |
| `pytest test_scoring.py` | 16 passed | The score ladder and its zero cases, the swab's starting seat, the breakpoint's place inside the rim, the "one piece" limits, the tip-in-tube and in-cup tests, and the two-hand numbers (the break moment is over 20 times the moment that tips the tube) |

What we learned building the demo, which shows what the task asks of a policy:

| Development run | What happened |
|---|---|
| Right hand swung straight to the handle | Its body clipped the top of the handle on the way and knocked the swab 5 mm round the rim; the fingers then closed beside the shaft |
| Tube gripped first, then the handle | Closing on the tube jolted it 1.2 mm and the swab flopped to the near side of the rim, out of the waiting fingers |
| Handle pinched first, 10 mm push | The shaft did not snap during the push: both arms gave way under the load. It broke later, as the handle was lifted, with the tip 27 mm up |
| Handle pinched first, 22 mm push (final) | Snapped half-way through the push with the tip on the floor |

## Runtime and seeds

About 1.7 minutes per demo run and about 4 minutes per Dream-1 run on one L40S, including under a minute of start-up.
`object_jitter=0`; each layout is fixed, and there is no randomness in the task.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table (x forward, away from the robot; y to the
left).

- **Transport tube:** a 16 x 100 mm skirted, free-standing transport tube with its cap off, base centred at x 26,
  y -10 (layout A) or x 28, y -8 (layout B). Clear plastic, wall 1.25 mm (bore 13.5 mm), white skirt; the inside floor
  is 12 mm above the table. It holds 3 mL of orange-pink transport medium (21 mm deep). Tube and medium together: 10 g.
- **Swab:** a regular flocked swab, 15 cm long: a white flocked tip 5.5 mm across and 16 mm long on a white plastic
  shaft 2.5 mm across, with a moulded breakpoint 80 mm from the tip marked by a red ring. About 1.1 g. It stands in
  the tube with its tip in the middle of the tube's floor and its shaft leaning on the far side of the rim (3.4
  degrees from upright, away from the robot). The breakpoint is 8 mm below the rim; the handle sticks 6 cm out.
- **Waste cup:** an open grey plastic cup, 10 cm across inside and 8 cm tall, 20 g, base centred at x 32, y -30.
- **Robot:** both arms start in the raised ready pose with their grippers open.

## Fidelity

- **Loose rigid bodies.** The tube, the two lengths of the swab and the cup are free rigid bodies with friction
  (tube 0.5, swab 0.6, cup 0.6). Nothing is fixed to the table.
- **The breakpoint is a joint that lets go for good** once the bending moment across it passes 0.03 N m or the pull
  across it passes 60 N. Those are calculated from the material (a 1.5 mm polystyrene neck carrying about 90 MPa in
  bending and 35 MPa in tension), not measured on a swab. A real shaft bends well past straight before it snaps; the
  rigid model snaps after about a centimetre of push at the handle.
- **A whole swab's joint gives a little.** Between two bodies of half a gram each, the simulator's joint gave up to
  1.5 mm and 7 degrees in these runs while still whole (pinched and lifted with the tip dragging in the tube). The
  "one piece" limits are therefore 4 mm and 12 degrees; a snapped swab passes them within one control step, because
  its lower length drops free.
- **The medium is drawn, not simulated.** Its weight is part of the tube's. Spilling is judged by tilt: more than
  60 degrees counts as spilled. A still tube with 3 mL in it would not pour until about 85 degrees; the margin is for
  the slosh of a tube being knocked over.
- **Sizes.** The tube's 16 x 100 mm, its 3 mL of medium and the swab's 80 mm breakpoint are the kit's catalogue
  figures. The shaft's and tip's diameters, the wall thickness, the skirt's height and the masses are estimates.
- **Left out:** capping the tube afterwards.

## Real-world build

- **Parts:** a 16 x 100 mm skirted transport tube with 3 mL of medium (or water) and a regular flocked swab with an
  80 mm breakpoint, as sold together for specimen collection; a 10 cm cup. Mark the tube's place on the table.
- **Each trial:** a new swab, tip on the tube's floor, handle leaning away from the robot; cap off; cup empty.
- **Scoring:** film from the side with a ruler in view. Check where the red ring was against the rim in the last
  frame before the handle came off, the tube's greatest tilt, that the tip never left the tube, where the handle ended
  and that the tube stands with the grippers clear.
- **Safety:** use sterile, unused swabs and plain medium or water; nothing here is sharp or under pressure. The
  snapped end of a shaft is blunt plastic.

## Third-party assets

None. Every part is built from primitive shapes in `swabkit.py`.
