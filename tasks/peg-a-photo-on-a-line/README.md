# Lift a photo that hangs by one corner back up to its wire, and peg its free corner there

A photo print hangs from a taut photo-display wire by one wooden clothes peg at its left top corner. Held by that one
corner it has swung down: its free top corner hangs 10 cm below the wire. Spare pegs are clipped on the rim of a peg
basket to the right. The robot has to peg the photo's other top corner to the wire too.

**Instruction:** "The photo hangs from the wire by one peg. Peg its other top corner to the wire too."

The instruction does not say how, and one hand cannot do it:

- **The corner is not at the wire.** It hangs 10 cm below it. Lifted to the wire and let go, the print swings straight
  back down.
- **A peg takes a whole hand.** A clothes peg is two wooden legs on a pivot with a torsion spring: squeezing its
  handles together opens its jaws, and letting go closes them on whatever is between them. It has to be squeezed to
  come off the basket's rim and kept squeezed all the way to the wire.

So one hand has to lift the print's hanging end until its top edge is back at the wire and keep it there, while the
other brings the open peg down so that its jaws pass either side of the 1.2 mm wire and the 0.3 mm print in front of
it together, and lets go. Only then can the lifting hand let go.

## Where this comes up

- **Home and classroom displays.** Hanging photos, drawings and cards on a wire or string with pegs is a common way
  to show them at home, in classrooms, nurseries, cafés and shops. A print pegged by one corner droops; pegging the
  second corner is a two-handed job for a person too.
- **Laundry.** The same peg squeeze-and-clip, with the same spring, is how washing is pegged out on a line, one hand
  holding the cloth up to the line while the other pegs it.
- **Holding something in place while fastening it.** Taping a poster, pinning fabric, clipping a sheet to a board:
  the part sags until it is fastened, and the fastener needs the other hand.

## What it measures

- **Two arms needed by physics, not by the instruction.** Without the lift the peg closes on the bare wire (control
  run below). With the lift but no hold, the print is back down before the peg arrives (control run below).
- **A spring mechanism worked by the gripper.** The robot has to squeeze, hold and release a sprung tool rather than
  grasp a rigid object, and the tool's state (open or shut) depends on how hard it is held.
- **Thin-object alignment.** The open jaws are 8 mm apart at their tips and have to take in a 1.2 mm wire and a
  0.3 mm print together, 16 cm above the table.
- **Not disturbing what is already there.** The print hangs from one peg and comes out of it at a 2 N pull; a print
  pulled off its first peg scores 0.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: scene (two layouts), the trial's record, scoring, the instruction |
| `pegkit.py` | Builds the clothes pegs (an articulation each: two legs, a sprung pivot, a groove for the line, soft rubber grips), the photo print, the fixed parts, and the first peg's hold on the print |
| `demo.py` | The scripted demo and its control runs |
| `test_scoring.py` | Offline tests of how the photo hangs, what counts as a peg holding it, the trial's record and the scoring rule (no simulator) |
| `task.json` | The submission's details |

## Requirements

The yam_arena embodiment and Isaac Lab-Arena, as set up by the program's GPU instance image (an NVIDIA RTX GPU is
needed). `test_scoring.py` needs only Python with numpy, pytest and the `yam_arena` package.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The photo hangs from the wire by one peg. Peg its other top corner to the wire too." \
  --scorer task_score --max-steps 1200
```

Run limit: 1,200 steps (1 minute at 20 Hz). The demo reaches full score at step 488 (24 s) on both layouts.

The photo hangs in one of two places along the wire. Layout A is the default; set the environment variable `PEG_LOT=B`
for the other (1.5 cm further right). The wire, the basket and the spare pegs do not move.

## Scripted demo

```bash
python demo.py task.py out_dir                    # the demo (PEG_LOT=B for the second layout)
python demo.py task.py out_dir --mode nolift      # control: the peg alone, the photo left hanging
python demo.py task.py out_dir --mode liftonly    # control: the photo lifted, held 2 s and let down; no peg
python demo.py task.py out_dir --mode early       # control: the photo lifted but let down before the peg is brought
python demo.py task.py out_dir --mode slide       # control: a spare peg squeezed, slid 2.5 cm along the rim and left there
python demo.py task.py out_dir --mode still       # control: the arms do not move
```

1. The right hand, leaning 30 degrees out to its own side, comes down on a spare peg on the basket's rim, its
   fingers on the peg's soft grips, and squeezes: the jaws open and let go of the rim.
2. Meanwhile the left hand, pointing forward and 25 degrees down with its fingers shut, goes in under the hanging
   photo, meets its bottom edge 10 cm from the pegged corner and swings it up about its peg until it hangs level. It
   stays there, the photo resting on its fingers.
3. The right hand carries the peg, still squeezed open, in level from beyond the photo's free end to over its free
   top corner, then down so that the open jaws pass either side of the wire and the photo's edge together, and lets
   go. The peg settles on the wire by the groove in its jaws, its tips pinching the photo. The hand rises clear.
4. The left hand lowers its fingers away from the photo and draws back. The photo hangs level from two pegs.

Every pose comes from the layout constants in `task.py` and each hand's own measured position (proprioception), with
one exception: a squeezed peg does not always settle the same way in the fingers (its jaw tips can sit several
millimetres either way), so above the wire the demo reads that peg's pose once and aims the middle of its open jaws
from it. Otherwise object states are read only for the result file.

## Scoring

Tools: the trial video, a ruler and a protractor.

- **0 for the whole trial** if at any time the photo comes off its first peg (its pegged corner 1 cm or more from
  where it hung).
- Otherwise **1.0** once a second peg holds the photo's free top corner on the wire, everything let go, for
  2 seconds. That means all of:
  - the peg has been let shut by its spring (its jaws under 4 degrees open, not squeezed open by fingers);
  - the wire lies between its two jaws;
  - the photo's paper lies between its two jaws too, and the jaws' tips pinch it on the sheet, 2 mm or more inside
    the photo's edges and within 2 cm of its free end;
  - both grippers are more than 4 cm from that peg and from the photo;
  - the photo is within 5 degrees of level;
  - the peg and the photo are at rest (each moves less than 1 mm in half a second).

  A peg on the wire beside the photo, or on the wire above a photo that still hangs below it, does not count.
- Otherwise **0.25 for each of**:
  - a spare peg has been taken off the basket's rim: for a quarter of a second the rim is no longer between its legs,
    and it has not dropped into the basket. Squeezing a peg open on the rim, or sliding it along the rim, however
    far, does not count;
  - the photo's hanging end has been raised to the wire: the photo within 10 degrees of level for 1 second.

So the scores are 0, 0.25, 0.5 and 1.0, and every outcome has one.

**With a ruler and a protractor, in the real world:**
- *Off the rim* means the peg is no longer astride the basket's edge.
- *Raised* is read from the video with a protractor on the photo's top edge against the wire.
- *Holds the corner* means the wire and the photo's edge are both between the peg's jaws, the jaws' tips are on the
  photo within 2 cm of its end, and the photo stays level when both hands have gone.

In code: `peg_on_rim()`, `taken_off()`, `grip_check()`, `holds_free_corner()`, `advance()`, `_watch()`, `score_from()`
and `score()` in `task.py`.

## Verification

All runs use the submitted scoring code.

| Run | Score | Notes |
|---|---|---|
| Scripted demo, layout A | 1.0 | Photo swung up level by step 180 and held there; spare peg off the rim and counted as taken at step 225, and let go on the wire and the photo's corner at about step 375: wire in its jaws, tips on the photo 16 mm in from its free end. Left hand away by step 440; the photo stayed 0.3 degrees from level on its two pegs. Full score at step 488 (24 s) |
| Scripted demo, layout B | 1.0 | The same steps; tips on the photo 9 mm in from its free end; photo 0.1 degrees from level. Full score at step 488 |
| Peg without the lift (control) | 0.25 | The right hand alone took a peg to where the corner would be and let go: it hangs on the bare wire. The photo still hangs 61 degrees down, 10 cm below it. Credit for the peg taken only |
| Lift without the peg (control) | 0.25 | The left hand swung the photo up level, held it 2 s and let it down: it swung back to hanging. Credit for the raise only |
| Photo let down before the peg comes (control) | 0.5 | Lifted, then let down; the peg then went on the bare wire above it. Both part credits, and no more |
| Spare peg squeezed and slid along the rim (control) | 0.0 | Squeezed open, slid 2.8 cm along the rim and let go there: never off the rim, so not taken |
| Arms still (control) | 0.0 | The photo hangs from its first peg; nothing moves |
| Dream-1, 1,200 steps, layout A | 0.0 | The right gripper went to the peg basket and hovered over it, then moved up over the wire's right end; no peg left the rim. The left arm stayed raised and back; the photo was not lifted |
| `pytest test_scoring.py` | 24 passed | How the photo hangs; a peg holding the corner (and not: beside the photo, above a photo still hanging, in the middle of the photo, squeezed open, shut in front of the sheet), a crooked peg; off the rim (and not: squeezed open on it, tipped on it, slid along it, dropped in the basket); the quarter-second, one-second and two-second counts; the photo off its first peg; the score ladder; the layout and the first peg's hold |

The videos show the overhead camera, the right wrist camera and the left wrist camera side by side.

What we learned building the demo, which shows what the task asks of a policy:

| Attempt | What happened |
|---|---|
| Fingers lifting straight up under the hanging photo's lower edge (worked out on paper, not run) | Nothing would lift: there the push points almost straight at the peg the photo hangs from. The fingers have to go behind the bottom edge and sweep it round, sideways first and then up |
| Peg carried straight from the basket up to the photo's corner | The photo was knocked out of its first peg as the open peg arrived (0). The peg now comes in level from beyond the photo's free end, its jaw tips 8 mm above the photo |
| Peg aimed by its pivot | A squeezed peg sits tipped about 10 degrees in the fingers, so its jaw tips are several millimetres off to one side of its pivot; one jaw came down on top of the wire. The middle of the open jaws is aimed instead |
| Peg's jaw let go with the wire below its groove | The peg shut on the wire and photo for a moment, then dropped until the wire sat in the groove, as a real peg does. It is let go a little high and left to settle |

## Runtime and seeds

About 2 minutes per demo run and about 3.5 minutes per Dream-1 run on one L40S, including under a minute of
start-up. `object_jitter=0`; each layout is fixed, and there is no randomness in the task.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table (x forward, away from the robot; y to the
left).

- **Wire:** a 1.2 mm steel photo-display wire, taut between two brown wooden posts (2 cm square, 17.2 cm tall) 30 cm
  apart, running across the table (along y) at x 30, from y -15 to y 15, 16 cm above the table.
- **Photo:** a 5 x 3.5 in (127 x 89 mm) print (a beach scene) on 250 gsm photo paper, 0.3 mm thick, 2.8 g, its
  picture facing the robot, just in front of the wire. It hangs from one peg that holds it 5 mm in from its left end
  and 12 mm down from its top edge, at x 30, y 3 (layout A) or y 1.5 (layout B). Held by that one corner it hangs
  swung down 61 degrees: its free top corner 10.1 cm below the wire, its lowest corner 1.6 cm above the table.
  Hanging level it would reach 1.2 cm above the wire, with its free end at y -9.2 (A) or y -10.7 (B).
- **Pegs:** three soft-grip wooden clothes pegs, 72 mm long, legs 5 x 10 mm, pivot 30 mm from the jaw tips, 6 g. A
  torsion spring (0.25 N m/rad, wound 7 degrees past shut) presses the tips together with about 1 N; squeezed fully
  open (18 degrees, 10 mm between the tips) it pushes back with about 5 N at the handles. Each jaw has a groove for
  the line 5 to 14 mm up from its tip; handles splayed 8 degrees with green rubber grip pads. One holds the photo's
  corner on the wire. Two spares are clipped on the basket.
- **Basket:** a fixed blue plastic basket, 12 x 12 cm, rim 9 cm tall, walls 3 mm, its near wall along y at x 24 from
  y -30 to y -18; the two spare pegs are clipped on that wall's rim at y -22 and y -26, legs across the wall.
- **Robot:** both arms start in the raised ready pose with their grippers open.

## Fidelity

- **Pegs, photo and wire are rigid.** Wood and paper do not deform; the wire is a taut rigid rod between fixed posts
  (a real wire sags a little under a photo).
- **The first peg's hold is declared, not simulated by friction.** Its jaws hold the print's corner as a pivot: the
  print turns freely in them in its own plane (which is why it hangs swung down), leans up to 20 degrees out of that
  plane against a light spring (0.005 N m per degree, standing in for the paper's own bending), and comes out at a
  2 N pull. The peg itself stays where it is on the wire up to a 3 N push; knocked 5 mm from its place, it lets the
  print drop. The turning friction in the jaws is modelled as light damping. These numbers are estimates, not
  measurements.
- **The second peg is fully simulated:** two legs on a sprung pivot, opened by the fingers' squeeze on the handles,
  hanging on the wire by the groove in its jaws and holding the print by contact and friction at its tips. A 0.3 mm
  rigid card pinched by 1 N sinks a little into the jaws in the simulator (the tips close to 0.1 to 0.2 mm), so the
  pinch is judged by where the tips are on the sheet, not by the jaws' angle.
- **The peg's spring is light.** About 1 N at the tips is less than most wooden pegs give; it is set so that the
  gripper can squeeze the peg fully open. A stiffer peg would need a harder squeeze but not a different method.
- **At rest** is judged by position over half a second, because a pinched print's reported speed chatters in the
  simulator while it does not move.

## Real-world build

- **Parts:** a 1.2 mm stainless photo-display wire stretched taut between two posts 30 cm apart, 16 cm above the
  table, across in front of the robot; a 5 x 3.5 in photo print; three soft-grip wooden clothes pegs; a small
  basket or tub with a 3 mm rim 9 cm tall. Tape marks for the posts and the basket, and a mark on the wire for the
  first peg.
- **Each trial:** peg the print's left top corner to the wire at the mark, lightly enough that the print swings down
  and hangs by that corner; clip the two spare pegs on the basket's near rim.
- **Scoring:** film from the front. Read the photo's angle with a protractor; check by eye that the wire and the
  print are both between the second peg's jaws and that its tips are on the print within 2 cm of the end; check that
  the first corner has not come out of its peg.
- **Safety:** nothing here is sharp, hot or heavy.

## Third-party assets

None: every object is built from primitive shapes in `pegkit.py` and `task.py`. The photo's picture is drawn from
flat shapes; no image file is used.
