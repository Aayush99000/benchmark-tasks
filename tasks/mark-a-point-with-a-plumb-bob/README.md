# Steady a swinging plumb bob, then mark the worktop under its point: the spot directly below a hole in a shelf

A wall shelf is fixed 45.7 cm (18 in) above the worktop. A plumb line comes down through a hole in it with a 16 oz
brass plumb bob on its end, the bob's point 6 mm above the worktop. A carpenter's pencil lies in a rest on the right.
The robot has to mark the worktop directly below the hole, as a fitter does before drilling for a pipe or cable
that has to drop straight through.

**Instruction:** "Use the plumb bob to mark the worktop directly below the hole in the shelf: steady the swinging bob,
let it hang free and still, make one pencil mark on the worktop under its point, and lift the pencil away."

Everything is full size: a 16 oz bob is a standard one, and 18 in is the usual gap between a worktop and the shelf
or wall unit above it. A plumb bob shows the point below its hanging point only while it hangs free and still. Once hung it swings, and a
heavy bob on a line keeps swinging for minutes. Here it starts 2 cm from plumb and is let go as the trial starts. The
job has three parts, and each can undo the one before:

1. **Stop the swing by hand.** A 454 g weight on a 45 cm line has to be brought to rest on its own plumb line.
   Waiting does not work: left alone, the swing halves in about two minutes and the trial lasts one.
2. **Let go without a push.** A hand that holds the bob a few millimetres off its plumb line, or opens unevenly, sets
   it swinging again as it lets go.
3. **Mark under the point without touching.** The point hangs 6 mm above the worktop, so the pencil has to come in
   at a slant, under the bob, and touch down within 3 mm of the point below it. A brush against the bob or its line
   moves the reference the mark is being made from.

## Why this task is different

**The thing to be handled is a pendulum.** Tasks on the board and in the task sets it is compared with handle
objects that stay where they are put. A bob on a line stores every push it is given and gives it back as a swing.
Stopping it means taking its energy out through contact, and letting go means leaving it none. We found no accepted
task that asks for this.

**The target cannot be read from the scene.** The point to mark is where the bob's point comes to rest, which
exists only after the robot has steadied the bob. The scoring holds the robot to that: a mark counts only if it was
made while the bob had been hanging free and still, with both grippers clear of it. In our control run the pencil
was put on the right spot from the layout alone, 2 mm from the point, with the bob still swinging: it scored 0.

**Fine work beside something that must not be touched.** The pencil's point has to arrive under a hanging weight
with 6 mm of headroom. Most precise placements on the board are made against fixed parts that guide or stop the
tool. Here the nearest thing is free to move and the mark is spoiled if it does.

**Checked against what exists.** We searched every pull-request title and description on the task board, the
accepted-task list and the reviewers' comments for plumb bobs, plumb lines, pendulums and swinging weights, and the
1,016 activity names of BEHAVIOR-1K and RoboCasa's task list for the same. None has a plumb bob, and no accepted
task asks for a swinging object to be steadied and then worked around. A web search for a plumb bob task in LIBERO,
RoboTwin and ManiSkill found none. The nearest accepted task
marks a board beside a tape measure's reading. It shares the pencil mark as its outcome; its reference (a tape held
out against a spring) is held still by the other hand throughout, where this one has to be brought to rest and then
left alone.

## Where this comes up

- **Carpenters** (O*NET 47-2031.00, 889,700 US workers). O*NET lists "Verify trueness of structure, using plumb bob
  and level" among their tasks. Carrying a point from a top plate, beam or ceiling down to the floor or bench with a
  plumb bob is everyday layout work.
- **Plumbers, pipefitters and steamfitters** (O*NET 47-2152.00, 510,600 US workers): "Locate and mark the position
  of pipe installations, connections, passage holes, or fixtures in structures". A hole in one level has to be
  matched by a mark on the next, straight below.
- **Others who plumb a point:** electricians setting boxes and pendant fittings, drywall and ceiling installers,
  masons, surveying technicians centring an instrument over a ground mark, and lift installers.
- **The same skill elsewhere:** steadying a load on a hoist before setting it down, stilling a hanging lamp, sign
  or sample before working on it, and any job done beside a suspended part.

## Potential impact

- **Damping by touch.** Measures whether a policy can take the motion out of a hanging object, and tell when it is
  out, from what it sees and feels.
- **A release that leaves no motion behind.** Opening a hand is usually the end of a manipulation. Here it is the
  step most likely to spoil it.
- **Precision next to a free body.** A 3 mm mark under a weight that moves if brushed.
- **Graded progress.** Partial credit separates a policy that never steadies the bob (0), one that steadies it
  (0.25), one that marks within 10 mm (0.5) and one that marks within 3 mm and leaves everything right (1.0).

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: the shelf, line, bob, pencil and rest and their layout, the instruction, what happened during the trial, scoring |
| `plumbkit.py` | Builds the plumb bob on its line (one articulation with a fixed hanging point), the pencil, the pencil dots and the fixed boards |
| `demo.py` | The scripted demo, and the control runs (`--mode nosteady`, `--mode steady`, `--mode still`) |
| `test_scoring.py` | Offline tests of the scoring rule, the layout and the write-up (no simulator) |
| `task.json` | The submission's details |

## Requirements

The yam_arena embodiment and Isaac Lab-Arena, as set up by the program's GPU instance image (an NVIDIA RTX GPU is
needed). `test_scoring.py` needs only Python with numpy, pytest and the `yam_arena` package.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Use the plumb bob to mark the worktop directly below the hole in the shelf: steady the swinging bob, let it hang free and still, make one pencil mark on the worktop under its point, and lift the pencil away." \
  --scorer task_score --max-steps 1200
```

Run limit: 1,200 steps (60 s at 20 Hz). The demo reaches full score at step 680 (34 s), so the limit leaves room for
a slower policy, and no room to wait for the swing to die away.

`PLUMB_LOT=B` in the environment selects a second layout: the shelf, line and bob 2 cm nearer the arms and 3 cm to
the left (the point under the hole at x 30, y 3). The default is layout A, described below.

## Scripted demo

```bash
python demo.py task.py out_dir                    # the demo
python demo.py task.py out_dir --mode nosteady    # control: the same mark, made with the bob left swinging
python demo.py task.py out_dir --mode steady      # control: the bob steadied, no mark
python demo.py task.py out_dir --mode light       # control: the pencil's point touches the worktop too lightly to mark
python demo.py task.py out_dir --mode still       # control: the arms do not move
```

1. The left hand comes in from the left, open, level with the bob's body, a finger either side of the swinging bob.
   It closes slowly until both fingers just press on the body, at its centre of mass. The swing stops between
   them.
2. It opens again slowly, each finger drawing back 1.4 mm a second. The bob follows the finger it was leaning on
   back to its own plumb line and is left there at rest. The hand draws back 12 cm.
3. Meanwhile the right hand has taken the pencil from its rest by its upper end, at the slant it lay at.
4. The right hand brings the pencil in along its own slant, under the bob, until its point touches the worktop
   below the bob's point, and presses it on: the hand goes on 3 mm along the pencil, the arm gives, and the point
   bears on the worktop with about 0.6 N. Then it draws the pencil back the same way.

The demo uses only the layout constants in `task.py`, the sizes in `plumbkit.py`, one measured constant (where the
pencil's point sits in the hand, about 1 mm off nominal) and the robot's own measured hand positions
(proprioception), which it uses to correct an arm's sag before a placement. It never reads the bob's or the pencil's
state, and it does not read the force on the pencil.

## Scoring

Tools: a ruler, the trial video, and two close-up cameras on the bob's point (one looking along x, one along y).

- **P** is the point on the worktop directly below the hole. On a real bench it is found by letting the bob settle
  and looking under its point; in the simulator it is known.
- **A mark:** the pencil leaves a mark only while its point is in contact with the worktop and is pressed on it with
  at least 0.3 N, a light pencil stroke (about the weight of 30 g on the point). A point held just above the
  worktop leaves nothing, and neither does one that rests on it more lightly than that. The mark runs as far as the
  pressed point is dragged, until the point is lifted 4 mm. A mark's distance is measured from P to its farthest
  part. On a real bench the mark is the graphite on the paper, which a pencil likewise leaves only where it is
  pressed; in the simulator the force is read from a spring under the lead (see Fidelity).
- **Hanging free and still:** the bob's point has stayed within 3 mm of plumb (measured across, so a swing of 3 mm
  either side at most) for 1.5 s, longer than one swing, with every part of both grippers' fingers and palms at least
  2 cm clear of the bob and 3.6 cm from its line. The simulator checks points along both fingers' inner and outer
  faces and on the palm (`gripper_points()` in `plumbkit.py`). On a real bench it is read from two calibrated
  close-up views of the bob's point, one camera looking along x and one along y, both level with the point: tape a
  steel rule to the worktop under the point, square to each camera's line of sight, and work out each view's scale
  from it (pixels per millimetre at the point's distance). After the trial, let the bob settle and take that as
  plumb in both views. The point is within 3 mm of plumb when the two readings, combined (the root of the sum of
  their squares), are 3 mm or less; it has to stay so for 1.5 s. The grippers' clearance is read from the overhead
  video.
- **A plumbed mark** is one made while the bob was hanging free and still.

| At the end of the trial | Score |
|---|---|
| At least one mark; every mark plumbed and within 3 mm of P; the pencil's point at least 4 mm off the worktop; the bob hanging free and still | **1.0** |
| Otherwise: a plumbed mark within 10 mm of P | **0.5** |
| Otherwise: the bob hung free and still at some time in the trial | **0.25** |
| Anything else | **0** |

Every outcome falls into exactly one row. Things that earn nothing by design: a mark made while the bob swings or
is held (it is not plumbed), a mark more than 10 mm out, and waiting. Things that cost the full score: a second mark
anywhere that is not itself plumbed and within 3 mm (a stray mark or a scribble), the pencil left down, and the bob
knocked after the mark. 3 mm is an eighth of an inch, the usual tolerance for this kind of layout. In code:
`_watch()`, `score_from()` and `score()` in `task.py`.

## Verification

| Run | Score | Notes |
|---|---|---|
| Scripted demo, layout A | 1.0 | Swing of 20 mm stopped between the fingers; the bob left hanging within 0.2 mm of plumb, free and still by step 395 (20 s); the pencil's point pressed on the worktop with 0.49 N at step 647: one mark, 0.5 mm from P; full score at step 680 (34 s) |
| Scripted demo, layout B | 1.0 | The bob left within 0.1 mm of plumb; point pressed on with 0.50 N; one mark 1.1 mm from P; full score at step 680 |
| Touch without pressing (control) | 0.25 | Everything as in the demo, but the hand stops 0.8 mm along the pencil past the touch instead of 3 mm: the point came to 0.13 mm from the worktop with no force on it, and left no mark. The bob had been steadied, so 0.25. Under the earlier rule (a mark whenever the point is within 0.5 mm) this run would have counted as a mark |
| Mark without steadying (control) | 0.0 | The same pressed mark made with the bob left swinging 15 mm and more either side: it landed 1.2 mm from P and does not count |
| Steady only (control) | 0.25 | The bob steadied and left hanging; no mark |
| Arms still (control) | 0.0 | The swing went from 20 mm to 14 mm in the 60 s of the trial: it halves in about two minutes (run before this revision; the bob and its line are unchanged) |
| Dream-1, 1,200 steps | 0.0 | Both grippers reached toward the swinging bob without closing on it; then the right gripper hovered over the pencil in its rest and the left drew back. The bob was never steadied and no mark was made (run before this revision: the pencil was never picked up, so the marking rule was not in play) |
| `pytest test_scoring.py` | 22 passed | The score tiers and what spoils a full score, what makes a mark (a point held just above the worktop, or resting on it too lightly, makes none; a pressed point does), the bob's hanging and starting points, the swing outlasting the run limit, the pencil's clearance under the bob, both layouts, the write-up's limits |

## Runtime and seeds

About 2.5 minutes per demo run and about 4 minutes per Dream-1 run on one L40S, including about a minute of
start-up. `object_jitter=0`; the layout is
fixed, and there is no randomness in the task.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table, which is the worktop (x forward, away from
the arms; y to the left; heights above the worktop). Layout A:

- **The wall shelf**, fixed: 18 mm plywood, 30 cm wide (y) and 20 cm deep (x), its underside 45.7 cm (18 in) above
  the worktop, the usual height of a wall unit over a worktop. Its front edge is at x 26 and it is centred on y 0.
  It is carried by an 18 mm plywood backboard, 30 cm wide and 47.5 cm tall, standing at x 46.
- **The hole** for the line is 6 cm in from the shelf's front edge, at x 32, y 0. The line is tied to a brass screw
  beside it on the shelf's top.
- **The plumb line:** 2 mm orange braided line, 33.1 cm long from the hole's lower edge to the bob.
- **The plumb bob:** 16 oz (454 g), brass, 12 cm long: a taper 6 cm long down to a steel point, a body 3.2 cm across
  and 4.5 cm long, and a cap 1.6 cm across and 1.5 cm long that the line is tied into. Hanging still, its point is
  6 mm above the worktop over x 32, y 0.
- **The start:** the bob is at rest with its point 2 cm toward the arms (over x 30), the line straight, and is let
  go as the trial starts. It swings 2 cm either side along x, once every 1.2 s.
- **The carpenter's pencil:** flat, 15 x 7 mm, 17.5 cm long, 10 g, red, sharpened to a point over its last 2.5 cm.
  It lies on its wide face in its rest, sloping 50 degrees from the worktop: its point down at x 24, y -19, 1.2 cm
  up; its other end at x 24, y -30.2, 14.6 cm up.
- **The pencil's rest**, fixed, pine: a groove 16.5 mm wide and 11.7 cm long sloping at 50 degrees, with a stop at
  its lower end that the pencil's point rests against, on a post and a foot plate 5 x 13 cm.
- **Robot:** both arms start in the raised ready pose with their grippers open.

## Fidelity

- **The line is a stiff rod.** It is a 2 mm rod hinged freely at the hole's lower edge and at the bob's cap (two
  crossed hinges at each), so the bob swings on the line and rocks on its end as a real one does. It cannot go
  slack or bend. That is exact while the bob hangs, which it does throughout the task; it would be wrong for a bob
  lifted or laid down.
- **How slowly the swing dies.** A heavy bob in still air loses its swing over minutes. The joints carry a little
  damping for the air and the line (`TOP_DAMP`, `KNEE_DAMP` in `plumbkit.py`), set so that the free swing halves in
  about two minutes (measured: 20 mm to 14 mm in 60 s). Part of each value cancels a known effect of the simulator's
  integrator at 240 Hz, which would otherwise make a free swing grow by about 2% a second; the comment beside the
  constants gives the amounts.
- **The pencil's mark needs contact and pressure.** The lead's point rides on a short stiff spring in the pencil's
  wood (1.5 mm of travel, 600 N/m), so the force it is pressed with can be read: the spring's compression times its
  stiffness, along the pencil. The task file lays a small dark dot only while the point is on the worktop (its tip
  within 0.3 mm of the surface, where it rests in contact) and that force is 0.3 N or more: one dot where it is
  first pressed, another every 2 mm it is dragged. Nothing appears from the point being near the worktop. A real
  lead is rigid: the spring is the simulator's force gauge, not a feature of the pencil. The 0.3 N is an estimate
  of a light pencil stroke, not a measurement; the pencil's own weight on its point is 0.1 N, and the demo presses
  with about 0.6 N. The lead does not wear or break.
- **The hole** is drawn, not cut: the line hangs from a fixed point at the shelf's underside, which is where a real
  line bears on the edge of its hole.
- **Contact** between the fingers, the bob, the pencil and the worktop is ordinary rigid-body contact with friction.
  Nothing is attached to a gripper or moved by the task file except the pencil dots.

## Real-world build

- **The shelf:** a 30 x 20 cm board fixed 45.7 cm above the bench on a backboard, with a 6 mm hole 6 cm in from its
  front edge.
- **The plumb bob:** a 16 oz brass bob on braided mason's line, through the hole and tied off on top, with its
  length set so the point hangs 6 mm above the bench.
- **The pencil:** a sharpened carpenter's pencil in a block with a 50 degree groove and a stop.
- **Each trial:** tape a fresh sheet of paper under the bob (or rub out the old mark), put the pencil back, draw the
  bob 2 cm toward the arms and let it go as the trial starts.
- **Scoring:** from the two calibrated close-up views (see Scoring), whether the bob was hanging free and still
  when each mark was made and at the end. Then let the bob settle and measure with a ruler from the point under it
  to the farthest part of each pencil mark.

## Third-party assets

None. Every part is built from primitive shapes in `plumbkit.py`.
