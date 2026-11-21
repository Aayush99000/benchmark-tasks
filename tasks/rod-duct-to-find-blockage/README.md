# Push a fish tape through a conduit to the far box and draw its tip out

A short run of 3/4-inch EMT conduit lies strapped on the table in front of the robot, between an open near mouth and a
4-inch square steel box with its cover off. A yellow fibreglass fish tape lies on the table with its brass tip just
inside the near mouth. The robot must push the tape through the conduit, around its bend, until the tip comes out in
the far box, then take the tip in the box, draw it out past a red line taped on the table beyond the box, and let go.
That is the first half of every wire pull through conduit: once the tape's tip is out at the far box, the wire is
fastened to it there and pulled back.

## Why this is electricians' work

Electricians (O*NET 47-2111.00, 821,000 employed in the US in 2025) pull wire through conduit as core work: "Place
conduit, pipes, or tubing, inside designated partitions, walls, or other concealed areas, and pull insulated wires or
cables through the conduit to complete circuits between boxes." O*NET also lists "Thread wire or cable through ducts or
conduits" among their work activities. A pull through a run that already has its boxes starts the same way every
time: push the fish tape in at one box, in short strokes, until it comes out at the other ("Push the tape through the
conduit, using short, firm motions"; "Once the end of the fish tape emerges from the other side of the conduit, it's
time to attach your conductors", sources below). That push is what is scored here, up to the tip being drawn out at the
far box ready for the wire.

The run is short: 49 cm along its axis with one 90-degree bend, like a run of EMT between two boxes over a bench or
across a wall section. Real runs are often longer, with more bends (the electrical code allows up to 360 degrees of
bends between pull points); a longer run needs the same feed, repeated for more strokes. The task does not claim to
test a long run.

Skills: tool use (a flexible tape pushed through a bore in short regrasped strokes with both hands), bimanual
coordination (one hand always holds the tape while the other regrips), finding the tip where it comes out and drawing
it out to a mark.

## What changed in this revision

Reviewer feedback on the previous version: "Importance (not met): Electricians regularly feed fish tape to pull wire, but
locating and marking a blockage is occasional troubleshooting rather than the routine operation scored here." and
"Please keep that domain but change the scene and primary score to test feeding the fish tape through a conduit to an
accessible far box as part of a wire pull; demonstrate and document the revised task."

What was done:
- The blockage, the cover, the ticks and the marker are gone. The scene is now a conduit run that is open at both
  ends: a near mouth with a bushing and a far 4-inch box with its cover off, as when a run is ready to be pulled.
- The primary score is now the feed through the run: 1.0 needs the tape pushed all the way through, around the bend,
  out into the far box, and its tip drawn out past a red line beyond the box, with the tape still running through the
  whole conduit and both hands off it (see Scoring).
- The conduit is now round 3/4-inch EMT (20.9 mm bore) with a bend at the radius of a 3/4-inch EMT hand bender, and the
  fish tape is 4.8 mm (3/16 in) round, the size of a fibreglass fish tape.
- The demo was rebuilt for the new task: the two-hand feed electricians use (below), then one hand takes the tip in
  the box and draws it out.
- The far box is accessible: its cover is off and it sits within the right arm's reach, so the tip is taken out
  there by hand, as an electrician at the far box does before fastening the wire to it.
- Demonstrated: the recorded scripted demo in `demo/` (all three cameras) scores 1.0 on the new score, and the
  shortcut variants score 0.0 (no motion; feed stopped partway) and 0.5 (tip out in the box but not drawn past the
  line); see Scripted demo and Shortcuts.
- Documented: this README, the scene table, the real-trial reset and the scoring all describe the new task only.
- Still Impact on jobs, electricians, and still the routine operation the feedback named: the feed of a wire pull.

## Not the same task as fishing a wire through a conduit with a fish tape

Both are stages of the same wire pull, and neither can be scored by doing the other:

| | fish a wire through a conduit | this task |
|---|---|---|
| start | the tape is already through a clear bent run, both ends out | the tape is outside, its tip just inside the near mouth |
| what the hands do | make a pulling joint between the wire's end and the tape's eyelet, then pull the tape back hand over hand, keeping the joint made through a bend | push the tape in, a few centimetres per stroke with one hand always holding it, until the tip comes out in the far box; take the tip there and draw it out |
| wire | pulled through the run | none: this is the stage before the wire is attached |
| scored | wire end out of the far mouth, joint intact | tape through the whole run, tip drawn out past a line at the far box, hands off |

A policy that can do that task has never pushed a tape through a bore; a policy that can do this one has never made a
joint or moved a wire.

## Sources

- O*NET 47-2111.00 Electricians: core task "Place conduit, pipes, or tubing, inside designated partitions, walls, or
  other concealed areas, and pull insulated wires or cables through the conduit to complete circuits between boxes.";
  detailed work activity "Thread wire or cable through ducts or conduits."; "Employment (2025) 821,000 employees".
  https://www.onetonline.org/link/details/47-2111.00
- ExpertCE, "How to Use a Fish Tape for Pulling Wire Through Conduit": "Push the tape through the conduit, using short,
  firm motions."; "Once the end of the fish tape emerges from the other side of the conduit, it's time to attach your
  conductors."; "One person should slowly and steadily pull the fish tape back through the conduit."
  https://expertce.com/learn-articles/how-to-use-fish-tape/
- 3/4-inch EMT: outside diameter 0.922 in (23.4 mm), inside diameter 0.824 in (20.9 mm).
  https://engineeringtoolbox.com/conduit-size-d_1738.html
- Klein 56204 hand bender for 3/4-inch EMT: "Centerline Bend Radius (Inch): 5". https://www.kleintools.com/product/56204
- Klein 56350 fibreglass fish tape: "Tape Size: 50' x 3/16''". https://www.kleintools.com/product/56350
- EC&M on NEC 358.26: "the maximum degrees of bends permitted in one run is 360°."
  https://ecmweb.com/national-electrical-code/qa/article/21266219/code-qa-nec-requirements-for-emt-bends
- The 4-inch square, 1-1/2 in deep steel box is a standard size; no page was opened for it.

## Scene (station table; cm from the midpoint between the arm bases, x forward, y left, heights above the table)

| object | type | size | material (real rebuild) | position |
|---|---|---|---|---|
| conduit | fixed round tube lying on the table (a 12-sided collider inside a round visual tube) | 3/4-inch EMT: 2.34 cm outside, 2.09 cm bore; 48.9 cm along its axis | galvanised steel EMT, bent with a 3/4-inch hand bender | near mouth at x 35, y 0, open towards +y; straight 27 cm along -y to y -27; one 90-degree bend of 12.7 cm centreline radius turning towards the robot; 2 cm on along -x into the box |
| bushing | fixed ring on the near mouth | 3.0 cm across, 0.9 cm long, flared throat | black plastic EMT bushing | on the near mouth |
| straps | fixed | 3 x 1.6 cm, 1.6 mm thick | two-hole steel straps | over the straight at y -8 and y -19 |
| far box | fixed open box | 4-inch square (10.2 cm inside), 3.8 cm deep, 1.5 mm walls, cover off | steel 4-inch square box, 1-1/2 in deep | knockout wall at x 20.3 (the conduit enters through a 2.8 cm knockout); inside x 10.1-20.3, y -34.6 to -44.8 |
| red line | thin strip on the table | 1.2 x 14 cm | red tape | 3 cm beyond the box's back wall (the wall facing the knockout), across the line the tape comes out on: x 7.0, y -46.7 to -32.7 |
| fish tape | free flexible round tape | 4.8 mm diameter, 1.40 m, 45 g | yellow 3/16-inch round fibreglass fish tape | tip D cm inside the near mouth (D drawn at every reset); out of the mouth straight along +y to y +33, one 180-degree bend of 13 cm radius away from the robot (to x 61), then straight back along -y |
| tip | rigid, on the tape's leading end | 12 x 6 x 6 mm, 4 g | brass leader tip | - |
| handle | rigid, on the tape's other end | 3 x 2.6 x 1.2 cm, 20 g | plain end block (a real tape pays out from a reel case) | at the end of the tape on the table |

## Reset and sampling

At every reset the tip's starting depth D inside the near mouth is drawn uniformly from 3 to 20 cm (to 1 mm) with
`numpy.random.default_rng([seed, 1101])`, where `seed` is the episode seed the runner passes to reset; without one
(the scripted-demo runner) it is `FT_SEED` (default 0) plus the number of earlier resets in the process. The tape is
laid in the shape in the table above with that much of it already in the conduit, and settles for 2 s. So how much
tape is left to push differs from trial to trial by up to 17 cm (about five strokes of 3.5 cm): a fixed stroke count tuned to
one trial stops short on another (at most 0.25) or pushes extra tape into the box. Watching the box shows when the
tip is through.

The conduit itself is fixed: the run, its bend and the box are the same in every trial. Only the tape's starting
depth changes.

Real rebuild: before each trial, pull the tape back out of the box and the conduit, draw D (with the same generator
or a printed list of draws), push the tip in D cm (a mark on a stick at the mouth), and lay the rest of the tape out of the mouth: in
simulation as in the table above; with a real tape, straight out of the mouth along +y to the table's left edge, the rest
hanging straight down off that edge, any end that reaches the floor lying there (see
Simulation simplifications). Deburr the cut conduit end and the box's
knockout edge, as an electrician would before a pull; they are the edges the tape and the gripper pads touch.

## How the two arms are used

The demo feeds with both hands, the way a fish tape is usually fed: the left (back) hand holds the tape on the
conduit's axis 20.5 cm out from the mouth, and the right (front) hand pinches it 5.5 cm from the mouth. Both push
3.5 cm together; then the right hand reopens and goes back while the left hand holds the tape, and the left hand does
the same while the right hand holds it, so the tape is always held. At the far end, the right hand takes the tape in
the box and draws its tip out past the red line.

Both arms are not strictly needed here, and the task does not claim they are. The right arm reaches both the near
mouth and the box, and in simulation one hand alone also fed the tape: with the `onearm_right` variant (seed 0), the right
hand alone made 18 strokes, about 2.7 cm net each, and the camera saw the tape 5.8 cm into the box after 34 s (tip out of
the far mouth, score 0.5 at that point). In that run the right hand's later pinch in the box missed the tape and the arm parked, so it
stayed at 0.5. Two hands are used because that is the usual technique: the tape is never let go, so it cannot spring back or
buckle between strokes.

## Scoring (`score()`, evaluated at the end; `success()` = score 1.0, must hold 1 s)

Score 0 unless both of these hold at the end:
- the tape runs through the whole conduit: some of it lies in the bore within 3 cm of the near mouth and some within
  3 cm of the far mouth;
- both grippers are at least 3 cm from every point of the tape (let go).

Then:

| score | end state |
|---|---|
| 1.0 | the tip lies on the table (centre at most 2 cm above it) past the red line (x below 7.0, y -51.7 to -27.7) |
| 0.5 | the tip is out of the far mouth (its centre more than 5 mm past the mouth, in the box or beyond it), but not past the red line on the table |
| 0.25 | the tip is through the bend but not out of the far mouth |
| 0 | the tip has not passed the bend, or a zero condition above |

Real trial, scored by eye and with the trial video: at the end, the tape goes in at the near mouth and comes out in
the box (one continuous tape), its brass tip lies on the table on the far side of the red tape, and both grippers are
open and off the tape. The 0.25 tier is read from a 1 cm wide band of coloured tape wrapped on the fish tape, its near edge 47 cm from the
tip (part of the setup): 0.25 when that near edge
has gone into the near mouth but the tip is not out of the far mouth; it is a
partial-credit tier only. Its window is narrow: the tip must sit in the last 2 cm of the run (the straight stub into the box, 46.95 to 48.9 cm along the axis) with the tape threaded and both hands off. Partial credit is kept
coarse on purpose: the scored skill is getting the tip all the way out and drawn to the line, and a tape pushed partway
and let go scores 0.

`distance()`: how far the tip still has to go, along the run to the far mouth and then on to the red line (m).
`progress()`: stage diagnostic (tip along the run, through the bend, out, past the line, through the whole run,
released); not the score.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py`, `cabledemo.py` and `vision.py` are in this folder (the cable simulation, the shaped
  start of the tape, the scripted-demo runner and the top-camera helpers the demo uses).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Push the fish tape through the conduit into the far box, then pull its tip past the red line and let go."
```

## Scripted demo

```bash
FT_SEED=0 python cabledemo.py task.py demo_out --max-steps 2400
```

Closed loop on what the robot can observe: the top camera image, the arms' own positions and the fixed layout (the
conduit, its mouth, the box). The script never reads the tape's position from the simulator to choose a motion.
The left arm first moves out of the camera's view and the script traces the yellow tape out of the near mouth in the
top image; the left (back) hand pinches it where the camera sees it 20.5 cm out and holds it on the conduit's axis, and
the right (front) hand pinches it 5.5 cm out. Then the two-hand strokes (see How the two arms are used), each pushing 3.5 cm. After
every stroke the script looks at the box in the top image and stops once it sees the tape 5 cm in from the knockout.
Both hands let go and the left arm moves away. The right hand finds the free end of the tape in the box in the top
image, pinches the tape 1.5 cm back from it with the fingers only a little open, lifts it over the box's back wall, draws
it out the way it comes out of the conduit, past the red line, lays it on the table and lets go.

Recorded demo (`demo/`, seed 0, tip starting 6.4 cm inside the near mouth): 20 two-hand strokes (about 2.5 cm net per
stroke, since the tape compresses and slips a little against the commanded 3.5 cm), the camera sees the tape 5.2 cm into the
box at 63.7 s (step 1,274), and the right hand then draws the tip past the red line and lets go. `success()` first holds at
step 1,466 (73.3 s) and the final score is 1.0. Files: `top_cam.mp4`, `left_cam.mp4`, `right_cam.mp4` (all three cameras),
`setup.png` (the start), `result.json` and `run.log` (the script's log, with the true tip position printed for
checking only). The
last frame shows the end state: in `right_cam.mp4` the brass tip lies on the table beyond the red tape with the tape
running back into the box, and in `top_cam.mp4` both arms are parked away from the tape.

## Shortcuts (scripted variants, `FT_DEMO=<variant>`)

All seed 0, scored by the same `score()`:

| variant | what it does | score |
|---|---|---|
| `nothing` | no motion | 0.0 (tip 6.4 cm in, short of the bend) |
| `partway` | two-hand feed for 6 strokes, then both hands let go | 0.0 (tip about 21 cm in, short of the bend) |
| `onearm_right` | the right hand alone feeds with the same strokes, then tries the draw-out | 0.5 (fed to the box in 18 strokes; its pinch in the box missed and the arm parked, so the tip stayed out of the far mouth but short of the red line; the run was stopped at step 900 with nothing left to move) |
| `hold` | the full two-hand feed, then the right hand grabs at a fixed point near the far mouth and keeps holding | 0.5 (the fixed grab point missed the tape, so the hand ended off it with the tip out in the box; this variant does not exercise the release rule) |

The release rule is visible in the demo itself instead: `demo/run.log` prints the score every 100 steps, and at step
1,200 (tip already out in the box, both hands still on the tape) and at step 1,400 (the right hand drawing the tip
out, still holding the tape) the score is 0.0; it reaches 1.0
only after the hand lets go and backs off (step 1,466).

## Runtime and seeds

- Run limit 2,400 steps (2 minutes at 20 Hz). The demo succeeds at step 1,466 (73.3 s of simulated time). One recorded
run took about 2 h 45 min of wall time on one GPU shared with other simulations.
- The tip's starting depth is drawn per episode from the episode seed (see Reset and sampling). The conduit, the box
  and the red line never move.

## Simulation simplifications

- The fish tape is 70 rigid 2 cm segments with bending springs (bend stiffness 0.03 N m/rad per joint; no twist,
  stretch or plasticity), 4.8 mm round like a 3/16-inch fibreglass fish tape; the brass tip is a rigid 12 mm box. Only
  1.40 m of tape is modelled, lying loose on the table, with a small block at its end where a real fish tape has its reel case.
- Friction is chosen, not measured: 0.25 on the tape and 0.15 in the bore (PhysX combines the two). The simulated
  tape is softer than solid fibreglass (a 3/16-inch fibreglass rod's bending stiffness corresponds to about 0.05 N
  m/rad per 2 cm joint here).
- The conduit's bore is a 12-sided collider inside a round visual tube, and the bend is built from 40 straight pieces.
  The bushing, straps and box are fixed bodies; the box's knockout is a 2.8 cm gap in its wall that the conduit's end
  fills.
- The run is short: 49 cm with one bend, sized to the table. A longer run with more bends needs the same strokes,
  repeated, and more force to overcome the friction of the bends.
- The start lay is written into the tape at reset and settles under gravity and friction for 2 s. The simulated tape
  keeps its start bend by friction; a real fibreglass tape laid in that bend springs back towards straight, so in a
  real trial lay the tape out of the mouth straight along +y to the table's edge and let the rest hang straight down off
  that edge.
