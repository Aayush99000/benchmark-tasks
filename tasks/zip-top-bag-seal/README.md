# zip-top-bag-seal

**Instruction:** "Seal the bag."

A filled 1-quart zip-top freezer bag of chopped vegetables lies on a butcher block, its press-to-seal
track open and standing out past the block's near edge. A press-to-seal track has no slider: it
closes only where it is squeezed between finger and thumb. One hand holds the mouth's corner; the other
pinches the track shut along to the far corner, pressing the rib into its groove the whole way with a
firm pinch (each cell closes only under a real two-pad pinch of at least 10 N per pad); then both let go
and stand clear.

## Why it matters

Sealing a zip-top bag is one of the most familiar kitchen jobs: leftovers, chopped vegetables, lunches
and freezer portions go into press-to-seal bags in most households every day, and a bag left partly
open lets food dry out, spoil or leak in the fridge or freezer. A home-assistance robot that stores food
has to do exactly this. The holding hand is what makes it work: slid along without the corner held, the
pinch drags the bag and slews it round on the board, and the far end is never pressed home.

## Real parts

| object | real part | size / mass | pose (m) |
|---|---|---|---|
| bag | 1-quart zip-top freezer bag, 0.1 mm (about 2.7 mil) LDPE films | 177 x 188 mm outside (6 15/16 x 7 3/8 in); 320 g filled | mouth toward the arms, centreline y 0 |
| contents | clean, food-safe silicone vegetable stand-ins (washable, reused for repeatable trials) | about 300 g, 30 mm deep: carrot coins 17 mm across x 5 mm (1.5 g), pepper pieces about 18 x 10 x 5 mm and celery pieces 20 x 8 x 6 mm (about 1 g); six top-layer coins lie loose | in the bag |
| track | press-to-seal colour-change track 10 mm in from the mouth: a rib profile on the top film over a groove profile on the bottom film | each profile 4 mm wide, 1.6 mm thick; pressed home 2.6 mm; full bag width (177 mm) | x 0.205, 30 mm out past the block; as left, the rib lies 3 mm above the groove |
| butcher block | 12 x 18 in end-grain maple block | 305 x 457 x 45 mm | centre (0.3875, 0.02), near edge x 0.235 |
| vegetable ends | silicone stand-ins: carrot top, pepper stem, pepper core, celery end | cone 24 mm x 30 mm; 8 x 12 mm; 30 x 20 x 12 mm; 24 x 12 x 10 mm | around (0.48, 0.175) on the block |
| prep bowl | 1.5-qt stainless steel bowl of more stand-in pieces | 200 mm across the rim, 85 mm deep | (0.645, 0.20) |
| bag carton | printed paperboard carton of quart freezer bags | 190 x 75 x 95 mm | (0.66, -0.20) |

No knife or other sharp tool is in the scene. The track profile is typical of press-to-seal freezer
bags (rib-and-groove profiles about 4 mm wide on each film); the 2.6 mm pressed-home thickness is the two
1.6 mm profiles interlocked.

## Scoring

The track is scored in 16 cells of 11 mm; a closed cell stays closed (a latched milestone).

- **0.25** at least 4 cells closed.
- **0.5** at least 12 cells closed.
- **0.75** all 16 closed, however the bag was moved on the block meanwhile.
- **1.0** also every finger pad at least 80 mm from the track, with the bag at rest (under 0.02 m/s and
  0.2 rad/s), held for 1 s.

A spill (any loose carrot coin out of the bag) or the bag falling off the block, at any time, including
after the track is closed, scores 0. Every quantity is measurable in a real trial: the green (closed)
length of a colour-change track, whether any piece has come out of the bag, and where the hands end up.
In a real trial, a closed length is checked by eye on a colour-change track (or by feel, the rib seated
in its groove) and measured with a ruler; the score's cells are 11 mm lengths of it.

## Simulation notes

- **Bag:** one rigid body: the packed load, the film's edges, the mouth's films and the track. Film
  does not crease, stretch or puff. The renderer draws no see-through film, so the clear film is drawn by
  its edges (rolls round the load, seams) and the light catching it in bands, and the load shows as it is.
  The top film over the load collides (its sheen bands 16 mm apart, its side and bottom walls), and six
  carrot coins lie loose under it: those are what a spill would lose.
- **Closing (from PhysX contacts):** a rigid contact view on the bag, filtered to the four finger-pad
  links (each hand's `tip_left` and `tip_right`), gives each pad's real contact force on the bag and the
  points where it touches. A track cell is pressed where one hand's two pads both press on the track
  (their contact points within 4.5 mm of the track's centreline and at its height), each with at least
  10 N, their forces opposed (pinching it from above and below), over that cell's length. A cell pinched
  that hard for 0.15 s (3 control steps) snaps shut, stays shut and reads green. A hand passing beside
  the track, touching it lightly or pushing it from one side closes nothing: there is no position or
  jaw-gap trigger. 10 N per pad is a firm finger-and-thumb pinch of the kind that snaps a freezer-bag
  rib home; the YAM gripper squeezes the track at about 17 N per pad in the demo (its fingers commanded
  fully closed and stopping on the profile), well inside its roughly 50 N capability. The rib's 3 mm
  travel into its groove is not simulated (the pads pinch the rib and groove profiles as they stand).
- **Track colliders:** the rib and the groove profiles collide as overlapping 13 mm segments (no seams
  for a pad to catch on), each 0.1 mm inside the drawn profile; the lower pad presses the groove, the
  upper pad the rib, 3 mm above it as left open.
- **Drag:** the pads grip the profiles (LDPE, friction 0.15 against the pads' rubber), so a pinch pulled
  along the track drags the bag by friction; the bag slides on the block (friction 0.4). Holding the
  corner keeps it still physically, not by a rule; even held, a pinch dragged along at sealing force
  pulled the corner out of the holding hand in development, so the demo pinches step by step.

## Demo

`task.py` contains a scripted waypoint demo. The left hand comes in along the track from the left
side, pitched 25 degrees down, and pinches the mouth's left corner hard (about 22-25 N per pad),
holding it. The right hand comes in across the mouth from in front, pitched 25 degrees down, and
presses the track shut a little at a time toward the right corner: 20 pinches 7.3 mm apart, each
pad pressing the track at about 15 N (fingers commanded closed, stopping on the 6.2 mm rib-over-groove
stack) for 0.8 s, easing off to 10 mm between pinches to step along. (A pinch dragged along the track at
sealing force pulled the held corner out of the other hand in development: the YAM arms are compliant,
and the bag slid 110 mm.) It presses the far corner home and draws back. The left lets go of its
corner; the right comes back and pinches home the track the left hand was holding, and both stand
clear.

## Reproduction

`prepare` exports the runtime into `runtime/`. From this task directory, with the
`yam-inspect-isaac:6.1.0` image (or `python3 runtime/build_image.py --image yam-inspect-isaac:6.1.0`):

```bash
# outputs go OUTSIDE the task folder: the runner snapshots the task folder into each run's
# task-source, so an output directory inside it would be copied into itself
OUT="$(cd .. && pwd)/zip-top-bag-seal-outputs"
mkdir -p /tmp/rc-task-home/tmp "$OUT"
docker run --rm --gpus device=0 --network host --shm-size 8g \
  --user "$(id -u):$(id -g)" -v "$PWD":/task:ro -v "$OUT":/outputs:rw \
  -v /tmp/rc-task-home:/tmp/task-home:rw -e HOME=/tmp/task-home -e TMPDIR=/tmp/task-home/tmp \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e OMNI_KIT_ACCEPT_EULA=YES -e ACCEPT_EULA=Y \
  --workdir /outputs --entrypoint /isaac-sim/python.sh yam-inspect-isaac:6.1.0 \
  /task/run.py --policy scripted --episodes 1 --livestream 0 --viz none
python runtime/rescore.py --task task.py "$OUT"/zip-top-bag-seal/RUN/episode-01/trajectory.jsonl \
  --expect "$OUT"/zip-top-bag-seal/RUN/episode-01/result.json
```

Seed 0; fixed starting poses with no randomisation. The scene settles for 30 ticks after reset before
control starts. Control is 20 Hz and physics 240 Hz; the run limit is 1,600 control steps (80 s). The
scripted demo takes about 31 simulated seconds (roughly 2-10 minutes of wall time depending on the
GPU). Each episode writes four 20 fps camera videos (overview, top and both wrist cameras), setup/final
PNGs and periodic snapshots, the measured state trajectory (trajectory.jsonl) and result.json. The
scorer reads only measured poses and velocities of the bag and its loose coins, the robot's joints and
finger travel (finger pads placed by forward kinematics at the rig's arm bases, station.BASES), and the
task's own measured quantity (which track cells are closed); `rescore.py` replays it over the
trajectory and checks it against result.json.

## Files

`task.py` (scene, the track's contact-force closing, scripted demo), `scoring.py` (stateful scorer), `bag.py`
(geometry and measurements of the bag, its loose coins, the block and set dressing, and the finger pads), `simkit.py` (compound rigid parts,
joints and the spline demo controller). No third-party assets. No hardware validation has been
performed.
