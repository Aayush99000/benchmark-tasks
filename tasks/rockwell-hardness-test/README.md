# Take a Rockwell C reading on a manual tester

A manual dial Rockwell hardness tester stands on the table with a steel test block on its anvil. The robot takes one
HRC reading the way the tester's manual says: turn the handwheel so the block creeps up onto the diamond indenter until
the long pointer, after three turns, stops at C (the 98 N preliminary load), with at most 5 divisions (0.01 mm) of
error and no overshoot; push the main-load handle back, hold the full load, pull it forward; then lower the block.

Instruction: `Take one Rockwell C reading on the test block lying on the tester's anvil: turn the handwheel to raise the block onto the indenter until the long pointer has made three turns and stands at C, push the load handle back and hold the full load 5 seconds, pull the handle forward, then lower the block.`

## Why it matters
Rockwell testing is the most common hardness test in metals, heat-treatment and incoming-inspection labs and in
materials teaching labs, and the manual dial tester (the HR-150A layout) is its everyday form. The hard step is the
approach: the block rises invisibly onto the indenter and the operator creeps the handwheel while watching the dial,
stopping within 0.01 mm of travel. Overshooting can't be undone by turning back: the manual says to change the spot
and start again. Then two hands do different things in turn: one sets the load by the handwheel, the other works the
load handle and holds it through the dwell without the handwheel being touched.

Nearest accepted tasks turn a control to a reading (a microscope's focus, an IV roller clamp, a pipette's volume):
there the reading can be approached from either side. Here it can be approached from one side only, through a screw,
with a rigid contact that can't be backed off, and the reading is then taken under a timed load.

## The tester and block
Sizes from the Insize ISH-R150 (same class and layout as the HR-150A) where published; the rest are estimates to be
measured on the bought tester.

- **Body:** 160 x 520 x 700 mm, 60 kg (retail listings: 6.30 x 20.47 x 27.56 in, 132 lb); throat 165 mm (6.5 in work
  surface depth); 170 mm maximum specimen height.
- **Anvil and screw:** 60 mm flat anvil, top 20 cm above the table; the elevating screw raises it 4 mm per handwheel
  turn `[estimate]`.
- **Handwheel:** three spokes, each ending in an upright free-spinning knob (22 mm across, 36 mm tall) 8 cm from the
  centre, 15 cm above the table `[estimate: fit a standard three-spoke handwheel with revolving handles if the
  tester's own wheel has none]`. Clockwise from above raises the anvil, as the manual says.
- **Indenter and dial:** 120 degree diamond cone; the block starts 1.2 mm below its point. Dial 9 cm across, facing the
  robot: the long pointer turns once per 0.2 mm (100 divisions of 0.002 mm), C at the top; the short pointer goes from
  a black dot to a red dot over three turns.
- **Main-load handle:** on the right side, pivot 30 cm up, 10 cm rod with a 24 mm knob; rests 20 degrees forward,
  travels to 40 degrees back. Pushed back past 30 degrees it releases the 1471 N total load, applied over 5 s (the
  manual's 4-6 s dashpot time).
- **Block:** a standard HRC test block, 64 mm across, 10 mm thick, certified value 62 HRC `[estimate: the block's
  certificate]`, flat on the anvil with the indenter 6 mm from its centre.

## Scoring
Read every control step from the handwheel's and handle's angles, the block's pose and the grippers' positions
(`hardness_scoring.py`). The indenter's travel is the block's rise past first contact, from the handwheel angle
through the screw.

| Score | When |
|---|---|
| 0 | no test reached the preliminary load at a fresh spot; or it overshot past 5 divisions; or the handle was pushed without the preliminary load set; or the handwheel was turned under load |
| 0.25 | preliminary load set at a fresh spot (at least 3 mm from earlier spots and the edge, block flat on the anvil): long pointer within 5 divisions of C after three turns, never more than 5 past |
| 0.5 | then the handle pushed back with the load still set, fully applied (5 s), handwheel untouched |
| 0.75 | then held 3-7 s at full load and the handle pulled forward: a valid reading |
| 1.0 | then the block lowered at least 0.5 mm clear, still flat on the anvil, handle forward, both grippers 4 cm from every knob; held 1 s |

`score()` never gives more than 0.75; 1.0 comes only from `success()` holding 1 s. A spoiled test leaves its spot
used; a later test at a fresh spot can still score full, as the manual allows.

**Real trial:** the dial on video (long pointer at C within 5 divisions after three turns, never past; the reading after
the load), the handle's position and timing on video, and the new indent's distance from old ones with a loupe and
scale. The block's certified value checks the reading.

## Run and score
```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=rockwell_hardness_test.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take one Rockwell C reading on the test block lying on the tester's anvil: turn the handwheel to raise the block onto the indenter until the long pointer has made three turns and stands at C, push the load handle back and hold the full load 5 seconds, pull the handle forward, then lower the block." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames
```
Swap `--policy dream` for the Dream-1 run. Set `ROCKWELL_DEBUG=<file>` to log the handwheel, the travel in divisions
from C, the handle and the current test every step. Probes: `ROCKWELL_DEMO_OVERSHOOT_DIV=8` raises the block 8
divisions past C before the load (expected 0); `ROCKWELL_DEMO_HOLD_S=5.0` leaves the handle back too briefly for a
dwell (expected 0.5). The demo aims the handwheel 1.6 degrees past its set point because the left hand stops about
that much short in the official sim; `ROCKWELL_DEMO_TRIM_DEG` overrides it (0.18 degrees per division).

The left hand raises the block; the right hand works the handle and then lowers the block. Lowering with the left hand
failed in the official sim: pulling the knob back from the front-left, the knob slid out of the open end of its
fingers. The right hand's lowering strokes are the mirror image of the left hand's raising strokes.

## Results (official simulator, 2026-11-22)
One fixed layout (`object_jitter=0`), seed 0, 20 Hz, 320 x 176 cameras, on the shared L40S.

| Run | Score | What happened |
|---|---|---|
| Demo (submitted file) | **1.0** at step 1,413 (70.7 s) | preliminary load set 1.1 divisions short of C; full load from 38.85 s, held 5.0 s; block lowered to 1.3 degrees of handwheel (1.19 mm below the start of contact) |
| Overshoot probe (8 divisions) | **0** | the block passed C (0.25 for a moment) and stopped 7.1 divisions past it: the test is spoiled, though the rest of the procedure ran |
| Short-hold probe (5.0 s) | **0.5** | load fully applied, held 0.1 s |
| Dream-1 (dreamlabs/dream-1-beta) | **0** | ran the full 2,400 steps (120 s); in the top camera both hands hover round the front of the tester near the anvil and the dial; no preliminary load was set (the handwheel's angle isn't in the run log) |

Earlier runs, kept for the record: without the trim the hand stopped 8.8 divisions short and the handle was pushed
without the preliminary load (0); with the trim but the left hand lowering, 0.75 (the knob slipped out after 17
degrees). The probes ran on the file before the hold was lengthened from 8.65 s to 9.9 s; neither depends on it.

The dial is about 20 pixels across in the top camera: its pointers show, but not to the division. Stopping within 5
divisions is judged from state, not from what a policy can read in the images.

## Offline checks (no simulator)
```bash
PYTHONPATH=<yam-isaac-lab-arena checkout>:. python -m pytest -q test_hardness_scoring.py test_rockwell_hardness_test.py
```
Needs pytest and NumPy. The scorer's cases (the 5-division window, overshoot, spot spacing, dwell band, wheel turned
under load, stage order) are unit-tested. The scripted demo's plan solves to under 2 mm and 2 degrees of IK error on
both arms, and replayed through the scorer (without the settle trim) with the handwheel following whichever hand
holds a knob and the handle the right hand's, it scores 1.0 with the load set within one division of C; the two
probes score 0 and 0.5.

## Third-party code
`mechanism.py` comes from robocurve/benchmark-tasks pull request #223 ("Open a panel held shut by a spring plunger"),
MIT License (its header has the notice and lists our changes).
