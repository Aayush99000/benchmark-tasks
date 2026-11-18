# Dispense one serving of cereal into a bowl

Created 11_16_26 (America/New_York).

A twist-knob cereal dispenser, the kind on the wall at hotel breakfast buffets and in college dining halls, is
screwed to an upright board on the counter. An empty bowl stands on the counter beside its spout. The robot has to
get one labelled serving of toasted oat rings (39 g, 1 1/2 cups) into the bowl without spilling any.

What makes it hard is physical, not a choice:

- **Metering by turning, with no stops.** The knob drives a six-vane paddle wheel; each sixth of a turn empties one
  compartment. A serving is about 1.3 turns in this model, and the knob has no detents or end stop, so the amount
  is set only by how far the robot turns it. Too little scores half; too much (over 47 g) also scores half.
- **Regripping a knob through more than a turn.** A YAM wrist rolls only 240 degrees, so the turn has to be made in
  strokes: grip the knob's bar, roll, let go, back off, roll back, take it again, without bumping the wheel in
  between.
- **Getting the bowl under an overhang.** The canister and wheel housing hang over the spot where the bowl must
  stand, 16 mm above its rim, so a top-down grip on the rim cannot set the bowl there: the hand has to slide it in
  from the side (or hold it up under the spout), and a bowl left short of the spout catches only part of the stream.

How it differs from the nearest tasks: the board's cereal-dustpan task (#2281, closed as a duplicate of dexhand's
Sweep Dustpan) gathers spilled cereal; the paper-cup task (#2127) pulls a cup from a gasket; the soda-fountain task
(#1887, closed as overlapping BEHAVIOR's cream soda) holds a cup on a sprung lever until a level; the coffee-grinder
task (#2216, closed as overlapping dexhand's Grinder Crank) turns a free crank. RoboCasa's `cereal_and_bowl` pours
from a box, and Embodied First Exam's kinder-scoop-pour scoops and pours. Here the quantity is metered by a real
paddle-wheel mechanism and is earned only by the wheel's rotation, the knob must be regripped through more than a
turn, and the bowl must be brought under an overhang it cannot be lowered into.

## Files

| File | What |
|---|---|
| `task.py` | layout, scoring criteria, the scripted demo and its shortcut variants |
| `task.json` | the submission card |
| `cereal_dispenser/geometry.py` | dimensions and masses (NumPy only) |
| `cereal_dispenser/build.py` | USD builders: the dispenser, its wheel articulation, the cereal, the bowl |
| `cereal_dispenser/embodiment.py` | `yam_cereal`: the stock `yam_arena` embodiment plus the dispenser, the bowl and the batched cereal readout |
| `cereal_dispenser/plan.py` | open-loop joint plans for the demo |
| `test_score.py` | 26 unit tests: every band, each shortcut, the geometry of the criteria, the demo plans |
| `dev/clearance.py`, `dev/arm_clearance.py` | offline gaps: wheel to chamber, and both arms to the dispenser along the demo |

## Run and score a trial

Install the package into the simulator's environment once (from this folder):

    uv pip install -e . --no-deps --python ~/IsaacLab-Arena/.venv/bin/python

Then run the scripted demo (or replace `--policy yam_scripted` by a model's policy):

    inspect-robots run --embodiment yam_cereal -E task=task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 --policy yam_scripted --no-prompt \
      --instruction "Get one serving of cereal from the dispenser into the bowl, that is 1 1/2 cups or 39 grams, without spilling any on the counter." \
      --scorer task_score --max-steps 2400 --seed 0 --store-frames --log-dir ~/yam-eval/cereal

`task_score` is printed at the end and kept in the log. Shortcut variants of the demo: `CEREAL_DEMO=none` (do
nothing), `nobowl` (turn the knob without moving the bowl), `under` (200 degrees), `over` (760 degrees).
`CEREAL_DEBUG=1` prints the wheel angle, cereal counts and score every half second.

- **Runtime per trial:** about 2 minutes on an L40S (about 50 s of it Isaac Sim's start); the demo ends by success
  after 637 control steps (32 s). The 760 pieces add little: a 17 s motor-driven test took 79 s of wall time.
- **Seeds:** `--seed 0` with `-E object_jitter=0` (fixed layout). The cereal starts as a fixed lattice
  (`geometry.initial_pieces`, NumPy seed 0; tilts seeded 1) and settles for 3 s before the trial starts. Two
  consecutive demo runs gave identical counts.
- **Unit tests:** `python -m pytest test_score.py` (24 run without the simulator packages; 26 with `yam_arena`).

## Scoring

At the end of the trial, from the simulated pieces (real trial: a kitchen scale and a count):

| Score | Condition | Real trial |
|---|---|---|
| 1.0 | 31 to 47 g in the bowl (the 39 g serving within 20 %), at most 5 pieces outside both bowl and dispenser, bowl upright (tilt under 20 degrees), both hands at least 5 cm outside the bowl's rim, bowl and contents at rest for 1 s | weigh the bowl's contents (tare the empty bowl); count pieces on the counter |
| 0.75 | 31 to 47 g as above, but 6 to 15 pieces spilled, or a hand still at the bowl, or not yet at rest | count; photo |
| 0.5 | 20 g or more in the bowl but short of 31 g or over 47 g, bowl upright, at most 15 spilled | weigh |
| 0.25 | at least one piece in the bowl but under 20 g; or any amount with more than 15 spilled or the bowl tipped past 20 degrees | weigh, count, photo |
| 0 | nothing in the bowl | look |

A piece is in the bowl when its centre lies inside the bowl (within its inner wall, above its floor, no higher than
its rim). Mass is the count times 0.103 g. "At rest": the bowl under 2 cm/s and no more than 2 pieces in it faster
than 5 cm/s (single pieces jitter at up to 2 cm/s in the solver). Doing nothing scores 0; turning the knob without
the bowl under the spout puts everything on the counter and scores 0. The canister is lidded and the dispenser is
fixed, so cereal reaches the bowl only through the wheel.

## Measured in simulation (L40S GPU)

| Run | In bowl | Spilled | Score |
|---|---|---|---|
| demo | 396 pieces, 40.8 g | 0 | 1.0 |
| demo again | 396 pieces, 40.8 g | 0 | 1.0 |
| do nothing | 0 | 0 | 0 |
| knob turned, bowl not moved | 0 | 392 | 0 |
| 200 degrees | 115 pieces, 11.8 g | 0 | 0.25 |
| 760 degrees | 630 pieces, 64.9 g | 0 | 0.5 |

Motor-driven wheel at 60 deg/s with the bowl under the spout (development run): about 6 g per second, steady,
with no piece lost through the wheel at rest. With the robot turning the knob in strokes, the demo's 485 degrees
delivered 40.8 g (about 30 g per 360 degrees; the listing says about 1 oz per twist).

## Simulation model

- **Dispenser** (static colliders): mounting board, back plate and bracket; the wheel chamber as 36 convex pieces of
  a 41 mm bore, open over 80 degrees at the top (inlet) and 60 degrees at the bottom (outlet), closed by end plates;
  a funnel of four sloped plates from the inlet up to a clear 112 mm canister, lidded; a 41 x 71 mm spout whose
  mouth is 80 mm above the counter.
- **Paddle wheel** (articulation): a fixed anchor, the wheel (hub, six rigid inner vanes, shaft, a 56 mm knob with a
  50 x 12 x 18 mm grip bar) on a revolute joint about the knob's axis, and six soft vane tips (the outer 12.5 mm of each
  vane), each on its own spring hinge (0.05 N m/rad, plus or minus 50 degrees), so a piece caught at the inlet's edge
  bends a tip instead of jamming the wheel. The seals' drag is a velocity drive toward standstill whose torque
  saturates at 0.10 N m, which acts as Coulomb friction: the wheel holds still under the cereal's weight and needs a
  light twist (about 4 N at the bar's end) to turn.
- **Cereal**: 760 rigid pieces, 11.5 mm across and 5 mm thick, 0.103 g each (78 g in all); collider the convex
  hull of the ring (a rounded disc), visual a torus. They start as a loose lattice in the funnel and canister and
  settle before the trial.
- **Bowl**: a rigid shell of 192 convex wall pieces plus floor and foot ring, 165 mm across, 63.5 mm tall, 2.2 mm
  wall, 80 g.

| Real measurement | Simulated stand-in |
|---|---|
| contents weighed on a kitchen scale | count of piece centres inside the bowl times 0.103 g |
| pieces on the counter counted | pieces outside both bowl and dispenser |
| bowl upright, hands away, nothing moving (photo) | bowl tilt, fingertip midpoints from the rim, velocities |
| knob turned by hand | wheel joint driven only through contact with the fingers |

**Not simulated, or simplified:** the wheel's inner proportions are not published and were chosen to fit the
listed outer size; pieces are convex discs (no hole, so no interlocking) of uniform size and mass and never break;
the vane tips are rigid blades on spring hinges rather than continuous rubber; the seal friction torque is assumed,
not measured; the mounting board stands in for a wall.

## Real-world setup and safety

| Item | Product | Price | Source |
|---|---|---|---|
| Dispenser | Zevro / Honey-Can-Do SmartSpace wall-mount dispenser, KCH-06139 (three 13 oz canisters of the same knob and paddle-wheel design; use one, leave the others empty). The single-canister KCH-06138 (4.8 x 5.8 x 13 in) is the modelled unit where it is still listed | $63.74 | https://build.com/honey-can-do-kch-06139/s1226783 ; https://www.homedepot.com/p/Honey-Can-Do-SmartSpace-Edition-Wall-Mounted-Triple-Cereal-Dispenser-KCH-06139/206131178 ; single: https://www.gatewayp.com/zevro-kch-06138-smartspace-edition-wall-mount-dispenser-single-13-oz-canister-4-8-width-x-5-8-depth-x-13-height-charcoal--1 |
| Mounting board | 3/4 in x 2 ft x 2 ft plywood project panel, cut to 24 x 50 cm, stood on edge | $14.68 | https://www.homedepot.com/p/Columbia-Forest-Products-3-4-in-x-2-ft-x-2-ft-PureBond-Poplar-Plywood-Project-Panel-Free-Custom-Cut-Available-2922/204853871 |
| Board feet | two 6 in corner braces, screwed to the board's foot | 2 x $3.27 | https://homedepot.com/p/Everbilt-6-in-Zinc-Flat-Corner-Brace-25205/327599232 |
| Board clamps | two 6 in one-handed bar clamps holding the braces to the bench | 2 x $19.99 | https://www.familyhardware.com/product/irwin-quick-grip-6-in-mini-light-duty-one-hand-bar-clamp/345687 |
| Bowl | IKEA VITABBORRE polypropylene bowl, 6 1/2 in across, 2 1/2 in tall | $0.79 (minimum 4 online) | https://www.ikea.com/us/en/p/vitabborre-bowl-green-30637711/ |
| Cereal | Original Cheerios, 8.9 oz box (labelled serving 1 1/2 cups, 39 g) | $6.01 | https://www.walmart.com/ip/10311408 (price from https://www.walmart.com/c/kp/cheerios-box-sizes) |
| Scale | Etekcity 0.1 g kitchen scale, 5 kg | $13.99 | https://www.walmart.com/c/brand/etekcity-kitchen-tools-gadgets |

Total about $150 once, plus cereal. **Consumables per trial:** about 40 g of cereal leaves the canister; pieces that
reach the bowl or counter are discarded, so an 8.9 oz (252 g) box covers about 4 trials after the first 78 g fill.

**Set-up:** screw the dispenser to the board (its own screws) with the spout mouth 80 mm above the bench; stand the
board on edge on the bench, braces clamped, 0.12 m behind the spout as in `task.py`; knob facing the right arm. Bowl
upright on the bench with its centre 100 mm to the dispenser's right of the spout's centre (seen from the front),
22 mm in front of the wheel's axis.

**Reset:** discard every piece outside the canister; top the canister up to 78 g (weigh on the scale), lid on; do not
turn the knob after filling. Empty bowl back at its mark.

**Cereal acceptance test (per box):** 100 whole pieces weigh 9.5 to 11.5 g; 10 pieces laid edge to edge measure 11
to 12 cm on a ruler; discard broken pieces and boxes open longer than a week (stale rings soften and pack
differently).

**Safety:** no power, heat, blades or pressurised parts; the knob needs a few newtons; the dispenser hangs on a
clamped board that cannot tip. Spilled cereal on the floor is a slip risk: sweep after each trial.

## Open points

- The wheel's internal proportions are estimated, so the real yield per turn should be measured once (turn the knob
  one full turn into the bowl and weigh) and the serving band kept at 39 g within 20 %.
- The dispenser's yield per turn depends on fill; the trial always starts with 78 g in the canister.
