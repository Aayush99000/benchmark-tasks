# Open a mini blind's slats by twirling its hanging tilt wand

Created 11_17_26 (America/New_York). Revised 11_17_26 afternoon: the simulated blind is now a priced retail product in the smallest size it is sold in.

**Instruction:** "Open the blind: turn the wand until the slats are level."

A full-size 1 in aluminium mini blind (American Blinds Trademark, made to measure in its smallest size: 18 in wide,
12 in drop, all twelve slats) hangs from a table-top stand, facing
the robot, slats closed. Its clear hexagonal tilt wand hangs on a hook from the tilter and swings when touched. The
robot has to turn the wand about its own axis, nearly two full turns, through the tilter's worm gear until the slats are
level, then let go and leave the wand hanging.

The skill is turning a thin dangling rod about its own axis with a parallel gripper. One wrist turn turns the wand a
fraction of a turn, so the hand has to pinch, turn, let go, wait for the freed wand to hang still and pinch again, many
times, and stop when the slats (not the hand) say so.

## Run and score a trial

```bash
uv pip install -e . --no-deps --python "$(which python)"     # once: registers the yam_mini_blind embodiment
inspect-robots run --embodiment yam_mini_blind -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy yam_scripted --no-prompt \
  --instruction "Open the blind: turn the wand until the slats are level." \
  --scorer task_score --max-steps 3000 --seed 0 --store-frames --log-dir ~/yam-eval/blind-demo
python -m pytest test_score.py        # scorer and tilter unit tests, no simulator
```

Swap `--policy yam_scripted` for `--policy dream` (or an agent) to evaluate a model. The run prints `task_score`.
`BLIND_DEMO=<variant>` switches the scripted policy to one of the variants below; `BLIND_DEBUG=1` prints one line of
readings per control step.

- Runtime: the scripted demo is 112 s of simulated time; one trial took about 5 to 9 minutes on a cloud L40S (alone
  or with a second trial beside it) and about 7 minutes on the local aarch64 host, start-up included. A full 3,000 step trial is 150 s of simulated time.
- Seeds: the layout is fixed (`object_jitter=0`); `--seed 0` was used throughout and the scripted demo is open loop,
  so the seed changes nothing.
- Run limit: 3,000 control steps (150 s at 20 Hz). The scripted demo uses about 2,230.

| File | What |
|---|---|
| `task.py` | layout, criteria, `score()`, the scripted demo and its variants |
| `mini_blind/mech.py` | the blind, stand, wand and tilter: sizes, sources, USD builder, worm and clutch relation |
| `mini_blind/embodiment.py` | the stock embodiment plus the blind; feeds the tilter and the scorer |
| `mini_blind/grip.py` | lets a scripted waypoint carry a level (not straight-down) gripper orientation |
| `test_score.py` | scorer, tilter and geometry unit tests on synthetic states |
| `dev/` | `run.sh` (one trial), `variants.sh`, `sheet.py` (contact sheet of camera frames), `media.sh` (video and screenshot) |

## How it differs from the nearest tasks

- **Set a window blind's height and louver angle (pull request 2225, closed):** that task was a three-slat mock-up with
  a lift handle and an orange press lever, closed for its simplification. This one is a retail blind at its true size with every
  slat, tilted the way real mini blinds are: by twisting the hanging wand through the worm tilter. There is no lever
  to press and no lift step; the whole task is the in-hand turning of a dangling rod.
- **Open a casement window wide on its peg stay (2110, merged):** a rigid stay arm lifted off a peg and set on another
  hole; one grasp, no turning about the tool's own axis, nothing hanging loose.
- **Hose onto a barb and tighten its worm-drive clamp (1892, merged):** also a worm drive turned in repeated wrist
  turns, but through a rigid nut driver seated on a fixed screw head, which holds itself in place between turns.
  Here the turned part is a 10 g rod hanging from a hook: it swings away when released, each re-pinch has to find it
  again and land on its flats, and the hand must not load the hook sideways.
- **Hang a blind headrail in gated brackets (1412, merged):** installs a headrail; no wand, tilter or slats.

## Where the numbers come from

| Quantity | Value | Source |
|---|---|---|
| **The simulated blind** | American Blinds Trademark 1 Inch Mini Blinds, made to measure, ordered 18 in wide x 12 in high (the smallest size sold), wand tilt, cordless lift | https://www.americanblinds.com/p/bella-view-trademark-1-mini-blind/503343 (fetched 11_17_26): "Minimum Width: 18"", "Minimum Height: 12"", "Controls include a clear Lucite wand for tilting (minimum of 24 inches long), and cordless lift". Price: see Items |
| Headrail | 1-1/2 in x 1 in (38 x 25 mm), phosphate-treated steel | product page ("Headrail Measurement: 1 1/2" x 1"", "the headrails and bottom rails are made of phosphate-treated steel") |
| Slats | 6 gauge (0.006 in) aluminium; nominal 1 in (25 mm) wide | product page ("Made from 6-gauge aluminum"); width from the product's name and SWFcontract Customiser: https://www.swfcontract.com/products/aluminum-horizontal-blinds/customiser-mini-blinds ("Slats are nominally 1 in wide") |
| Lift | cordless: the bottom rail is raised by hand, no lift cord hangs from the headrail | product page ("cordless lift") |
| Ladder spacing | **21.5 mm, 14.2 slats per foot (stand-in for this product)** | not on the product page; from a contract 1 in mini blind with the same 1-1/2 x 1 in headrail, Mariak "Estate" cut sheet: https://sweets.construction.com/swts_content_files/53604/763268.pdf ("Contract Ladder: 21.5mm (14.2 Slats/Ft.)"). Count the slats of the unit as below |
| Slat count | 12 slats and a bottom rail | follows from the 12 in height: 305 mm less the 25 mm headrail and the bottom rail, at 21.5 mm |
| Tilt rod, tilter | hexagonal tilt rod; worm and gear in an acetal housing, with a clutch | not on the product page; Mariak cut sheet ("Pinion Rod (Tilt Rod) - Hexagonal", "The tilt mechanism features a clutch that prevents over-rotation", "The worm and gear arrangement is entirely enclosed inside an acetal thermoplastic housing") |
| Wand section | **hexagon, 1/4 in (6.35 mm) across flats (stand-in for this product)**; clear acrylic | the product page says only "clear Lucite wand"; hexagonal 1/4 in is the section on the Mariak cut sheet ("Wand - Clear Hexagonal"), the SWFcontract page ("hexagonal cross section measuring approximately 1/4 in diameter") and the replacement wand below |
| Replacement tilter | wand tilter, $5.99 (1/8 in square-rod pattern), "the gear spins for a maximum of 340 degrees before becoming disengaged"; a 1/4 in hex-rod tilter for a 1 x 1-1/2 in headrail with a clutch and wand hook, $10.29 | https://blindparts.com/?p=4167 (fetched 11_17_26); https://fixmyblinds.com/products/mini-blind-tilter-083 |
| Worm ratio | **10 wand turns per tilt-rod turn (stand-in)** | not published for any blind found. US patent 9,376,859 (https://patents.google.com/patent/US9376859) gives a worm-to-gear ratio "in the range of between approximately 5:1 to approximately 15:1"; 10:1 is the middle of that range |
| Closed angle | **70 degrees from level at either stop (stand-in)** | not published. US patent 4,541,468 describes 180 degrees between the two closed positions for an ideal blind; 1 in slats at 21.5 mm spacing overlap fully from asin(21.5/25) = 59 degrees, and real blinds stop a little past that |
| Turning torque at the wand | **0.015 N m (stand-in)** | not published; small enough for a fingertip twirl. Measured on a unit as below |
| Wand length | **250 mm (the supplied wand sawn short)** | the product's wand is "minimum of 24 inches long" (610 mm), twice the height of a 12 in blind; with the blind's top 350 mm above the table the hook is 298 mm up, so the wand is sawn to 250 mm from its plain lower end and hangs 48 mm clear of the table. See Open weaknesses |
| Wand stand-off, hook, tilter position, bottom rail section, stand | 27 mm in front of the slats' centre line; hook 8 mm below the headrail, 52 mm from its left end; rail 25 x 10 mm | not published; chosen, and set on the built unit with a ruler |

### Measure your own unit, and what to change

The scoring is a measured end state (slat angle, wand plumb, stand position), so a real unit that differs from the
stand-ins is still scored the same way. To make the simulator match a purchased blind:

1. **Wand turns.** Close the slats room-side edge down until the clutch slips. Put a tape flag on the wand. Count the
   turns (to a quarter turn) until the slats are level by the inclinometer, and on until the clutch slips the other
   way. Set `RATIO` in `mini_blind/mech.py` to 360 x (turns closed to level) / (closed angle in degrees).
2. **Closed angle.** With the slats closed, hold a phone inclinometer on a middle slat. Set `CLOSED_DEG`; the task's
   0.25 band (20 degrees from the start) follows it.
3. **Wand.** Calipers (or a ruler) across two opposite flats, and a tape measure from the hook to the lower end. Set
   `WAND["af"]` and `WAND["len"]`.
4. **Turning torque.** Clamp a clothes peg or a 50 mm strip of card square on the wand; pull its end with a 1 N (100 g)
   spring scale, square to it, until the wand turns. Torque = reading x 0.05 m. Set `TILTER_TORQUE_NM`.
5. **Ladder spacing.** Count the slats and measure from the top slat to the bottom rail with the tape measure; set
   `PITCH` and `N_SLATS`.
6. **Wand stand-off and tilter position.** Ruler from the slats' centre line to the wand, and from the headrail's end
   to the hook. Set `WAND_STANDOFF` and `TILTER_END`.

## Simulation model

- One articulation with a free root: the stand (base, two posts, head piece) with the headrail, 1.38 kg, standing loose
  on the table (friction 0.5). Nothing is clamped: a hand that drags on the wand drags the stand.
- The tilter stem turns about the vertical without limit against Coulomb joint friction (0.015 N m static, 80% of it
  moving). The wand, a rigid hexagonal prism (convex hull collider, 10.3 g, friction 0.6), hangs from the stem on a
  universal joint (two revolute axes, +-70 degrees, lightly damped), so it swings freely and carries twist to the stem.
- Worm, gear, clutch, tilt rod, drums and ladder cords are one kinematic relation (`mech.Tilter`), fed every control
  step with the stem's measured angle: the tilt rod turns one tenth of the stem's turning and stops at +-70 degrees
  while the stem keeps turning (the clutch). Clockwise seen from above lifts the slats' room-side edge.
- All twelve slats and the bottom rail are separate rigid bodies (25 x 452 mm; 1.5 mm thick colliders standing for the
  crowned section), each on its own revolute joint through its centre line, driven to the tilt rod's angle by a soft
  position drive (0.02 N m/rad). A slat pushed by a finger gives and springs back when let go, as a real slat lifted
  off its ladder rung drops back; nothing a slat does reaches the stem, as a worm does not back-drive. So pushing
  slats cannot open the blind, and pulling the wand down only loads the hook and the stand.
- Not modelled: the ladder cords as string and the cordless lift (the bottom rail cannot be pushed up, slats cannot be
  bunched or lifted, and do not rattle); slat
  flex; the hook's play and its ability to unhook; the tilter stem's forward slope (the stem here is vertical); gear
  backlash.

## The scripted demo

Open loop from the fixed start; it reads no simulator state. Only the left hand works (the wand hangs in front of the
left arm, out of the right arm's reach). It comes in level, fingertips toward the blind, and pinches the wand 138 mm
below its hook, pads flat on two of the wand's flats (commanded gap 3.5 mm on the 6.35 mm wand). It draws the pinched
point 35 mm toward the robot, clear of the slats, as a person does, then swings 60 degrees round the wand's own axis,
which turns the wand one flat. It lets go and backs off; the freed wand swings back and is hanging still about two
seconds later; the hand comes round to where it began and pinches the next pair of flats. Twelve turns, about 110 s.
A turn is one flat because parallel pads close flat on a hexagon: a pinch that starts off the flats turns the wand
to the nearest pair as it closes (an early version with 95 degree turns lost about a third of each turn this way).

## Measured in the simulator

`object_jitter=0`, `--seed 0`, 20 Hz, 320 x 176 cameras (640 x 352 for the media run).

| Run (label) | Host | `BLIND_DEMO` | task_score | Final reading |
|---|---|---|---|---|
| demo18a | cloud L40S | demo | 1.0 | slats +2.0 degrees (1.999 wand turns), wand 0.0 degrees off plumb, stand moved 0.0 mm |
| demo18b | cloud L40S | demo | 1.0 | same |
| media | cloud L40S | demo, 640 x 352 | 1.0 | same (the video and screenshot come from this run) |
| local18 | local aarch64 | demo | 1.0 | slats +1.9 degrees, 1.997 turns, stand moved 0.0 mm |
| v-nothing | cloud L40S | `nothing`: hands stay at the ready pose 60 s | 0.0 | slats -70.0 degrees |
| v-one | cloud L40S | `one`: a single turn | 0.0 | slats -64.3 degrees: one turn is about 6 degrees of slat |
| v-half | cloud L40S | `half`: six turns | 0.25 | slats -34.1 degrees (0.997 turns) |
| v-over | cloud L40S | `over`: sixteen turns, past level | 0.25 | slats +25.7 degrees (2.659 turns): 0.7 degrees outside the 0.5 band |
| v-far_over | cloud L40S | `far_over`: twenty-six turns | 0.0 | slats +70.0 degrees, closed the other way; the wand went on to 4.33 turns with the clutch slipping |
| v-yank | cloud L40S | `yank`: pinch the wand and pull it down 50 mm, twice | 0.0 | slats -70.0 degrees; the pads slid down the wand, stand moved 0.0 mm, wand swung 3.5 degrees and hung still again |
| v-push | cloud L40S | `push`: closed fingertips sweep up under the slats' room-side edges | 0.0 | single slats were pushed as far as +59.8 degrees while touched and all sprang back to -70.0; the stand was shoved 5.1 mm |
| v-hold | cloud L40S | `hold`: the demo, but the hand never lets go | 0.0 | slats +2.0 degrees, hand still on the wand (wand held 14.6 degrees off plumb) |

All twelve runs above are of the 18 x 12 in blind, made on 11_17_26 after the change of size.

One hand or two: the demo is one-handed, and no claim is made that two hands are needed. In this layout the wand
hangs out of the right arm's reach, so a two-handed (hand-over-hand) version was not run.

Earlier runs, all with a 12 in wide blind of the same build (wand 77 mm nearer the centre): demo 1.0 three times on the
cloud host and once locally (slats +1.9 degrees), and the eight variants with the same scores as above. `gate-one` (one 95 degree turn: 0.0, slats -58.5 degrees) showed that the gripper can turn the
hanging wand at all; `demo1` (six 95 degree turns: 0.25, slats -25.5 degrees) showed the loss when a pinch closes
off the flats, which led to whole-flat 60 degree turns. In the final demo's trace the turns are not all equal: they turn the slats between about 4 and 7
degrees each (the pinch slips or settles a little on the flats), and twelve of them end at +1.9 degrees.

Dream-1: 0.0 in one full 3,000-step run.

## Scoring, and how a real trial measures it

All criteria are judged on the final state.

| Criterion | Simulation | Real trial |
|---|---|---|
| Hands clear | each fingertip midpoint at least 4 cm from the wand's axis and from the box the slats and headrail fill, for the last 1 s; otherwise 0 | the robot stopped with its grippers visibly away; the supervisor waits 1 s |
| Slat angle | mean of the twelve slats' measured joint angles from level: within 10 degrees 1.0, within 25 degrees 0.5, within 50 degrees 0.25, else 0; the same either side of level | phone inclinometer (or protractor against a side photo) on a middle slat; the slats move together |
| Wand hanging | hook-to-end line within 10 degrees of plumb, or the 1.0 band becomes 0.5 | side and front photo against a plumb line (a weighted thread), or the inclinometer held beside the wand |
| Stand in place | base within 20 mm of its start, or the 1.0 band becomes 0.5; more than 100 mm scores 0 | masking-tape outline round the base; ruler from the outline |
| Stand upright | within 10 degrees of upright, else 0 | by eye; inclinometer on a post if in doubt |

Every outcome is covered: the slat angle always falls in one band, and the other criteria only gate or cap it. Doing
nothing scores 0 (70 degrees). Turning the wrong way scores 0 (the clutch slips at the stop). Turning on past level
scores by the same bands on the far side, down to 0 at the other closed stop. A hand left on the wand scores 0.
Nothing is remembered from earlier in the trial: a stand nudged and put back, or slats turned too far and turned back,
score by where they end (unit tests cover each case).

## Real-world setup and safety

Stand: a 520 x 180 x 12 mm plywood base; two 20 x 20 mm softwood posts, 319 mm long, screwed upright to the base 32 mm
behind the blind's centre line and 430 mm apart; a 500 x 60 x 19 mm softwood head piece screwed across their tops with
its top 350 mm above the table. The blind's two brackets screw to the underside of the head piece so the headrail
hangs centred on the base. The lift is cordless: leave the bottom rail fully lowered. Saw the wand to 250 mm from its
plain lower end and smooth the cut. Set the stand on the table inside a masking-tape outline with the middle of the
slats 350 mm in front of the midpoint between the arm bases, the blind facing the robot and the tilter on the robot's
left. Nothing is powered, hot, sharp or heavy: the wand needs about 0.3 N at a 50 mm lever, the stand weighs about
1.4 kg and everything is under 0.35 m high. The aluminium slats' edges are thin but rounded, and no cord hangs from a
cordless blind to snag the arms.

## Items for a real trial

| Item | Product | Price | Where |
|---|---|---|---|
| **Blind (the one simulated)** | American Blinds Trademark 1 Inch Mini Blinds, made to measure, 18 in wide x 12 in high, wand tilt, cordless lift, with its brackets and wand | $66.14 shown with width 18 and height 12 in the page address (list $110.24); $68.96 shown with no size (list $114.94); both with a 40% sale code shown as ending 11/18; fetched 11_17_26. The page does not print which size a shown price is for; the configurator gives the exact figure | https://www.americanblinds.com/p/bella-view-trademark-1-mini-blind/503343 |
| Spare tilter | mini blind wand tilter, 1/4 in hex rod, 1 x 1-1/2 in headrail, with clutch and wand hook | $10.29 | https://fixmyblinds.com/products/mini-blind-tilter-083 |
| Spare wand | clear hexagonal wand with tip, 20 in (the blind comes with its own wand, 24 in or longer; either is sawn to 250 mm) | $9.09 | https://fixmyblinds.com/products/clear-plastic-wand-hex-shaped |
| Stand | plywood offcut, 1 m of 20 mm square softwood, 0.35 m of 19 x 60 mm softwood, eight wood screws | about $15 at any hardware shop (not priced from a listing) | local |
| Turning torque | 1 N (100 g) or 10 N spring scale, e.g. Arbor Scientific push-pull spring scale | $10.95 | https://arborsci.com/products/1000g-push-pull-spring-scale |
| Lengths | Stanley 16 ft PowerLock tape measure; Westcott 12 in steel ruler | $11.97; $8.49 | https://www.homedepot.com/p/Stanley-16-ft-PowerLock-Tape-Measure-33-116L/100654911 ; https://www.quill.com/westcott-stainless-steel-rulers-12/cbs/001457.html |
| Outline | ScotchBlue 2090 painter's tape | $10.27 | https://www.walmart.com/ip/ScotchBlue-Original-Multi-Surface-Painters-Tape-94-inch-x-60-yard-2090-1-Roll/471970548 |
| Angles | a smartphone's built-in level | free | already owned |

About $145 in all, reusable; nothing is consumed per trial. The same blind (same product number, sizes and headrail)
is listed at Blinds.com: https://www.blinds.com/p/blindscom-1-mini-blind/503343 ($38.87 shown with no size, $63.18 with
width 18 and height 12 in the address; fetched 11_17_26). All are sale prices and change with promotions.

## Reset

Turn the wand anticlockwise (seen from above) until the clutch slips: the slats are closed, room-side edge down. Turn
the wand until two flats face left and right. Still the wand. Put the stand back in its outline.

## Open weaknesses

- **Stand-in mechanism numbers.** The worm ratio (10:1), the closed angle (70 degrees) and the turning torque
  (0.015 N m) are not published for any blind found; the first is the middle of a patent's range and the other two are
  estimates. They set how many turns the task takes, not how it is scored. A purchased unit is measured as above.
- **The wand is sawn short.** The product's wand is at least 24 in (610 mm) long, longer than a 12 in blind is tall;
  on a window it would hang past the sill. Here the blind's top is 350 mm above the table (the height the arms work
  under), the hook is 298 mm up, and the wand is sawn to 250 mm. A full-length wand needs the blind mounted about
  0.35 m higher. The pinch, turn and re-pinch are the same on a shorter wand; it swings faster (a shorter pendulum).
- **Parts the product page does not publish.** The ladder spacing (21.5 mm), the wand's section (1/4 in hexagon), the
  tilter's build and its place on the headrail (52 mm from the left end) come from contract blinds with the same
  1-1/2 x 1 in headrail and from replacement parts, not from this product's page. Each is measured on the unit as
  above; none of them enters the scoring.
- **Price shown is a sale price.** The blind's price was read from the retailer's page on 11_17_26 with a 40% code
  (list price $110.24 to $114.94); the page does not print the size a shown price belongs to.
- **Cordless lift is not modelled.** On the real blind a hand that pushes the bottom rail up raises the slats; in the
  simulator the bottom rail and slats only turn. The instruction does not ask for it and the real trial's slat-angle
  reading on a middle slat still applies, but a raised blind is an outcome the simulator cannot produce.
- **Ladder cords are a kinematic relation, not string.** Slats cannot be lifted off their rungs, bunched or
  bent; a slat pushed by a finger springs back through a soft drive instead of dropping back under its own weight.
- **The tilter stem is vertical and the hook is an ideal universal joint.** Real stems slope forward and real hooks
  have play and can unhook under a hard pull; a yank here only slides the pads down the wand.
- **Whole-flat turns.** The scripted demo relies on the wand starting with two flats facing left and right (a reset
  step). A policy that pinches off the flats still turns the wand, but loses part of each turn.
- **Slow for a simple act.** A person opens a blind in about three seconds; the scripted demo takes 110 s for twelve
  turns, most of it waiting for the freed wand to hang still.
- **One arm.** The second arm has nothing to do in this layout.
- **Relatability rests on the wand.** Cord-tilt and cordless blinds are also common; the claim is that most adults
  have twisted a blind wand, not that every blind has one.
- **Top camera view.** The top camera sees the blind's face from above at about 45 degrees; the slat angle is plain
  in it at 640 x 352 but subtle at 320 x 176. The wrist camera sees the wand and slats close up.
