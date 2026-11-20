# Put a banknote away in the till, under its bill hold-down, and shut the drawer

An open cash drawer stands on the table in front of two YAM arms. In its till, the middle note compartment holds
a stack of notes under a sprung plastic hold-down. The customer's note lies on top of the drawer case, where a
cashier leaves it until the change has been given. The robot has to lift the hold-down with one hand, lay the
limp note flat on the stack underneath it with the other, let the hold-down down on it, and push the drawer shut.

Results on the final scene and scorer (Isaac Lab-Arena, one trial each): the scripted demonstration scored 1.0 in
968 steps; Dream-1 scored 0.0 over the full 2,400 steps. `results/qualification-summary.json` holds the numbers
copied from both runs' `RESULT.json`, and `tests/test_results.py` checks that the run scored the very files in
this folder (same SHA-256 for the scene, the asset, the model and the scorer).

Folder contents:

| Path | What it is |
| --- | --- |
| `task.json` | the task card |
| `rdb_till_note/till_model.py` | every dimension, mass and spring, each marked published, derived or estimate |
| `rdb_till_note/till.usda` | the scene asset, generated from `till_model.py` by `python -m rdb_till_note.build_scene` |
| `rdb_till_note/fixed_scene.py` | instruction, run limit, names of bodies and joints |
| `rdb_till_note/grading.py` | the scorer (NumPy only) |
| `rdb_till_note/adapter.py` | adds the drawer and the note to the stock Inspect Robots YAM station and feeds the scorer |
| `rdb_till_note/scripted.py` | the scripted demonstration |
| `rdb_till_note/run.py` | runs one trial and writes the result, videos, scorer trace and diagnostics |
| `results/qualification-summary.json` | key numbers from the qualification run's two `RESULT.json` files |
| `tests/`, `tools/` | unit tests, the stand-in world for dry runs, the dry-run script, arm collision hulls for clearance checks |

## The real objects, and where the numbers come from

| Quantity | Value used | Source | Status |
| --- | --- | --- | --- |
| Cash drawer | APG Vasario VB320-BL1616, 16 x 16 in class, black, 16.8 lb, fixed 5 x 5 till, "Durable Plastic Hold Downs", "Latch Ejection Mechanism", roller bearing suspension | retailer page <https://www.barcodefactory.com/apg/cash-drawers/vasario/vb320-bl1616> | read on the page |
| Case outer size | 411 x 414 x 109 mm (16.2 x 16.3 x 4.3 in) | retailer listings for the same part number | seen in search-result summaries only; the page opened gives 16 x 16 in and no height. Measure on the bought drawer |
| Till insert | VPK-15B-2A-BX, "Bill/Coin Count = 5 x 5, Fixed Bill/Coin Areas, Plastic Bill Hold Downs, 14.2 x 12.05 x 2.35 inches, 361 x 307 x 60 mm", "Standard till for 1616 or 1618 cash drawers" | APG Standard Accessories Guide M-43-024 Rev. L, page 3, <https://www.webstaurantstore.com/documents/pdf/brochure/m-43-024-rev.-l-standard-accessories-guide_1_.pdf> | read in the document; its drawing shows coin cups at the front and note compartments with hold-downs behind |
| Hold-down, what it is | a plastic arm hinged on a bracket along the back of the till and pulled down by a coil spring | cover photograph of the same guide (close-up of one hold-down); photographs in APG Performance Accessories Guide M-43-021 Rev. N | photographs viewed; nothing was copied from them |
| Hold-down, size and force | arm 100 x 16 x 4 mm, finger tab 16 x 12 x 16 mm, 1.5 N at the free end when down, about 2 N when lifted 50 degrees | none | **estimate**, to be measured (ruler, spring scale at the tab) |
| Till weight | 1.13 kg (2.5 lb) | a retailer listing | seen in a search-result summary only |
| Note compartments | 72.2 mm pitch (361 / 5), 2 mm walls, so 70.2 mm inside; 172 mm long, 40 mm deep; coin cups 102 mm deep at the front | pitch derived from the published width | wall thickness, length and depth are **estimates** |
| Drawer travel, front panel, slide friction | 310 mm, 20 mm thick, 2 N | none | **estimates** |
| Ejector spring and latch | not modelled | the page above says the drawer has them | their forces are not published |
| US note | 6.14 x 2.61 in (156 x 66.3 mm); paper three-fourths cotton, one-fourth linen | <https://www.uscurrency.gov/denominations/1> | read on the page |
| Note thickness and weight | 0.11 mm (0.0043 in), 1 g | the usual Bureau of Engraving and Printing figures | quoted from memory, not on the page opened |
| Paper stiffness | E = 3 GPa, giving EI = 2.2e-5 N m^2 for the 66.3 mm width | none | **estimate**; a real used note may be two to three times limper |
| Where the note starts | on top of the drawer, over its edge | University of Texas Handbook of Business Procedures, 6.2.B "Counting Change": "Keep money received from the customer on the register above the drawer until the transaction is completed." <https://afm.utexas.edu/hbp/part-6/2-cash-collections> | read on the page. That the note hangs 50 mm over the front edge is our choice of how it was put down; flat on the case with no overhang it could not be picked up by this gripper |
| Who does this | cashiers, O*NET 41-2011.00, first listed task "Receive payment by cash, check, credit cards, vouchers, or automatic debits", 3,106,300 employees (2025); fast food and counter workers, 35-3023.00, first listed task "Accept payment from customers, and make change as necessary", 3,855,900 employees (2025) | <https://www.onetonline.org/link/summary/41-2011.00>, <https://www.onetonline.org/link/summary/35-3023.00> | read on the pages |

## How it is modelled

Everything is a rigid body with box colliders; there is no cloth, no soft body and no software rule.

- **The note** is a chain of eight rigid strips, 19.5 x 66.3 mm, joined by seven hinges across the note's width,
  a floating articulation. Each hinge has the bending stiffness of 19.5 mm of paper. So the note droops, hangs,
  drapes over an edge and lies down along its length as paper does; it cannot bend across its width, twist,
  crease or tear. Two departures from the real note are needed for stable contact and are stated in `task.json`:
  each strip's collider is 2 mm thick (the note is drawn as a 0.4 mm sheet in the middle of it), and the note
  weighs 8 g, not 1 g. The hinge stiffness is raised by the same factor of eight (9.0e-3 N m/rad), so the shape
  the note takes under its own weight is the 1 g note's. The print is a generic green engraving made of
  rectangles, not a copy of a real note.
- **The notes in the till** are one loose rigid block, 156 x 66.3 x 6 mm, 50 g. It can be pushed out of place,
  and the scorer checks that it was not.
- **The drawer unit** is one fixed-base articulation: the case is fixed to the world; the drawer with the till
  moulded onto it slides 310 mm on a prismatic joint with 2 N of friction; each of the five hold-downs is a rigid
  arm with a finger tab on a hinge with a spring (0.06 N m/rad, preloaded to 0.15 N m when down). A hold-down in
  an empty compartment sits on its lower stop just clear of the floor.
- **Not modelled:** the drawer's ejector spring and latch (shut means pushed fully home; a real drawer needs a
  firmer push at the end and then stays shut by its latch), coins, the till's lift-out handles, the case's feet,
  cable and lock mechanism (the lock is drawn only).

## Run and score a trial

Runtime: the RoboCurve YAM runtime with Python 3.12, Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Linux, NVIDIA RTX
GPU), Inspect Robots 1.0.0, inspect-robots-yam 0.38.0 and yam-arena 0.1.0
([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena)). Install this folder into that
environment with `uv pip install --no-deps .`. For the Dream-1 run, put a Dream Labs key where the Dream client
looks for it (`~/.dreamlabs/credentials`); no key is stored here.

```sh
python -m rdb_till_note.run scripted --out out/scripted    # the scripted demonstration; exits 0 only on 1.0
python -m rdb_till_note.run dream --out out/dream          # one Dream-1 trial
TILL_DEMO_VARIANT=no_lift python -m rdb_till_note.run scripted --out out/no-lift   # negative control (see below)
```

Each command runs one trial through Inspect Robots with the scorer `till_note_score`, seed 0, zero object jitter
and a limit of 2,400 actions at 20 Hz (120 simulated seconds). The scene is fixed, so there are no other seeds.
Each run writes `RESULT.json`, `media/setup.png`, `media/top_cam.mp4` (the overhead camera), `media/side_cam.mp4`
(a review-only camera, 1280 x 704, never given to a policy) and `media/grader-trace.jsonl` (the scorer's state at
every step). Policies receive 320 x 176 RGB from the overhead and the two wrist cameras plus joint and gripper
positions, and nothing else.

Runtime per trial, measured on one data-centre GPU: the scripted demonstration took 309 s of wall time for its
968 steps (it ends when the score reaches 1.0); the Dream-1 trial took 814 s for the full 2,400 steps and made
99 requests to the model.

### Results of the qualification run

| Run | Score | Steps | What happened |
| --- | --- | --- | --- |
| Scripted demonstration | 1.0 | 968 steps of 2,400 (48.4 s) | hold-down taken and raised by step 192, note taken at step 318 on the first try, laid in by step 627, hold-down let down on it by step 743, drawer home at step 956 |
| Dream-1 (`dreamlabs/dream-1-beta`) | 0.0 | 2,400 of 2,400 | both grippers went down in front of the open drawer and moved about over its front half for the whole trial, partly closed. Neither touched the note (no gripper came closer than 158 mm to it), the hold-downs or the stack; the note never left the case. The grippers brushed the drawer front now and then (peak 24 N) and pushed it in by 42 mm, leaving it 268 mm open |

What the demonstration measured, against what was expected before the run (`diagnostics` in its `RESULT.json`):

| Where | Expected | Measured |
| --- | --- | --- |
| scene at reset | drawer 310 mm open; hold-downs about -24.5 and (middle) -21 degrees; folds within 3 degrees | 310.0 mm; -24.5 and -21.19 degrees; largest fold 2.7 degrees (the overhanging end droops 3 mm) |
| hold-down tab grasp | opening 15 to 16 mm, first attempt | 15.9 mm, first attempt; 9.9 N on each fingertip |
| hold-down raised | 29 to 33 degrees | 30.1 degrees (it took a command of 45.4 degrees: the loaded arm rides low) |
| note grasp | opening 1.5 to 2.5 mm, end strip within 5 mm of the gripper | 1.87 mm, first attempt; end strip 3.4 mm from the gripper's centre; 4.9 N on each fingertip |
| note centred over the compartment | under 1 mm off, under 1 degree skew | 1.8 mm off before the last correction, 0.4 degrees skew |
| note laid | far end 292 to 296 mm into the drawer, side within 2 mm | 295.3 mm; side within 0.3 mm |
| note let go, hold-down let down | strips about 47 mm high; hold-down about -20 degrees | all eight strips at 47.0 mm (flat on the stack); -19.97 degrees, its free end 0.02 mm into the note's top |
| drawer pushed | under 1.5 mm open | 0.0 mm |
| peaks | tab 5 to 15 N, note 3 to 6 N, drawer 3 to 12 N, stack under 3 N | 9.9 N, 4.9 N, 37 N, 0 N |
| arm to arm contact, steps, score | 0 N, 900 to 1,100, 1.0 | 0 N, 968, 1.0 |

Two things the run showed that the dry run did not: the drawer crept in by 20 mm (310 to 290 mm open) while
the left hand swung the hold-down up and held it, because the slide has only 2 N of friction (the demonstration
reads the drawer's opening, so it followed); and the push on the drawer front peaked at 37 N for an instant when
the drawer reached its stop, against about 1 N while it was rolling.

### What the cameras show

From the rendered frames of the two runs. The overhead camera looks down and forward from behind the arms and
sees the whole table: the black drawer and case, the till's ten compartments with the five grey hold-down arms,
the green-printed notes in the middle compartment and the customer's note on the case, light against the black.
At 320 x 176 the note on the case is a pale rectangle about 23 x 13 pixels and the hold-downs are thin grey bars;
whether the note is under or on a hold-down cannot be told at that size, but where the note is can. At the ready
pose the right wrist camera has the note in view and neither wrist camera has the till in view.

The review camera of the qualification run sat above the robot's left shoulder. It shows the till, the note and
the hold-downs clearly at the start and the end, but the left arm stands between it and the middle compartment
from the moment the hold-down is lifted until it is let down, so it does not show the note going in. The camera
has since been moved (`recording.review_camera_pose`: to the robot's right of the till, 46 cm above the table,
looking down across the middle compartment, 18 mm lens); a check of the arms' collision hulls along the
demonstration's path says the note is in view for most of the time it goes in from there, against about a third
from the old place. The scene, the scorer and every policy input are unchanged by this; the old pose is still
`fixed_scene.side_camera` so that `fixed_scene.py` keeps the hash the run recorded. New review video needs one more
run.

`RESULT.json` holds the score, steps, wall time, the reason the trial ended, file hashes, the scorer's final state
and a `diagnostics` block that is never given to a model policy: the scene as it came up, peaks over the trial,
the state at the end of each phase, the demonstration's own log of grasps, trims and checks, and every 5 steps the
commanded and measured gripper positions, openings, hold-down angles, drawer opening, each strip of the note, its
fold angles, and fingertip contact forces on the tab, the note, the drawer and the stack.
`peak_interarm_contact_n` is recorded for information only and is not part of the score.

## Scoring

The scorer is `rdb_till_note/grading.py`; `task.json` states the same rules in words. It looks only at the end
state: where the note's eight strips are, where the stack is, the angle of the middle hold-down, how far the
drawer is open, where the two grippers are.

| Score | State |
| --- | --- |
| 0 | any part of the note outside the drawer, or a gripper within 30 mm of the note |
| 0.25 | the whole note inside the drawer and let go |
| 0.5 | the whole note inside the middle note compartment, below the rim |
| 0.75 | also flat (no part more than 10 mm above the stack) and under the hold-down, whose free end is directly above the note and within 4 mm of touching it; the stack not pushed more than 5 mm or lifted more than 3 mm |
| 1.0 | also the drawer within 2 mm of fully home, both grippers 30 mm clear of drawer and case, nothing moving for 1 s |

Ways to a score that was not earned, and what they get (`tests/test_grading.py`, `tests/test_demo.py`):

| Strategy | Score |
| --- | --- |
| Shut the drawer and leave the note on the case | 0 |
| Drop the note anywhere into the drawer and shut it | 0.25 |
| Drop the note into an empty compartment (no stack to get under) | 0.25 |
| Lay the note on top of the lowered hold-down, or on the stack beside a hold-down that is still up | 0.5 |
| Push the note in standing or folded | 0.5 |
| Drag the stack out from under the hold-down to make room | 0.5 |
| Do everything but leave the drawer open, or leave a gripper leaning on the drawer | 0.75 |

### Scoring a real trial

At the end of the trial note where the grippers are (video), check that the drawer front is home, then pull the
drawer open and photograph the till from above and from the side. The marked note (a pencil dot in one corner)
is either under the hold-down on the stack, flat, or it is not; "within 4 mm" and "10 mm above the stack" are
read from the side photograph against the 6 mm stack. Whether the stack moved is read against the compartment's
front wall (it starts 8 mm from it). On a real drawer "shut" is the latch having caught.

## Real-world build list

- APG Vasario VB320-BL1616 cash drawer with its standard till (about 145 US dollars), clamped to the table, cable
  not connected, opened with its key to the manual-open position and pulled out to its stop.
- 51 used 1 dollar notes; one marked with a pencil dot.
- Tape marks on the case top for the note's start place.
- A spring scale to measure the hold-down's lift force and the drawer's closing force once, for the record.

## The scripted demonstration

`rdb_till_note/scripted.py`. It reads joint and gripper readings and simulator state (the note's strips, the
hold-downs' angles, the drawer's opening), which the review rules allow for a scripted demonstration; it changes
nothing in the simulation. Phases:

1. `reach hold-down`, `lift hold-down`: the left gripper, tilted 60 degrees below level with its pads side by
   side, comes down on the middle hold-down's finger tab (16 mm across, pads opened to 24 mm inside the 70 mm
   compartment), closes to 2 mm inside the tab's faces (about 10 N a finger), and swings the arm up about its
   hinge to 32 degrees above level. The grasp is checked by the measured opening (16 mm) and tried again up to
   twice; the lift is raised further until the measured angle is within 3 degrees.
2. `reach note`, `draw note off the case`: the right gripper, tilted 30 degrees below level and turned so that
   its pads close one above the other, straddles the note's overhanging end with pads 24 mm apart, closes fully
   on the end strip (pads stop on the 2 mm collider: about 5 N a finger), is checked (opening above 0.8 mm and
   the end strip within 15 mm of the gripper) and tried again up to twice, then moves up and back toward the
   robot in the note's own lane until the note hangs free.
3. `carry note over the till`, `lay note under the hold-down`: sideways to the middle compartment at 215 mm
   height; the gripper is shifted and turned about the vertical until the hanging note is centred and square to
   the compartment (it has 2 mm to spare on each side); then forward and down, so that the note's far end lands
   on the stack in front of the raised hold-down and slides to the back under it; finally the gripper is moved
   until the note's far edge is 10 mm from the back wall.
4. `let go of note`, `let hold-down down`: the right gripper opens and draws back, and the note's raised end
   falls flat; then the left gripper swings the hold-down down onto the note, opens and rises. (In this order
   because there is no room for both hands at the compartment while the hold-down comes down.)
5. `reach drawer front`, `push drawer shut`, `away`: the right gripper, closed, pushes the drawer front 60 mm
   above the table at 7 cm/s until the drawer is home (it leans further in if the arm gives), and both arms go
   back to ready.

Dry run (`tools/dryrun_demo.py`, no contact physics): 1.0 in 943 steps; in the simulator: 1.0 in 968 steps.

### Two hands, at the same time

In the run the left hand held the hold-down up for the whole time the right hand had the note: both left
fingertips were loaded on the tab (never under 9.6 N) and both right fingertips on the note from step 310 to step
625, and through the laying-in (steps 518 to 627) the hold-down stayed at 29.7 degrees. The note went under the
hold-down because it was being held up at that moment. The other phases take turns: the right hand lets go of the
note before the left lets the hold-down down (there is no room for both hands at the compartment then), and one
hand pushes the drawer shut.

What if the hold-down is not held up? With it down, its free end rests on the stack with about 1.5 N, so there is
no gap to slide the note into. A limp note pushed against it buckles: the strips' hinges give way at roughly half
a newton of end push (hinge stiffness over strip length, 0.009 N m/rad / 19.5 mm), and pushing a square edge
against the arm's end face gives almost no lifting turn about its hinge. The note then lies on the arm or in front
of it: 0.5 at best (on the arm, in the compartment) and 0.25 or 0 if it hangs over the walls or out of the drawer.
`TILL_DEMO_VARIANT=no_lift` runs exactly that, the right hand's half of the demonstration with the left hand idle;
in the stand-in world it scores 0.25 (the note drapes over the hold-down and a part of it ends above the rim). It
has not been run in the simulator. The scorer itself does not require two hands: a single hand that lifted the
hold-down with the same fingers that hold the note, or wedged a strip under its free end, would be scored on the
end state like anyone else. Nothing in the scene prevents that; we know of no way it works and have not tried it.
(Cashiers themselves often flip the clip up with the thumb of the hand that holds the note.)

## What remains uncertain

- The note bends along its length only, is 2 mm thick and 8 g; a real note also curls across its width, creases
  and slips more easily in the fingers.
- The hold-down's shape and force, the compartment's inside size and the drawer's travel are estimates until the
  drawer is bought and measured; the latch and ejector spring are not modelled, so "shut" is pushed fully home.
- The note is put down with one end over the edge of the case so that a gripper can take it; flat on the case with
  no overhang it could not be picked up by this gripper.
- One scripted trial and one Dream-1 trial have been run; the negative control without the left hand has been run
  only in the stand-in world.
- In the overhead policy view the hold-down is 3 pixels wide.

## Tests and dry runs

```sh
python -m pytest -q tests
python tools/dryrun_demo.py --yam-arena <yam-isaac-lab-arena checkout> --events
```

The tests need NumPy and pytest; the dry runs also need SciPy and yam-arena's pure-Python parts and are skipped
without them. They cover the scoring tiers and every strategy in the table above, the scene file against
`till_model.py` (bodies, masses, springs, limits, that every collider is a box, that the note's colliders are not
drawn and its drawn sheet does not collide), `task.json` against the code, pixel sizes in the overhead view, and
the whole demonstration in a stand-in world scored by `grading.py`: nominal, with drooping shoulders and slow
arms, with the note hanging 4 mm to one side, and four shortened versions (drawer left open: 0.75; hold-down
never let down: 0.5; hold-down never lifted: at most 0.5; only the drawer shut: 0), and the qualification run's
numbers (`tests/test_results.py`). `tests/kinematic_world.py` says exactly what the stand-in models:
arms with a lag, the hold-down following the gripper that holds its tab, the note as the least-energy shape of
the real hinge chain above the till's real surfaces (the hold-down's top included, unless the note went in while
it was raised) without friction, the drawer moving with the fingertips that push it. It has no contact physics and does not replace a run in the simulator.

## Third-party assets and licenses

- Drawer, till, hold-downs and notes are boxes generated by `rdb_till_note/build_scene.py` (plus drawn-only
  cylinders for the lock and hinge knuckles); no meshes, textures or images from any third party are included.
  Product names are used only to identify what was modelled; the maker's photographs were looked at, not copied.
  The print on the notes is our own pattern of rectangles and reproduces no banknote design.
- Robot, station and cameras: `yam-arena` (the yam-isaac-lab-arena repository above; its README states that it
  has no license yet) with I2RT's YAM meshes (MIT, `LICENSE.i2rt` in that repository). They are dependencies,
  not copied here. `tools/clearance.py` reads those meshes at test time to measure clearances.
- Inspect Robots, inspect-robots-yam and the Dream client are installed dependencies under their own licenses.
- The code in this folder is under the MIT license (`LICENSE`).
