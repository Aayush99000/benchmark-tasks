# Aim a spring-balanced desk lamp onto an open notebook

Created 11_16_26 (America/New_York).

> Light the notebook for reading: aim the lamp at the red dot, with the shade at least 30 cm above the desk.

An Anglepoise Type 75 desk lamp stands behind and to the right of an open notebook, parked low and swung to the side, its
light on the bare desk 13 cm from a red dot on the right-hand page. The robot has to bring the shade up over the notebook
and aim it so the beam's centre lands on the dot, then take its hands away without knocking the lamp off target.

What makes it a physical skill rather than a choice: the lamp is a coupled, spring-balanced linkage (swivel, shoulder,
elbow, head knuckle), and each joint has its own stiffness. The shade's knuckle is the stiffest for its lever, so a push
on the shade alone swings the whole arm about its elbow instead of tilting the shade. The way people do it is the way the
demo does it: one hand holds the arm near the head while the other tilts the shade on its knuckle, then both let go.

## How it differs from the nearest tasks

- **#636 (closed, final)**, a bimanual foldable desk lamp: there the lamp was unfolded inside a tape frame and switched on,
  and the review faulted that illumination was never scored. Here nothing is unfolded; the score is where the beam lands
  on the page, to the centimetre.
- **#2036 (closed)**, raising and turning a monitor on a counterbalanced stand, and **#1742 (closed)**, stowing the clamp
  lever of an articulated bench light: both adjust a single support to a pose. Here the target is a remote point that the
  shade's axis must hit, through a four-joint coupled linkage, with the brace-and-aim hand split.
- The board has no lamp-aiming task; BEHAVIOR-1K has lampshade cleaning and bulb changing; RoboCasa, LIBERO, SimplerEnv,
  DataCenterBench, RoboDojo and the Embodied First Exam list have no lamp adjustment. This team has no other lamp task.

## Run and score

From this folder, with the simulator's Python environment active and this package installed
(`uv pip install -e . --no-deps --python "$(which python)"`):

    inspect-robots run --embodiment yam_desk_lamp -E task=task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 --policy yam_scripted --no-prompt \
      --instruction "Light the notebook for reading: aim the lamp at the red dot, with the shade at least 30 cm above the desk." \
      --scorer task_score --max-steps 2400 --seed 0 --store-frames --log-dir lamp-demo-logs

`dev/run.sh <label>` wraps the same command. The run prints `task_score`. The layout has no randomness (`object_jitter=0`,
`--seed 0`); the demo is open loop from the fixed start. A trial takes about 3 minutes on an L40S cloud GPU including
Isaac Sim's start (about 40 s of simulated time for the demo, which ends on success).

Shortcut variants of the demo: `LAMP_DEMO=nothing|one_hand|no_tilt|low|hold`. Scorer unit tests:
`python -m pytest test_score.py`. `LAMP_DEBUG=1` prints the beam's distance from the dot, the shade's height and the
joint angles every step.

## Files

| File | What it is |
|---|---|
| `task.json` | the task card |
| `task.py` | layout, start and goal poses, the scorer and the scripted demo |
| `desk_lamp/mech.py` | the lamp and notebook: dimensions with sources, kinematics, USD builders |
| `desk_lamp/embodiment.py` | `yam_desk_lamp`: the stock embodiment plus the lamp articulation and the notebook, read through PhysX tensor views |
| `desk_lamp/grip.py` | lets a demo waypoint carry a full hand orientation (the hands work level, 45 cm up) |
| `test_score.py` | scorer unit tests on synthetic states |
| `dev/` | `run.sh` (one trial), `sheet.py` (contact sheet of camera frames), `media.sh` (video and screenshot) |

## Where the numbers come from

| Quantity | Value | Source |
|---|---|---|
| Shade | 14.5 cm across the mouth (5.7 in), 19.05 cm tall (7.5 in) | Anglepoise Type 75 desk lamp technical sheet, US V4 and UK V2.0 |
| Base | 19.5 cm across (7.7 in), cast iron | same sheets |
| Maximum reach | 28 in horizontal from base to shade, 35.4 in vertical | US sheet |
| Weight | 3.1 kg | Anglepoise support page for the Type 75 desk lamp |
| Lower arm, upper arm | 30.1 cm, 30.4 cm between pivot centres | measured off the sheet's side-view drawing (below) |
| Shoulder pivot | 21.4 cm above the desk, 2.6 cm behind the swivel axis | drawing |
| Head knuckle | 4.3 cm from the shade's axis, 4.8 cm below the shade's top | drawing |
| Cap | 6.2 cm across, 8.3 cm long before the bell flares | drawing |
| Notebook | 13 x 21 cm, 240 pages, about 0.33 kg (0.71 lb shipping weight) | Moleskine Classic Notebook, Large, hard cover, retailer listings |
| Moving-part masses, joint friction | swivel 0.15, lower arm 0.30, upper arm 0.15, head 0.30 kg; 0.35, 0.60, 0.45, 0.30 N m | estimates (see Fidelity) |

The drawing: the sheet's side view is vector artwork, with every pivot drawn as a small circle. Its scale comes from the
printed dimensions: 7.90 pt per inch across (from the 12.2, 7.7 and 5.7 in dimensions, which agree within 1.5%) and
8.34 pt per inch up (20.5 and 7.5 in). With both arms laid out level, the measured layout reaches 27.3 in from the base's
axis to the shade's far rim, against the published 28 in, and stands 20.97 in tall in the drawing's pose, against the
printed 20.5 in. The measured lengths are good to about 3%.

## The scripted demo

Open loop from the fixed start; it reads no simulator state. The lamp's path is worked out offline from the lamp's own
geometry. The right hand comes in level from the robot's side and pinches the upper arm's two rods from above and below,
9 cm from the head knuckle, then carries the arm along the lamp's own joint path (swivel, shoulder, elbow) to where the
goal needs it. The shade rides along on its stiff knuckle, still tilted 25 degrees too far. Holding the arm there, the
left hand pushes the shade's near side with its closed fingertips (about 2.8 N), turning it on its knuckle until its axis
points at the dot, and backs away; then the right hand opens and backs off the rods. The plan carries the arm a few
degrees past the goal and pushes the shade 4.2 degrees past it, for the give of the rods in the pinch and of the
fingertips on the shade (constants in task.py). About 31 s of simulated time; the trial ends on success.

## Measured in the simulator

Cloud L40S GPU, `object_jitter=0`, `--seed 0`, 20 Hz, 320 x 176 cameras (the media run at 640 x 352).

| Run (label) | `LAMP_DEMO` | task_score | Final reading |
|---|---|---|---|
| g5 | demo | 1.0 | beam 18.0 mm from the dot, mouth 34.2 cm up, success at 31.4 s |
| demo2 | demo | 1.0 | identical: 18.0 mm, 34.2 cm |
| v-one_hand | `one_hand`: the arm is carried, let go, then the shade pushed without holding the arm | 0.25 | the arm swung down about its elbow (9.6 degrees) and the shade did not turn; 94 mm, 30.8 cm |
| v-no_tilt | `no_tilt`: arm carried, shade never re-aimed | 0.0 | 289 mm |
| v-low | `low`: same aim with the mouth 22 cm up | 0.25 | 26 mm but 21.4 cm up (capped) |
| v-hold | `hold`: right hand never lets go | 0.0 | 17 mm on target but never clear |
| media | demo, 640 x 352 cameras, the final code | 1.0 | success |
| v-nothing2 | `nothing`: both hands stay at the ready pose for 60 s | 0.0 | 132 mm; the lamp stays put |

The `one_hand` run is the point of the task: the same push on the shade that re-aims it when the arm is held swings the
arm instead when it is not.

## Simulation model

- One articulation with a free root: the cast-iron base (3.1 kg, resting on the desk, Coulomb friction 0.6) carries the
  swivel on a vertical joint; lower arm, upper arm and head each turn on a horizontal joint square to the arm's plane.
  Arms are pairs of 9 mm rods; the elbow has its round cover plate; the shade is a cap and a five-step bell.
- Springs: the real lamp balances its arms with springs at the base ("constant tension spring technology"), and holds any
  pose by itself. Spring rates and anchor points are not published, so the simulation stands in an ideal balance: the
  moving links keep their mass and inertia but carry no weight, and Coulomb joint friction holds the pose. With no robot
  motion, and in the 10 s after the demo's hands leave, no joint angle changes (to 0.1 degree, the printed precision).
- The joint friction values are estimates chosen so the real lamp's behaviour holds. In the goal pose, a level pull
  toward the base on the bottom of the shade's near rim turns the shade on its knuckle at 2.1 N if the arm is held, but
  with the arm free the arm swings about its elbow first, at 1.8 N; a sideways pull of 2.0 N at the knuckle swings the
  arm on its swivel. The reset notes give the spring-scale test that matches a real lamp to these figures.
- The light is not rendered. The beam's centre is where the shade's axis (through the mouth's centre, square to it) meets
  the page, computed every step from the head link's simulated pose and the notebook's simulated pose.
- The notebook is one loose rigid block, 26 x 21 cm and 9 mm thick, 0.33 kg, with the dot drawn on it.

## Scoring, and how a real trial measures it

| Criterion | Simulation | Real trial |
|---|---|---|
| Hands clear | each fingertip midpoint at least 5 cm from both arms' centre lines and 11 cm from the shade's axis | the robot's hands visibly away from the lamp; the supervisor waits 3 s after the robot stops |
| Lamp settled | no joint moved more than 0.3 degree during 3 s with the hands clear | the laser spot does not move for 3 s (photo at the start and end) |
| Beam's centre to the dot | distance in the page's plane, from the shade's axis meeting the page to the dot's centre | steel ruler from the red laser spot to the dot's centre; overhead photo |
| Shade height | the mouth's centre at least 30 cm above the desk | tape measure from the desk to the middle of the shade's mouth |
| Notebook and base not moved | at the reading, each within 20 mm of its start and tilted under 3 degrees (a bump that leaves them in place does not count) | masking-tape outlines round the base and the notebook; ruler from the outline |

Bands: 1.0 within 30 mm with the mouth at least 30 cm up; 0.5 within 60 mm with the mouth at least 30 cm up; 0.25 within
100 mm, or within 60 mm with the mouth lower than 30 cm; otherwise 0. A shade facing level or up has no spot and scores
0. Moving the notebook or dragging or tipping the base scores 0, so sliding the page under the light earns nothing. The
reading is taken only after 3 s with the hands away and the lamp still, so holding the lamp on target, or letting it
drift off, earns nothing. Doing nothing scores 0 (the beam starts 132 mm from the dot).

## Real-world setup and safety

The lamp stands free on its own base on a desk or table at least 90 x 60 cm; the notebook lies open at its middle, its
spine square to the desk's front edge, the dot stuck in the middle of the right-hand page. Masking-tape outlines mark the
base and the notebook. The lamp's start pose is set from a printed side-view template.

The beam's centre is made visible by a Class 2 (1 mW), 650 nm dot laser module held centred on the bulb's dome by a
paper collar, pointing along the shade's axis; it weighs a few grams, well inside the bulb mass the lamp's sheet allows
(25 to 45 g). Its 2 x AAA (3 V) battery box with an on/off switch stays on the desk, its thin lead taped along the arm.
A Class 2 beam (1 mW or less) is safe for a momentary accidental glance, which the blink reflex limits; the
5 mW module first specified was replaced for this reason. Safe aim before switching on: the supervisor waits until the robot has stopped with its hands away,
confirms by eye that the shade's mouth faces down onto the desk (its axis below horizontal and the notebook under it),
that nobody's face is at or below the shade's height within 1 m, and that no mirror or glossy surface is in the beam's
path; only then is the module switched on, and it is switched off before anyone reaches toward the lamp. The lamp's own
bulb is a 6.5 W LED and runs cool; the lamp can stay unplugged for scoring. The robot moves only light joints: the
largest push the demo needs is about 3 N at the shade (2.8 N turns the shade at the point it pushes).

## Items for a real trial

| Item | Product | Price | Where |
|---|---|---|---|
| Lamp | Anglepoise Type 75 Desk Lamp, Jet Black (supplied with an LED bulb) | $320 | https://aplusrstore.com/products/anglepoise-type-75-desk-lamp ; https://www.dwr.com/lighting-desk-lamps/type-75-desk-lamp/2196970.html |
| Notebook | Moleskine Classic Notebook, Large, hard cover, 240 pages | $26.00 | https://www.target.com/p/moleskine-notebook-classic-large-hardcover/-/A-89487844 |
| Dot | Avery 5466 removable color-coding labels, 3/4 in round, red, 1,008 | $14.29 | https://www.staples.com/avery-laser-inkjet-color-coding-labels-3-4-dia-red-1008-labels-per-pack-5466/product_166751 |
| Beam marker | Picotronic 70100594 red dot laser module, Class 2, 1 mW, 650 nm, 2.7-3.3 V | GBP 40.14 (about $51) | https://www.rapidonline.com/picotronic-70100594-red-class-2-laser-10m-1mw-2-7-3-3vdc-30ma-650nm-wavelength-57-5872 |
| Its battery box | Adafruit 4191, 2 x AAA holder with on/off switch (3 V, plus two AAA cells) | EUR 5.75 (about $6) | https://opencircuit.shop/product/2-x-aaa-battery-holder-on-off-switch-jst-ph |
| Joint stiffness test | Arbor Scientific 1000 g (10 N) push-pull spring scale | $10.95 | https://arborsci.com/products/1000g-push-pull-spring-scale |
| Shade height | Stanley 16 ft PowerLock tape measure | $11.97 | https://www.homedepot.com/p/Stanley-16-ft-PowerLock-Tape-Measure-33-116L/100654911 |
| Spot to dot, outlines | Westcott 10415 12 in stainless steel ruler, cork back | $8.49 | https://www.quill.com/westcott-stainless-steel-rulers-12/cbs/001457.html |
| Outlines | ScotchBlue 2090 painter's tape, 0.94 in x 60 yd, one roll | $10.27 | https://www.walmart.com/ip/ScotchBlue-Original-Multi-Surface-Painters-Tape-94-inch-x-60-yard-2090-1-Roll/471970548 |

Total about $460, all reusable; nothing is consumed per trial (one sticker dot lasts many trials; the AAA cells last
many sessions).

## Reset

Put the base and the notebook back in their outlines; set the lamp to its start pose from the template (the shade's mouth
16 cm up over a tape cross). Before a session, with the lamp in the goal pose and the upper arm held, pull the bottom of the
shade's near rim level toward the base with the spring scale: the shade should start to tilt at about 2.1 N. With the arm
free, the same pull should swing the arm first, at about 1.8 N; a sideways pull at the head knuckle should swing it at
about 2.0 N. If a real lamp differs, set FRICTION in desk_lamp/mech.py to match.
