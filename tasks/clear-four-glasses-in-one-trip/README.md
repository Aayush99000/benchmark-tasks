# Clear four glasses into a bus tub in one trip

A bus tub stands on the table between the two arms. Two 16 oz plastic restaurant tumblers stand on the table on each side of it, 10 cm apart, each still holding three plastic ice cubes and a straw. The robot has to get all four into the tub in one trip: each gripper takes its two tumblers at once, a finger down inside each with the two walls pinched together, lifts them over the tub's 12.5 cm wall, stands them on the tub's floor and comes away. To take two at once the tumblers first have to be brought rim to rim, by pushing one low on its wall so that it slides and does not tip. Instruction: "Clear the four glasses from the table into the bus tub in one trip: each hand goes to the tub only once."

Install once, from this folder: `pip install -e . --no-deps`

Run a trial:

    inspect-robots run --embodiment yam_bus -E task=task.py -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 --policy yam_scripted --no-prompt \
      --instruction "Clear the four glasses from the table into the bus tub in one trip: each hand goes to the tub only once." \
      --scorer task_score --max-steps 1800 --seed 0

The score is printed as `task_score`. Use `--policy dream` for Dream-1.

- **Runtime per trial:** about 5.5 minutes on a cloud GPU (319 s and 327 s wall clock for the two logged demo runs, including simulator start-up); the demo ends at step 796 of 1,800 (40 s simulated).
- **Seeds:** none. The start is one fixed layout: the ice and straws start inside the tumblers, so `yam_bus` does not shift objects on a seeded reset (`object_jitter` is forced to 0). `--seed 0` was used for the demo.
- **Scoring tests, no simulator** (run in the Python environment that has `yam_arena`; Isaac Sim is not started): `python -m pytest -q test_score.py` (55 tests: every band, and each shortcut listed below). `python task.py` reports the demo's reach and length.
- **The scripted demo** is open loop. It uses only the fixed starting layout (where the tub and the four tumblers stand), which a policy can see, and reads nothing from the simulator. Both arms do the same thing, mirrored, at the same time. Each hand leans 40 degrees with its fingertips forward and 36 mm apart, comes down behind its nearer tumbler with the fingertips' ends 22 mm above the table, and pushes it 9 cm along the row until the two rims are about 4 mm apart. A hand held straight down cannot do this: its fingers widen toward the palm, so it would meet the tumbler high on its wall. The hand then turns its fingers in line with the row, goes down with one finger 28 mm inside each rim, and closes. The two walls and the pads' flat faces stop the fingers about 11 mm apart; the squeeze draws the rims together and the two tumblers swing base to base, each 4.6 degrees from upright (the wall's taper), hanging from the pinch. The hand lifts them, leans 25 degrees forward (a hand held straight down cannot rise above 22 cm, and the tumblers' bases have to clear a 12.5 cm rim), carries them over the tub's long wall at 31 cm, straightens as it comes down, stops with their lowest edges about 4 mm above the tub's floor, opens, lifts its fingers out, rises above the straws and moves away.
- **What the physics does:** every body is rigid and everything is contact and friction; nothing is ever attached to a gripper and no state is set by a criterion. Each tumbler is hollow: 24 flat wall panels 2.5 mm thick round a floor disc, each panel its own convex collider, so a finger really is inside the tumbler and the two walls really are squeezed between the pads. The ice cubes and the straw are loose bodies inside. The tub is loose on the table (0.64 kg) and slides if it is knocked. The score reads only poses.
- **Shortcuts tried in the simulator** (`BUS_DEMO=<name>`, for testing the score; see Results): `none`, `half` (one hand only), `single` (one tumbler at a time, each pinched by one wall: four carries), `high` (the push is made 12 mm below the rim instead of low on the wall), `held` (the full demo, but the hands never let go). `BUS_DEBUG=1` prints each step's measurements. `python dev/sim.py demo demo none ...` plays several episodes in one simulator session.
- **Third-party assets:** none. Every part is built from boxes, capsules and simple convex meshes in the task's own code.

## How this differs from the nearest tasks

Bussing has been proposed here before as a sorting task (glasses into a rack, plates stacked in a tub), with solid cylinders for glasses and one object in the gripper at a time; the review asked for realistic hollow glasses. The accepted one-trip task in this repository carries one stack of three boxes squeezed between both arms, and a pending dishwasher task loads two cups one at a time. A task under revision moves four sealed cups of unknown weight and scores a cup 0 if it shares a gripper with another. BEHAVIOR-1K's `collecting_dishes_from_restaurant_tables` and `cleaning_glasses_off_bar` and RoboCasa's `clearing_table` tasks score only where objects end up and are done one object per grasp; dexhand's Tray Glass Carry carries glasses on a tray, a different skill. Here the tested ability is a multi-object grasp that only works because the tumblers are hollow and true size: one finger inside each of two tumblers, their adjacent walls pinched together, after herding them rim to rim without tipping them, and then lifting the hanging pair over a wall nearly as tall as they are. Taking them one at a time, if done without a drop, scores 0.5.

## Results

Scripted policy (`yam_scripted`), seed 0, fixed layout, 20 Hz, on a cloud GPU; a further demo on an aarch64 host also scored 1.0. One Dream-1 run (`--policy dream`, 1,800 steps) scored 0.0: the arms reached toward the outer tumblers but never pinched a pair, and nothing was moved.

| Run | Steps | Score | Notes |
|---|---|---|---|
| Demo, `inspect-robots run`, first | 796 | 1.0 | carries: left 1, right 1 |
| Demo, `inspect-robots run`, second | 796 | 1.0 | the same |
| Demo, `dev/sim.py`, two more episodes | 796, 796 | 1.0, 1.0 | the same; while being pushed no tumbler leaned at all (0.0 degrees), the pushed rims stopped 4.4 mm apart, and the hanging pairs leaned at most 28 degrees during the carry with their contents inside |
| `none`: the hands come down and wait | 141 | 0.0 | nothing delivered |
| `half`: the left hand only | 867 | 0.25 | two delivered in one carry |
| `single`: one tumbler at a time, each pinched by one wall, four carries | 1,111 | 0.0 | three delivered; the left hand's second tumbler swung out of its one-wall pinch and fell over (latched), spilling its ice |
| `high`: the push made 12 mm below the rim | 867 | 0.0 | both pushed tumblers tipped over (60 degrees, then 85) |
| `held`: the full demo, never letting go | 718 | 0.0 | the scripted plan ends 2 s after the pairs are set down, with the hands still on them and the tumblers still moving, so none counts as delivered; held still for longer this would score 0.5 (a hand within 6 cm), never 1.0 |

The 0.5 band (all four delivered in more than one trip) is covered by the unit tests; the simulated one-at-a-time shortcut did not reach it, because a single tumbler held by one wall swings and was dropped.

## The objects and their numbers

- **Tumbler.** A 16 oz stackable pebbled SAN (styrene-acrylonitrile) restaurant tumbler: 3 1/4 in (82.6 mm) across the rim, 2 3/8 in (60.3 mm) across the base, 5 7/16 in (138.1 mm) tall, 17.7 oz to the rim. These are a restaurant-supply listing's figures for a common model; other makes of the same class are within a few mm. A case of 72 ships at 14.18 lb, 89 g apiece with the carton, so 85 g is used. The 2.5 mm wall and 4 mm floor are assumed (typical for moulded SAN drinkware; the listing does not give them). The wall leans 4.6 degrees.
- **Ice.** Reusable plastic ice cubes, 1 in (25.4 mm), 13 g each (a retail pack of 40 weighs 531 g). Three per tumbler. Two do not lie flat side by side on the tumbler's 55 mm floor, so they sit wedged and stacked, as real cubes do.
- **Straw.** A 7 3/4 in (197 mm) jumbo paper straw, 6 mm across, 1.3 g (a case of 4,800 weighs 13.51 lb). It stands on the tumbler's floor and leans on the rim, sticking 4 cm out.
- **A filled tumbler** weighs 125 g; a hand carries 250 g.
- **Bus tub.** A 20 x 15 x 5 in polyethylene bus box: 19 15/16 x 15 5/16 x 4 15/16 in (50.6 x 38.9 x 12.5 cm) over its rim, 17 x 12 5/8 in (43.2 x 32.1 cm) inside at the floor, 4 5/8 in (11.7 cm) deep inside, 1.4 lb (0.64 kg), from a manufacturer's catalogue and a supplier's listing. The walls therefore lean out about 11 degrees. The 3 mm wall and the 10 mm flat lip standing in for the rolled rim are assumed.
- **Friction.** PhysX averages the two bodies in contact. Tumbler 0.4, ice 0.3, straw 0.4, tub 0.3; the station's table and the gripper's fingers have the simulator's default 0.5. So: tumbler on table 0.45, tumbler on finger 0.45, tumbler on tumbler 0.4, tumbler on tub floor 0.35, tub on table 0.4. At 0.45 a tumbler pushed more than 67 mm above the table tips before it slides (base radius 30 mm / 0.45).
- **Pinch.** The gripper's fingers are position-driven (5,000 N/m). Commanded 4 mm apart, they are stopped about 11 mm apart by the two walls, so each presses with about 17 N. Each finger has to hold up 1.2 N by friction.

## Real-world setup

- **Table and layout:** the station table, bare. Tape outlines mark the tub's four corners and each tumbler's base (60 mm circles) at the positions in `task.json`. Measure each tumbler's centre from the tub's long wall (12.6 cm from the rim's outer edge) and the gap between the two rims of a pair (10.0 cm) with a ruler before each trial.
- **Tumblers:** four identical 16 oz stackable pebbled SAN tumblers of the dimensions above, any colour that the cameras see well against the table (blue in the simulation). Dry inside and out.
- **Contents:** three 1 in reusable plastic ice cubes and one 7 3/4 in paper straw in each tumbler, the straw's top leaning on the rim on the side away from the tub. Plastic cubes are used instead of ice so that no water is near the robot and nothing melts between trials. No liquid.
- **Tub:** a 20 x 15 x 5 in polyethylene bus tub, empty, dry, loose on the table (not taped down), long side pointing away from the robot.
- **Grippers:** the standard fingers and pads, clean and dry. Wipe the pads between sessions.
- **Safety:** everything the robot handles is plastic and light: no glass, nothing sharp, hot or wet; a hand carries 250 g. Nothing is pressed, clicked or triggered, and the pinch is the gripper's ordinary grasp on two 2.5 mm walls. A dropped tumbler falls at most 30 cm onto the table or into the tub and does not break; a raised edge or a towel round the table keeps spilled cubes off the floor. Nobody reaches into the workspace while the arms are enabled, and the usual emergency stop is within the supervisor's reach.
- **Hygiene, for anyone copying the method:** fingers go inside glasses only when clearing dirty ones, never clean ones, and hands are washed after handling soiled ware.

## Scoring: real measurement and simulated stand-in

The real trial is scored from the video and from the tub and the table at the end, nothing moved. Tools: a 30 cm ruler, a camera, a count.

The bands (numbers in `task.json`'s scoring): 0 if any tumbler was knocked over at any time, anything fell off the table, an ice cube or straw lies outside the tub and outside every tumbler, or fewer than two tumblers are delivered; 0.25 with two or three delivered; 0.5 with all four delivered but not in one trip, or a gripper still at a tumbler, or something still moving; 1.0 with all four delivered in one trip, both grippers clear and everything still for 1 s.

| Criterion | Real measurement (how) | Simulated stand-in |
|---|---|---|
| Delivered: standing in the tub | The tumbler stands on its base on the tub's floor, not on another tumbler, the rim or the table. Within 15 degrees of upright: with the ruler held upright beside the rim, the rim's high side is no more than 2 cm above its low side (82.6 mm x sin 15 degrees = 21 mm). It may lean on a neighbour or a wall within that. It does not move in the last 1 s of video. | Its base's centre lies between the tub's walls at floor level and no more than 8 mm above the tub's floor, its axis is within 15 degrees of the vertical, and it moves at under 1 cm/s and 0.5 rad/s; all in the tub's frame, wherever the tub ends up. |
| Delivered: with its contents | Count: three ice cubes and the straw are in that tumbler. | The centre of each of its three cubes and of its straw lies inside that tumbler's hollow. |
| Knocked over (score 0, latched) | From the video: at any moment a tumbler is more than 45 degrees from upright (lying down, or leaning further than a corner-to-corner diagonal). Standing it up again does not undo it. | A tumbler's axis more than 45 degrees from the vertical at any control step. |
| Spilled (score 0) | Look: an ice cube or a straw lies on the table or the floor, outside the tub and not in a tumbler. A cube loose in the tub is not a spill, but the tumbler it left is not delivered. | A cube's or straw's centre outside the tub's inside and outside every tumbler's hollow. |
| Off the table (score 0, latched) | Look: a tumbler, cube, straw or the tub is on the floor. | A body's origin more than 5 cm below the table top at any step. |
| One trip: carries | From the video, count for each gripper the times it brings tumblers in over the tub's rim. Tumblers that pass over the rim in the same gripper within 1 s of each other are one carry. More than one carry by either gripper is not one trip. A tumbler that gets into the tub without a gripper carrying it (thrown, tipped in) is not one trip. | Each time a tumbler's base centre passes inward over the outline of the tub's rim, the gripper whose fingertip midpoint is nearest its rim's centre, and within 12 cm, is credited (no gripper within 12 cm: credited to none, which is not one trip). A gripper's crossings within 1 s of each other are one carry. |
| One trip: no nesting | From the video: no tumbler was put inside another at any time. | A tumbler's base centre inside another tumbler's hollow at any step (latched). |
| Grippers clear and still | In the last frame each gripper's fingertips are 6 cm or more from every tumbler (ruler against the frame, or measured on the table before anything is moved); nothing moves in the last 1 s of video. | Each fingertip midpoint 6 cm or more from every tumbler's surface; tumblers under 1 cm/s and 0.5 rad/s and fingertip midpoints under 2 cm/s for 1 s. |
| Not simulated | Leftover drink, condensation, lemon wedges and napkins; the pebbled texture and the rounded rim; chipping or cracking. The real pads grip SAN better than the simulated 0.45. | |
