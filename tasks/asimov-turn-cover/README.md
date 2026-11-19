# Turn a humanoid's hung upper body round on its gantry hook, then keep it still and fit its back cover

The upper body of Asimov 1, Menlo Research's open-source humanoid (the torso shell, both arms hanging straight, and the head; 18.3 kg), hangs by its shoulder shackles from a bench gantry's swivel hook over the table, facing the arms, its back open. It is to be turned round so that its back faces the arms, held still there, and its back cover, Menlo part 100_04C, pressed into place. On the swivel it turns freely: the push that starts it loads it with momentum that the hook's drag takes well over a full turn to remove, so it has to be stopped, where its back is within reach, without a blow. Once there, any push off the swivel's axis turns it again, and the cover's own press is such a push: 5 N pressed 3 cm off the axis sets the 0.26 kg m2 body turning at about 16 deg/s within half a second, carrying the back, and the cover with it, off its place. So the cover has to be pressed straight through the swivel's axis, or the body held while it goes on; in the demo one hand holds the body while the other carries the cover from its rest, turns it upright and seats it on the back within 5 mm and 5 degrees.

In the demo the right hand pushes the torso's front near its left edge; the body starts turning at about 50 deg/s, and the right pulls back out of its sweep. The left meets the rear face of the body's left upper arm 12 degrees before the mark and gives way with it, braking the spin to rest. The left then takes the body's left wrist, below the torso, and holds it. The right takes the cover by its handle bar, draws it out of its rest and lifts it clear, turns it upright and brings it square to the open back 10 cm out. Steering by the cover itself (where its handle bar is, and how far it is turned from its place), it squares the cover up, moves it straight in onto the shell's ledges and holds it there; the trial ends when the cover's centre has been within 5 mm, and its turn within 5 degrees, of its place for 1 s, with the body still.

**Why it matters.** Robots that build robots have to handle robots under assembly. Asimov 1 is built hanging from a gantry hook, and its build guide's steps 10.6 and 10.7 read: "Turn the robot around so we can now work on the back cover." and "Take part 100_04C and put on the back of part 100_01C." Here the upper body hangs alone on a bench gantry. It stands in for the whole hung robot, whose legs would not clear the table within the arms' reach. The cover's 8 M4 bolts (step 10.8) are left out, so a hand holds the cover home at the end, as it would while the first bolt goes in.

**Ability tested:** two-handed work on a robot that turns freely on a swivel: turning a heavy body half a turn and stopping it without a blow where its back is within reach, then keeping it there while a curved 117 g shell is carried from its rest and seated on it within 5 mm and 5 degrees. A hard block, or one the body bounces off, scores 0. A press off the swivel's axis turns the body off the mark unless it is held.

**Nearest tasks.** A hoisted motor lowered onto studs, where the hands correct its yaw as it lands: there the load hangs on a hoist and the yaw is set slowly. A revolving display turned and held while boxes go into its pockets: there the turntable is a fixture and the parts go into pockets. Here a hung robot spins freely, is stopped on its mark, and a shell is fitted onto it while it is held. This task is not a fallen-robot recovery: the robot is under assembly.

**Arms.** The instruction does not ask for two hands. In the demo one hand holds the body while the other fits the cover. One hand alone can also fit it, if it presses straight through the swivel's axis: the `onehand` check below scores 1.0.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder. `upper_body.npz` holds the body's meshes, colliders and mass properties; `cover.npz` holds the cover's mesh, the open back's colliders and the cover's 4 check points (made by `dev/make_cover.py` from the maker's STL); `seat.npz` holds the shell's ledge pads, the cover's colliders and its foam rest (made by `dev/make_seat.py` from the maker's STEP files, tessellated by `dev/step_to_stl.py`). Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1600 \
  --instruction "Turn the body hanging from the swivel hook half a turn so its back faces you, then keep it still and press its back cover into place."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode turn --max-steps 1600                                 # full score, with a video
python demokit.py task.py OUT --no-record --mode turnonly,onehand,push,none --max-steps 1600   # the checks below
python check_score.py                                        # the scorer on hand-built trials, with no simulator
```

The demo reads the swivel's angle and rate, and the cover's pose, as a camera would. It reads nothing from the scorer.

## The scene and what is simplified

The upper body is one rigid body: the maker's URDF links for the torso shell, both arms and the head, merged at the zero pose with the arms rigid and hanging straight; a real trial locks the arms' joints straight. Its mass (18.28 kg) and inertia come from the URDF; its yaw inertia about the hook's axis is 0.257 kg m2. Each link's convex hull is its collider. It yaws on a revolute joint at the hook with a viscous drag of 0.0254 N m s/rad in all (the joint's damping, 0.0125, plus the simulator's default damping of the body's spin, 0.05 per second, 0.0129; no joint friction): a time constant of 10.1 s, so a free spin slows from 45 to 27.4 deg/s in 5 s and to 10.2 deg/s in 15 s (measured in the `push` run: 50.1 to 30.6 deg/s in 5 s, the same ratio every 5 s). The real hook is fitted with a magnetic brake and tuned to the same spin-down (next section). The shackles and chains are not modelled, so the body neither swings nor tilts.

The torso's back is open where the cover goes. The cover's stops are the shell's own ledges, taken from the maker's CAD: `dev/make_seat.py` keeps every rear-facing face of the torso shell (`ASV1_100_01C.STEP`) within 2 mm of the cover's inner face, with the cover fitted to them (4,590 faces, 10,233 mm2). They are a cross ledge across the top, a rail down each side and two short pads beside the handle slot, 71 convex pads of 3 mm in all (the pale grey pieces in the film). The cover's middle and foot touch nothing, as on the robot. Fit: 95 % of the ledge faces lie within 1.5 mm of the cover's inner face, and the URDF torso (a simplified copy) matches the STEP to a median 6 mm, so the pads' depth is set by the cover's own wall (2.5 mm collider, the maker's wall is 2.1 mm). Behind the cover the battery and boards are shown dark with no collider. The cover's colliders are the convex hulls of its own mesh in 22 mm cells, each split by the way its face turns, so that no piece fills the inside of a corner where the back turns into a side wing (148 pieces, slivers under 30 mm3 left out; the handle slot and bar are left to the handle box, where the fingers bear). Its place on the back is the STL fitted to the torso's mesh (median gap 2.8 mm).

The cover starts on a foam rest shaped to the back between its side wings, leaning back 15 degrees with its foot on a ledge. The gantry is a beam on two posts, outside the body's sweep (its arms reach 21 cm from the hook's axis).

## Scoring

See `scoring` in `task.json`, and `Trial` in `task.py`. `check_score.py` feeds the real `Trial` hand-built states for the honest task and each lazy or faulty route.

Scored by the effect: the turn is what brings the back within reach, and any turn that does is enough (about half a turn faces it to the arms); the heading itself is not scored.

| What happened | Score |
|---|---|
| The cover seated on the back (its centre within 5 mm, its turn within 5 degrees of its place), the body still (under 1 deg/s), for 1 s | 1.0 |
| The same with the cover within 15 mm and 10 degrees of its place, not seated | 0.5 |
| A blow, a collision, or left spinning over 20 deg/s at the end | 0 |
| Anything else | 0 |

A blow: the spin slows or reverses by more than 25 deg/s within 0.1 s, about 11 N pressed 10 cm from the swivel. A collision: the spin changes by more than 5 deg/s within 0.1 s while the body turns faster than 10 deg/s, with no fingertip within 5 cm of it. Spin changes are read over two steps. Distances to the body are measured to its colliders.

## Results

| Run | What it does | Score |
|---|---|---|
| Dream-1 | Run at 320 x 176, 1,600 steps | 0.0: the arms turned the body half a turn the other way (to -176 degrees) and left it still, but never took the cover off its rest |
| `turn` | The whole task (the film) | 1.0, at step 660 (33.0 s): pushed to 50 deg/s, caught on its left upper arm 12 degrees before the mark, at rest 3.8 degrees short; the left holds its wrist; the right draws the cover out of its rest, squares it 10 cm out by its own pose and seats it on the ledges, 2.6 mm and 0.8 degrees from its place |
| `onehand` | The same, but the left lets go of the body before the cover goes on | 1.0, at step 606: the right's press goes through the swivel's axis, so the body stays still |
| `turnonly` | The turn, then the hand away, no cover | 0.0 |
| `push` | The push alone, no catch, no cover | 0.0: past the mark by 351 degrees, still turning at 0.9 deg/s at 45 s |
| `none` | The arms wait | 0.0 |

`check_score.py` also runs, on hand-built states: the whole task the other way round (1.0); the cover 4 mm or 4 degrees off its place (1.0); 6 mm off, held 7 mm out, 7 degrees off, or sliding on from 2 mm off (all 0.5); 2 cm off (0); home only 0.9 s (0); home with the body at rest 12 degrees short of the mark (1.0: the heading is not scored); home on a body still turning at 3 deg/s (0) or turned by the press at 25 deg/s (0); home after a blow or a collision stop (0); and the turn-only cases: a blow, a collision, a block the body bounces off, two hands rattling it, left spinning (all 0).

## Runtime and seeds

The demo takes 33.0 s of simulated time (660 steps), about 2 to 3 minutes of wall time on one shared GPU at 640 x 360, start-up apart. There is no randomness: `object_jitter=0`, so no seed is needed.

## A real trial

Hang the upper body, in practice the low-cost stand-in below (`standin/`), from a ball-bearing swivel hook on a bench gantry, so that its centre of mass is 35 cm above the table at (45, 0). First run the spin-down test below. Print the cover 100_04C from the maker's STEP in PA12 (about 117 g) and stand it on a foam rest at (26, -50), leaning back 15 degrees. Bench check of the press: with the body hung still on the mark and nobody holding it, press the fitted cover 3 cm off the hook's axis with a 5 N push gauge and time the turn it starts. Put a contrasting mark on each shoulder for the camera above, and log yaw at 20 Hz or faster. Mark the cover's 4 corners (two by the handle bar, two at its foot) and their places on the shell's rim; read each mark's offset from a photo square to the back, or with a feeler gauge. Keep people out of the body's sweep (its arms reach 21 cm from the hook's axis), and check the hook's rated load.

## Stand-in for the suspended body (real trials)

Nobody has to own an Asimov 1. `standin/drawing.txt` is the dimensioned build (about 60 to 90 USD; 117 g cover print); `standin/budget.py` does the arithmetic below. Axes: x forward, y left, z up, origin at the body's centre of mass, which hangs on the hook's axis. Targets from `task.py` (Menlo's URDF, `upper_body.npz`): 18.276 kg, centre of mass on the swivel axis, yaw inertia 0.2566 kg m2.

- Torso: a 12 mm birch-plywood box, 220 x 250 x 380, back open, 3.06 kg with the ballast cleats. The cover (Menlo 100_04C, printed from the STEP, 117 g) is pressed into its open back until it meets six printed ledge blocks (`blocks.stl`, about 80 g in PA12, positions in the drawing): the shell's own ledges, the same as the scene's pads. The cover itself is the jig for gluing them. Ballast, the "battery": 7 steel plates 10 x 100 x 100 (5.495 kg) stacked along x at x +6 cm, z +7 cm. Head: a 90 x 130 x 180 pine block (1.05 kg) at x +1.2, z +29 cm.
- Arms: two 60 x 20 x 450 steel flat bars, 4.239 kg each (0.060 x 0.020 x 0.45 m x 7850 kg/m3), bolted to the sides, at x -5.3, y +-13.7, z -9 cm.
- Mass: 3.06 + 5.495 + 1.053 + 2 x 4.239 = 18.09 kg; 0.19 kg of washers on the head makes 18.28 kg.
- Centre of mass: Σ m x = 0 fixes the arms' x. With the ballast at x +6 cm, the arms sit at -5.3 cm (inside the real arm's -5.1 to +3.2 cm range, rounded); Σ m z = 0 gives -0.05 cm; y is 0 by symmetry.
- Yaw inertia: Σ (own + m (x² + y²)) = 0.259 kg m2, +1.0 % on 0.2566. The arms carry 2 x 4.239 x 0.137² = 0.159 of it; their y was solved to land on the target.
- Not matched: pitch and roll inertia. The cover seat is the shell's ledges from the CAD.
- Hook: a ball-bearing swivel sling hook of 1 t or more, e.g. Yoke G80 swivel eye sling hook with ball bearing (code 113308, aimsindustrial.com.au), Crosby L-3322B, Gunnebo LKNK or Certex X100; hung through two 250 mm chain legs to eyebolts on the top panel.
- Trim: weigh it (18.3 +-0.2 kg), hang it level, then run the spin-down test. A hook alone turns almost freely, so it carries a magnetic brake (drawing: a copper disc on the fixed eye, two N42 40 x 10 mm magnets on a bracket on the turning part, gap 4 to 12 mm).

## Swivel damping and spin-down test

Scene: viscous drag 0.0254 N m s/rad on 0.257 kg m2, time constant 10.1 s (`SWIVEL_DAMPING` in `task.py`, 0.0125, plus the simulator's default body damping, 0.0129; no joint friction). Real hook: the eddy-current brake in `standin/drawing.txt` (its drag grows with speed, like the scene's), tuned by its magnet gap. Test, before the first trial and after any move of the hook: hand-spin the hung stand-in to 45 +-5 deg/s, release, read its rate (camera at 30 fps or a phone gyroscope on the top panel) after 5 s and 15 s. Accept: rate(5 s)/rate(0) = 0.61 +-0.025 and rate(15 s)/rate(0) = 0.23 +-0.03 (time constant 9.3 to 11.0 s), three times in a row, the three time constants within 0.5 s of each other. A constant friction fails the 15 s reading (it would have stopped the body by then). Otherwise change the gap: narrower for more drag, wider for less. The band is checked in the scene: the demo scores 1.0 at both of its ends (time constants 9.3 and 11.0 s, `ASIMOV_SWIVEL_TAU=17.5` and `24.5`: seated at steps 663 and 660) as at 10.1 s (step 660), so a hook that passes reproduces the stated motion.

## Sources

- Asimov and Menlo are the maker's names, used for identification only.

- Asimov 1's URDF, meshes and mass properties: Menlo Research, https://github.com/menloresearch/asimov-1, licensed CERN-OHL-S-2.0. `upper_body.npz` is derived from them and stays under CERN-OHL-S-2.0.
- The back cover, part 100_04C (`ASV1_100_04C.STEP` in the same repository, CERN-OHL-S-2.0): `cover.npz` holds its mesh, as exported from the STEP, and colliders derived from it, and stays under CERN-OHL-S-2.0.
- Build steps 10.6, "Turn the robot around so we can now work on the back cover.", and 10.7, "Take part 100_04C and put on the back of part 100_01C.": Menlo Research's Asimov 1 build guide, https://docs.menlo.ai/asimov/1/build/assembly-steps/torso-module/battery-and-cover-installation/step-10.md
- The torso shell, part 100_01C (`ASV1_100_01C.STEP`, same repository, CERN-OHL-S-2.0): `seat.npz` and `blocks.stl` hold the ledge faces and pads derived from it (`dev/step_to_stl.py`, `dev/make_seat.py`) and stay under CERN-OHL-S-2.0.
- All other meshes are generated by `task.py` and `mechkit.py`.

## Licence

MIT, except `upper_body.npz`, `cover.npz`, `seat.npz` and `blocks.stl` (CERN-OHL-S-2.0, see Sources).
