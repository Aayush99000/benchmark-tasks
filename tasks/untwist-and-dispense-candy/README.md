# Untwist and dispense a wrapped candy

Untwist one end of the wrapper and let the candy fall free onto the tabletop. Leave it intact and clear of the wrapper.

The wrapper is the handling surface when opening a portion for someone else. The robot must control both the twist and the release of a loose, small object. It starts with one fully wrapped candy and no containers, markers, knives, or other loose objects.

## Setup and reset

Use a clean, dry, rigid tabletop 110 × 130 × 6 cm, with its top 76 cm above the floor. Mount the fixed YAM bases 61 cm apart. Place the center of the package 34 cm straight ahead of their midpoint. Point the two ends parallel to the line between the bases. Start with both arms in the ready pose and both grippers open.

Use a single round hard-boiled drop, 17 mm wide, about 14 mm high and 3.4 g. Wrap it in one approximately 128 × 67 mm, 45 µm strip of printed twist-retaining cellophane. Orient the overlapping seam down and the two pinched necks at least half a turn each. Replace the opened package with a fresh one of the same type after each trial. A separate full-duration hands-off control verifies that the wrapper retains the candy and neither neck unwinds 90° by itself. The graphics include a navy registration stripe across the print. [Exact scene coordinates and dimensions](setup.json)

The overhead camera, support and wrist cameras use the calibrated station configuration. The fixed oblique detail camera is used for judging. Its focal length and pose are in the runner. For a physical trial, include the two necks, the landing area and the final wrapper position in the video. The board media include the setup image.

## Scoring

- **0.0** if the wrapper tears, the sweet chips or breaks, the sweet contacts the floor or an unclean surface, or either hand touches it after it has fully cleared the wrapper. Also score zero when the untwist milestone is unmet. A subsequent correction does not erase a violation.
- **0.25** for holding a wrapper end and reducing its winding by at least 90° relative to the wrapper body for at least 0.25 seconds before the sweet exits. Track the navy stripe across the neck in the close video. Turning the whole package together contributes zero.
- **0.5** if the untwist milestone is met and the whole sweet finishes outside the wrapper with at least 2 mm clearance. Use a close final photo with a ruler.
- **1.0** if the preceding milestones are met and the sweet falls at least 3 cm from the wrapper onto the tabletop without being caught. It must rest there for the final two seconds with the film at least 2 mm away. Measure the vertical drop against a separate ruler in the camera calibration or measure it from the trace. Both grippers must be open at least 75 mm, with their pinch points at least 8 cm from the sweet center and paper for those two seconds. The arms may leave in any order.

Use the highest satisfied milestone. The physical judge watches the full clip for violations and scores separation from photos and video. The native runner uses film and candy geometry to observe separation and records robot link poses at 240 Hz for contact auditing. The scripted controller sends only precomputed joint targets, including its gripper commands.

## Run and score

The policy limit is 2,400 actions at 20 Hz. The joint script uses 600 actions, or 30 seconds of simulation. Native dynamics update at 240 Hz. Allocate several minutes of GPU time for path-traced video. Reset recreates the deterministic authored geometry. PhysX contact resolution after initialization can vary slightly, so fitting the recorded trial to its separately recorded idle control also checks the observed reset.

From this task directory, activate the Python environment for Isaac Lab Arena and the installed YAM station. See [ENVIRONMENT.json](ENVIRONMENT.json) for the versions and [YAM setup](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) for installation. Replace `<gpu>` with the physical NVIDIA device index on your host.

```bash
python native_study.py candy --variant printed-drop --gpu-index <gpu> --motion clear-wrapper --output evidence/candy-review
python native_study.py candy --variant printed-drop --gpu-index <gpu> --motion hold --state-only --steps 2400 --output evidence/candy-idle
python grade_candy_trial.py evidence/candy-review evidence/candy-idle
python native_study.py candy --variant printed-drop --gpu-index <gpu> --motion dream --output evidence/candy-dream
python grade_candy_trial.py evidence/candy-dream evidence/candy-idle
python audit_evidence.py evidence/candy-review --output evidence/candy-review/camera-audit.json
```

The Dream-1 run uses the installed Inspect Robots Dream policy and the SDK's private credential store. It reads the three standard YAM RGB views at 320 × 176 and the measured joints, with the instruction from `task.json`. Dream runs one 2,400-action episode at 20 Hz. The first 100 steps are held by the policy to let the wrapper settle, then eight rows from each model reply execute before the next request. Allow roughly 30–60 minutes for rendering and inference on an RTX A6000. The overhead and wrist feeds are the policy images. The fixed detail view is used for review.

The outputs include camera recordings, the reset and final images, generated USD assets, and complete measured traces. The score record contains the trial and control trace hashes, the measured hold duration, and contact clearances. The scorer can also read an independently filled schema 3 measurement record using `python scoring.py candy measurements.json`. Use `grade.json`'s `scorer_input` as the field layout.

The recorded Dream-1 trial completed 2,400 actions and scored 0.00. It made 288 inference requests in one episode. The model did not complete the qualifying held untwist.

## Fidelity and asset licenses

The sweet is a rigid convex hull underneath a smooth molded render surface. The print and glaze affect appearance. The sheet is a continuous deformable surface with friction, gravity and self-contact. The formed rest mesh approximates twist retention and springback. Fixed topology excludes tearing, and the rigid sweet excludes fracture. Physical tests must inspect wrapper integrity, any candy damage and contamination separately. The actual bending and friction parameters are estimates. Food-packaging literature documents twist-retaining film for hard candy. [Illustrated packaging reference](https://patents.google.com/patent/HK1023325B/en)

- The candy hull, rounded render mesh, wrapper and berry artwork are original procedural assets. The art is generated by [candy_finish.py](candy_finish.py) with no imported logo, artwork or font.
- The YAM robot geometry comes from [i2rt](https://github.com/i2rt-robotics/i2rt) under MIT. The installed `yam_arena` dependency supplies the model and [I2RT notice](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt). The table and camera stand are primitives from that installed station. The dependency is used in place and contains code with no published license.
- The judging camera helper adapts [TK-LGTDUX](https://github.com/robocurve/benchmark-tasks/pull/2190) under MIT. [License notice](LICENSE.pram-reference)

## Review context

ALOHA directly overlaps on pickup, two-tail pulling and peeling a candy wrapper. This draft also requires a fully separated sweet and a released tabletop outcome without touching the loose food. The pending sterile-pouch task already measures dropping contents without touching them. The candy uses a held untwist, and its gravity-release milestone overlaps with the pouch task. Both comparisons matter to the reviewer’s importance decision. [ALOHA paper](https://tonyzhaozh.github.io/aloha/aloha.pdf) · [TK-N5GU6U](https://tasks.robocurve.org/board?task=TK-N5GU6U) · [Robocurve review prompt](https://tasks.robocurve.org/review-prompt)
