# Prepare a shower chair for use

The empty chair starts with one leg one adjustment hole shorter than the other three. Preparing it for its next user requires noticing the unequal support, correcting the telescoping mechanism, and leaving the chair independently supported. The instruction is only: **Prepare the shower chair for use.**

The initial fault threatens the stability of a subsequent sitter. No person is present during adjustment and the simulation does not claim to model injury. Deliberately lifting or turning the empty chair is allowed. A coordinated one-arm recovery is also valid; two arms are not imposed by the scorer.

## Run and score

Use the Python environment containing `yam_arena` and `inspect-robots`:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/reset-shower-chair-short-leg/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Prepare the shower chair for use." \
  --scorer task_score --max-steps 600 --store-frames \
  --log-dir ~/yam-eval/shower-chair-demo
```

For the model trial, use `--policy dream --max-steps 2400` and camera dimensions `320` by `176`. Trial output reports `task_score`. The layout is fixed with `object_jitter=0`; no seed-dependent randomization is used. The scripted plan uses fixed waypoints and never reads scorer state. A 600-step trial allows 30 simulated seconds; the model limit is 120 simulated seconds. The two fresh full demos each used 362 control steps (18.1 simulated seconds). Their measured wall times are listed below.

CPU scorer regression checks:

```bash
python tasks/reset-shower-chair-short-leg/check_scoring.py
```

## Mechanism and physical reconstruction

The chair is a full-scale, explicitly dimensioned reconstruction of the familiar push-button bath-bench mechanism, not an exact digital replica of a certified product. The [Drive Medical backless bench product sheet](https://shop.drivemedical.com/us/en/c/deluxe-aluminum-shower-bench-without-back/p/RTL12203KDR/downloadPdf) documents an aluminium bench, approximately 49 by 29 cm seat, one-inch leg adjustments and a 5 lb product variant. The simulation uses a 50 by 30 by 4 cm seat and 2.268 kg total mass. Its 46 by 34 cm foot-centre footprint, 10-degree splay and tube dimensions are explicitly specified reconstruction dimensions, not measurements attributed to that product.

[Medline's bath-bench assembly manual](https://www.medline.com/media/catalog/Docs/MKT/MPL_MDS89740RWA%28H%29_Bath%20Bench_Manual.pdf) documents the push-button/extension mechanism and requires matching leg settings with buttons fully protruding. [Medline's related shower-stool user guide](https://www.medline.com/media/catalog/Docs/MKT/MAN_G2-203FRX1_Trilingual_19W2439682.pdf) requires all legs to contact the floor simultaneously and lock securely before use. These sources establish the safety requirement and mechanism, not the simulated spring constants or product certification.

`task.py` provides all geometry and starting transforms. Each 22 mm aluminium inner tube slides inside a 25 mm outer tube. Eight indexed holes are spaced 25.4 mm; the stated nominal settings are 14–21 inches. A 6 mm button protrudes 4 mm and has 5.5 mm travel. Its spring is estimated at 2,000 N/m: 8 N at 4 mm depression. This is a disclosed engineering approximation, not a measured manufacturer's force. Each inner tube/tip assembly is 0.10 kg, each button 0.01 kg, and the frame/seat is 1.828 kg. Frame COM and inertia are engineering estimates in the source. Uniform static/dynamic friction is 1.2/1.0, so wet-floor sliding is not represented.

Pressing a button at least 3 mm releases that leg's axial joint. The inner tube then moves under actual gravity/contact forces. When an uncovered hole passes the button, a joint drive models the engaged pin; there is no simulated metal shear or pin fracture. Button coverage is inferred geometrically from the gripper pads. The seat is rigid; drainage and handle recesses are cosmetic. Rebuild a test fixture from the published dimensions and calibrate its button force and leg sliding resistance before claiming physical transfer. Do not seat a person on an uncertified reconstructed fixture.

The station table represents the supporting floor in robot-relative coordinates. Its surface is at z=0.7475 m. The real chair remains full size. The fixed camera is at `(-0.30, -0.60, 1.50)` m, aimed at `(0.45, 0.0, 0.98)` m. The camera body and stand use this same pose; both model and demo receive the same views.

## Scoring

All equal leg settings in the supported range are accepted; 17 inches is not a hidden target.

- **1.0:** all four buttons out and legs at one common indexed setting; seat tilt at most 1 degree; each tip gap between -2 mm (solver penetration tolerance) and +1 mm; feet horizontally inside the tabletop; speed below 5 mm/s and angular speed below 0.05 rad/s; all sampled finger points more than 3 cm clear. Every condition must persist continuously for 3 seconds.
- **0.5:** all four legs have a common locked setting and their feet remain horizontally over the tabletop and no more than 2 mm below it, but the full condition is not yet satisfied, for example while still held.
- **0:** every other outcome, including the unchanged initial layout, an unlatched leg or unequal settings.

A person can check protruding buttons, count indexed holes, use a level and a 1 mm feeler gauge, and observe the released chair for three seconds. No force sensor is required for scoring. `Trial.step()` updates once per control step; `score()` and `success()` only read the result. Reset restores the original leg settings, buttons, starting pose and timer. Temporary tilting or shortening of an unoccupied chair is not treated as fictitious personal injury.

## Assets and helpers

The chair geometry and joints are authored structural/mechanical parts. The two distractors are NVIDIA SimReady props, downloaded at first run from pinned revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [cleaning_soapDispenser_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_soapDispenser_a01): mass set to 0.45 kg; authored bodies merged where needed.
- [cleaning_sponge_b01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_sponge_b01): mass set to 0.03 kg.

© NVIDIA, CC-BY 4.0. No textures or meshes are bundled; the loader fetches the pinned licensed packages. `simready.py` is adapted from the open-source `cutters-off-pegboard` loader. `mechkit.py` derives from the shared mechanism kit used by `heat-shrink-lug`, with joint-frame rotation, frame COM, and explicit latch-drive gain helpers added for the splayed legs.

## Validation status

Two fresh full demonstrations on the final simulation source both scored **1.0**. The CPU regression checks passed for timing, pure score reads, all eight equal heights, release, unlatching, support, bounds and reset.

| Trial | Score | Control steps | Wall time |
|---|---:|---:|---:|
| demo1 | 1.0 | 362 | 96 s |
| demo2 | 1.0 | 362 | 159 s |
| nothing | 0.0 | 61 | 42 s |
| pressloaded | 0.0 | 236 | 57 s |
| hold | 0.5 | 496 | 266 s |
| liftonly | 0.0 | 282 | 200 s |
| squeezelift | 1.0 | 235 | 100 s |

The one-arm squeeze/lift genuinely restores the support and is correctly accepted. Merely holding the corrected chair earns partial credit; lifting without resetting the leg and pressing the loaded button do not complete the task. The loaded-button test physically depressed the button 5.5 mm and shortened the leg 50.8 mm before re-latching, verified by a print-only validation witness. Contact sheets were inspected for every run. Trial duration varies with GPU contention. No model score is claimed yet.

Dream-1 completed its full 2,400-step budget on the same frozen source and scored **0.0**. The model recording and final state were visually inspected.
