# Pull, tuck and cut: wrap a watermelon half in cling film

**Instruction:** Use both arms to pull cling film from the dispenser, cover the watermelon half and tuck the film underneath, then slide the cutter across and release the wrapped fruit.

**Category: Impact on jobs.** [O*NET Food Preparation Workers](https://www.onetonline.org/link/summary/35-2021.00) lists portioning/wrapping food among regular duties; RoboCurve reviewer prompt v22 lists 0.9 million workers. The task measures continuous-film handling, coverage over rounded produce and retained underside wrapping before cutting. Gift wrapping precreased paper or winding tape around a pipe tests a different manipulation.

`EQUIPMENT.md` supplies all scene objects, dimensions, materials, coordinates, the four fruit rests, a defined guarded slide-cutter/roll-holder assembly, cameras and reset tolerances. This is a procedural fixture design; no particular manufactured dispenser or physically built station is claimed. `SCORING.md` is the complete v2 score and ruler/photo procedure.

| Primary score | Cumulative outcome |
|---:|---|
| 0.2 | Both-edge draw; 300–650 mm of material including the initial75 mm. |
| 0.5 | At least 95% coverage of the cut-face scoring disk. |
| 0.75 | Two connected underside patches, each>=200 mm² in side view and>=15 mm wide. |
| 1.0 | Full cut, film released, robot>=20 mm clear, all conditions retained>=1 s. |

No qualifying draw or a latched failure gives 0. Cut-before-tuck, torn film, fruit drop/displacement/tip and invalid numeric states are explicitly covered. Version2 uses physical patch area rather than mesh-node counts. Neither controller labels nor a cutter endpoint alone award success.

## Implementation and approximation limits

Native YAM rigid dynamics, a rigid 0.8 kg fruit half and native passive sliding cutter are coupled to reduced position-based film. Pinches depend on measured gripper/edge contact. Film payout responds to extension; a roll boundary replaces a layered spindle. Film contact/tack is local and uncalibrated; film has no self-collision or force feedback to the arms/fruit. The cutter sweep releases supply constraints. No controller teleports the fruit or assigns a successful wrap geometry. These are declared simulation approximations, not hardware calibration.

Physics runs at 240 Hz, control at 20 Hz; the C++ contact kernel accelerates the same surrogate computations. Policy cameras remain top/left/right; the overview is diagnostic. The installed adapter revision is 0059d02a1a91bc6834c1d0846022891df0c78730.

## Run

```bash
python build_contact_kernel.py
/path/to/inspect/python launch.py demo --adapter /path/to/yam-adapter --out runs/v2_demo_001
/path/to/inspect/python launch.py model --adapter /path/to/yam-adapter --out runs/v2_dream1_001
python -m unittest test_contract test_tuck_measurement test_contact_kernel -v
python verify_execution.py runs/v2_demo_001
```

The chosen Python environment must contain Isaac/Arena, Inspect Robots and Dream-1; `inspect-robots` must be beside that Python executable. Model mode uses Dream-1, seed 0, one epoch, 3,600 controls at 20 Hz, no jitter and stored frames. Maximum simulated duration is 180 seconds; the recorded demo took 66.75 simulated seconds, and the model run took about 22 minutes wall time on an RTX 4090 host with shared workload. Keep credentials in the existing environment. Use the identical frozen source for demo and model runs and new output directories; retain actual failures and scores.

**Evidence:** Fresh v2 `evidence/v2_demo_001` scored **1.0 at1,335/3,600 actions**, with99.8946% coverage and4.05 s released retention. Independent source, geometry and video audits pass; final independent reviewer verdict is **ACCEPT** (round10). The authenticated v2 Dream-1 run completed normally at0.0/1.0 after3,600 actions (seed0, max_steps). Historical run014 remains explicitly v1. See `INDEPENDENT_REVIEW.json`.

## Assets and licences

All task-specific film, fruit, seeds, rests and dispenser/cutter geometry is procedural source. Robot meshes are external installed dependencies. [i2rt](https://github.com/i2rt-robotics/i2rt) revision 120c3c81400171174604e503943f8d1ebc891058 is MIT. The [YAM adapter/station](https://github.com/kairosiann/yam-isaac-lab-arena) revision above declares no licence; it is not redistributed. IsaacLab Arena revision 3b0f10ce2df6c25649afbbf5aed0b1cd27d5cd66 and Isaac Sim are operator installations under their upstream licences/terms. Preserve these pins and the exact runtime versions with each new run.

Current model evidence: `evidence/v2_model_002/model_eval.json.gz` and `score_trace.json`; the derived-log path anonymisation is disclosed in `EVIDENCE_PROVENANCE.md`. Full demonstration and model camera media are uploaded to the task board using the ID in `task.json`.

The model EvalLog uses lossless gzip compression. Read it with `gzip -dc evidence/v2_model_002/model_eval.json.gz` or Python `gzip.open`.
