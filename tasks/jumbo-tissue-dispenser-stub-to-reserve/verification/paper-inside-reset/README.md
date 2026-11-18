# Constructed adverse paper reset control

This contains the exact v3 harness and required inputs used for the native400-control adverse reset. `reset_control.py` SHA256 is `a84df199854b0a7ad9849a863dc013a7161cc3372780b0d78949db20e5367a39`. The unchanged canonical entry SHA256 is `64848fea2f1be48445eee7c14fd09590af23da48bb486968707e3aad935a4649`.

The original task reset, paper authoring/default/rest geometry,33 root attachments and node-label assertions run first. Still inside the reset wrapper, the harness constructs five rigid poses/zero velocities and the current paper state once using public native APIs. It then runs400 ordinary stock READY controls with no control-phase state writers, added constraints, material changes or scorer changes. This changes the initial state; it is not an ordinary robot error policy from the task's normal start.

The observed normal completion scored0.8. At controls200,250,300,400, both seats, cover lock, selector-left, rigid settling, release geometry and paper/robot clearance passed. Paper topology passed; paper geometry was inaccessible, root attachment was also11.2–12.3mm outside tolerance, and nodes were moving. This is a combined native adverse-state rejection, **not a native inside-only or isolated negative**. It does not imply fracture. See `native-adverse-audit.json`; its strict `valid_isolated_negative` remains false. `CPU-RESULT.json` is the exact earlier construction audit; its `NATIVE_PENDING` label predates the native run and is not the final result.

## Reproduce

Use Linux with a CUDA GPU, the same Isaac Lab/Arena and YAM installation used to run the task, and the original-callback no-camera runner included here. Python dependencies include `inspect_robots`, `yam_arena`, Isaac Sim/PhysX, Torch, NumPy, SciPy and the local task imports. The environment must expose `libcuda.so.1`. The native run used20Hz controls,1/240s physics, seed0, and a single environment. No model inference or media is produced.

From the repository root, set the environment script and the **same shared GPU lock used by other simulations on the host**. Output paths must be outside this verification directory and outside the submitted task. Replace the two `/path/to/...` placeholders with actual host paths:

```bash
export YAM_ENV=/path/to/IsaacLab-Arena/yam.env
export GPU_LOCK=/path/to/shared/gpu.lock
source "$YAM_ENV"
export TASK_DIR="$(realpath tasks/jumbo-tissue-dispenser-stub-to-reserve)"
export VERIFY_DIR="$TASK_DIR/verification/paper-inside-reset"
export RUN_DIR="$(mktemp -d "${TMPDIR:-/tmp}/paper-inside-reset.XXXXXX")"
export ROBOCURVE_CANONICAL_TASK_DIR="$RUN_DIR/canonical"
export ROBOCURVE_TRACE_DIR="$RUN_DIR/trace"
export DEMO_VARIANT=reference
unset ROBOCURVE_NEGATIVE_ACTION_FILE ROBOCURVE_NEGATIVE_ACTION_SHA256
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1
python "$VERIFY_DIR/prepare_reproduction.py" \
  --canonical "$TASK_DIR" --out "$ROBOCURVE_CANONICAL_TASK_DIR"
flock --no-fork "$GPU_LOCK" python -u "$VERIFY_DIR/robot_without_cameras_v2.py" \
  --task "$VERIFY_DIR/reset_control.py" --out "$RUN_DIR/native" \
  --steps 400 --visualizer none --livestream 0 --device cuda:0
```

The harness requires the exact29 frozen file hashes, including publication metadata. `prepare_reproduction.py` copies the runtime files from the submitted task and restores only the frozen README, task and materials snapshots in a disposable copy; it never changes the submitted task. This accommodates later score/media edits without changing the executed harness or physics. Any runtime/plan hash mismatch fails closed.

Require a `native/completed.json` marker,400 controls, normal terminal and unchanged source. Runner return code0 alone is insufficient because initialization exceptions are written to a sibling error JSON. Inspect `constructed-control-observations.jsonl` at every saved late control>=200, the paper node snapshots, root-error/geometry reasons, actual hardware/release conditions and write counters. Do not assert an expected score or relabel this case as isolated. Native resource containment should follow the host's run policy; the recorded run used a5600MiB memory cap,5200MiB high watermark, no cgroup swap and a900s runtime limit.

No absolute personal paths, host addresses, tokens, large trajectories or videos are bundled. `frozen-metadata/` preserves historical input bytes solely for exact reproduction; the old pending score/media and controller text does not replace the task's current submission metadata or materials note.
