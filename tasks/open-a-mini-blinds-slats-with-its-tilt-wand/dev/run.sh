#!/bin/bash
# Dev only: one simulator trial of the scripted demo. dev/run.sh <label> [max_steps]; variants and tuning via BLIND_* env vars.
# Created 11_17_26.
set -u
cd "$(dirname "$0")/.."
source ~/IsaacLab-Arena/yam.env
[[ -e /lib/aarch64-linux-gnu/libgomp.so.1 ]] && export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1 PYTORCH_JIT=0  # aarch64 hosts only
LOCK=${SIM_LOCK:-$HOME/sim.lock}
OUT=~/yam-eval/blind-$1
mkdir -p "$OUT"
flock "$LOCK" inspect-robots run --embodiment yam_mini_blind -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=${CAM_W:-320} -E cam_height=${CAM_H:-176} --policy yam_scripted --no-prompt \
  --instruction "Open the blind: turn the wand until the slats are level." \
  --scorer task_score --max-steps "${2:-2400}" --seed 0 --store-frames --log-dir "$OUT" > "$OUT/stdout.log" 2>&1
echo "exit $?" >> "$OUT/stdout.log"
grep -E "task_score|Traceback|Error" "$OUT/stdout.log" | tail -5
