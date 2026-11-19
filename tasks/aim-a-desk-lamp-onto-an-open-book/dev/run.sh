#!/bin/bash
# Dev only: one simulator trial of the scripted demo. dev/run.sh <label> [max_steps]; variants and tuning via LAMP_* env vars.
# Created 11_16_26.
set -u
cd "$(dirname "$0")/.."
source ~/IsaacLab-Arena/yam.env
[[ -e /lib/aarch64-linux-gnu/libgomp.so.1 ]] && export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1 PYTORCH_JIT=0  # aarch64 hosts only
LOCK=${SIM_LOCK:-/tmp/desk-lamp-sim.lock}
OUT=~/yam-eval/lamp-$1
mkdir -p "$OUT"
flock "$LOCK" inspect-robots run --embodiment yam_desk_lamp -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=${CAM_W:-320} -E cam_height=${CAM_H:-176} --policy yam_scripted -P hold_s=${HOLD:-3} --no-prompt \
  --instruction "Light the notebook for reading: aim the lamp at the red dot, with the shade at least 30 cm above the desk." \
  --scorer task_score --max-steps "${2:-2400}" --seed 0 --store-frames --log-dir "$OUT" > "$OUT/stdout.log" 2>&1
echo "exit $?" >> "$OUT/stdout.log"
grep -E "task_score|Traceback|Error" "$OUT/stdout.log" | tail -5
