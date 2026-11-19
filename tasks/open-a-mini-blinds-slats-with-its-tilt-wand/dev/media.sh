#!/bin/bash
# Dev only: one demo trial at 640 x 352, its camera videos as H.264 MP4 and the setup screenshot, in ~/yam-eval/blind-media/
# Created 11_16_26. Run inside the task folder (cloud host or local).
set -u
cd "$(dirname "$0")/.."
source ~/IsaacLab-Arena/yam.env
CAM_W=640 CAM_H=352 bash dev/run.sh media 2400
OUT=~/yam-eval/blind-media
log=$(ls -t "$OUT"/*.json | head -1)
inspect-robots video "$log"
mkdir -p "$OUT/mp4"
FFMPEG=$(command -v ffmpeg || python -c "import imageio_ffmpeg as i; print(i.get_ffmpeg_exe())")
for f in $(find "$OUT/frames" -name "*.mp4"); do
  "$FFMPEG" -loglevel error -y -i "$f" -c:v libx264 -pix_fmt yuv420p -movflags +faststart "$OUT/mp4/$(basename "$f")"
done
python - "$OUT" <<'PY'
import glob, sys
import numpy as np
from PIL import Image
out = sys.argv[1]
f = sorted(glob.glob(f"{out}/frames/**/*top_cam_000000.npy", recursive=True))[0]
Image.fromarray(np.load(f).astype(np.uint8)).save(f"{out}/mp4/setup.png")
PY
ls -la "$OUT/mp4"
