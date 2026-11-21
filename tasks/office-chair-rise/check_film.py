"""Count black or flat frames in each camera's film and in demo.mp4, and check that the MP4 has its moov atom at the front (faststart).
    python check_film.py DIR      (DIR holds top_cam.mp4, left_cam.mp4, right_cam.mp4, demo.mp4)"""
import subprocess
import sys
from pathlib import Path

import numpy as np

import demokit

d = Path(sys.argv[1])
for name, size in (("top_cam", (640, 360)), ("left_cam", (640, 360)), ("right_cam", (640, 360)), ("demo", (1280, 480))):
    f = d / f"{name}.mp4"
    raw = subprocess.run([demokit._ffmpeg(), "-loglevel", "error", "-i", str(f), "-f", "rawvideo", "-pix_fmt", "rgb24", "-"], capture_output=True).stdout
    n = len(raw) // (size[0] * size[1] * 3)
    fr = np.frombuffer(raw, np.uint8)[: n * size[0] * size[1] * 3].reshape(n, -1)
    stds = fr.reshape(n, -1).std(axis=1)
    mean = fr.mean(axis=1)
    head = f.read_bytes()[:4096]
    print(f"{name}: {n} frames, black/flat (std<12) {int((stds < demokit.BLANK_STD).sum())}, darkest mean {mean.min():.1f}, first frame std {stds[0]:.1f}, moov before mdat: {head.find(b'moov') != -1 and (head.find(b'mdat') == -1 or head.find(b'moov') < head.find(b'mdat'))}")
