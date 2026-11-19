"""Encode every saved 5 Hz diagnostic frame at real simulated time."""
import argparse,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);p.add_argument('--output',default='development_run.mp4');a=p.parse_args()
frames=sorted((a.trial/'frames').glob('*.png'))
if not frames:raise SystemExit('No captured frames')
indices=[int(p.stem) for p in frames]
if any(b-a!=4 for a,b in zip(indices,indices[1:])):raise SystemExit('Missing frames; do not silently compress time')
subprocess.run(['ffmpeg','-y','-framerate','5','-pattern_type','glob','-i',str(a.trial/'frames'/'*.png'),'-c:v','libx264','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(a.trial/a.output)],check=True)
print(a.trial/a.output)
