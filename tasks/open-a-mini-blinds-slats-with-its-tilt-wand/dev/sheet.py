"""Dev only: a contact sheet of stored camera frames. python dev/sheet.py <frames dir> <out.png> <step> [<step> ...]
Each row is one control step; the columns are top_cam, left_cam and right_cam. Created 11_16_26."""

import glob
import sys

import numpy as np
from PIL import Image, ImageDraw

src, out, steps = sys.argv[1], sys.argv[2], [int(s) for s in sys.argv[3:]]
rows = []
for s in steps:
    tiles = []
    for cam in ("top_cam", "left_cam", "right_cam"):
        hits = glob.glob(f"{src}/**/*{cam}_{s:06d}.npy", recursive=True)
        img = Image.fromarray(np.load(hits[0]).astype(np.uint8)) if hits else Image.new("RGB", (320, 176))
        ImageDraw.Draw(img).text((4, 4), f"{cam} {s}", fill=(255, 255, 0))
        tiles.append(np.asarray(img.convert("RGB")))
    rows.append(np.concatenate(tiles, axis=1))
Image.fromarray(np.concatenate(rows, axis=0)).save(out)
