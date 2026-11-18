"""Record a real trial of shielded-pin, then pull out what the rater needs.

  python record_trial.py record --port /dev/ttyACM0 --camera 0 --out trial01   # Ctrl-C to stop
  python record_trial.py sheet trial01                                          # the onlooker frame at each press

record: reads the Pico's key lines (pinpad_logger.py) and the onlooker camera, stamping both with this computer's
clock; writes trial01/keys.csv (host time, key, down|up) and trial01/onlooker/<host time>.jpg.
sheet: for every "down" in keys.csv, copies the onlooker frame nearest in time to trial01/presses/<n>_<key>.jpg and
prints the press list. The rater then looks at each press's frame (README, "Scoring a real trial").

Needs: pip install pyserial opencv-python.
"""

import argparse
import csv
import shutil
import threading
import time
from pathlib import Path


def record(port: str, camera: int, out: Path) -> None:
    import cv2
    import serial

    (out / "onlooker").mkdir(parents=True, exist_ok=True)
    stop = threading.Event()

    def keys() -> None:
        with serial.Serial(port, 115200, timeout=0.1) as s, open(out / "keys.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["host_time", "key", "event"])
            while not stop.is_set():
                line = s.readline().decode(errors="replace").strip()
                if line.count(",") == 2:
                    _, key, event = line.split(",")
                    w.writerow([f"{time.time():.4f}", key, event])
                    f.flush()

    t = threading.Thread(target=keys, daemon=True)
    t.start()
    cam = cv2.VideoCapture(camera)
    try:
        while True:
            ok, frame = cam.read()
            if ok:
                cv2.imwrite(str(out / "onlooker" / f"{time.time():.4f}.jpg"), frame)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        cam.release()
        t.join(1.0)


def sheet(out: Path) -> None:
    frames = sorted((float(p.stem), p) for p in (out / "onlooker").glob("*.jpg"))
    (out / "presses").mkdir(exist_ok=True)
    with open(out / "keys.csv") as f:
        downs = [(float(r["host_time"]), r["key"]) for r in csv.DictReader(f) if r["event"] == "down"]
    for n, (t, key) in enumerate(downs, 1):
        ft, p = min(frames, key=lambda fp: abs(fp[0] - t))
        shutil.copy(p, out / "presses" / f"{n:02d}_{key}.jpg")
        print(f"press {n}: {key} at {t:.3f} (onlooker frame {1000 * (ft - t):+.0f} ms)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("--port", required=True)
    r.add_argument("--camera", type=int, default=0)
    r.add_argument("--out", type=Path, required=True)
    s = sub.add_parser("sheet")
    s.add_argument("out", type=Path)
    a = ap.parse_args()
    record(a.port, a.camera, a.out) if a.cmd == "record" else sheet(a.out)
