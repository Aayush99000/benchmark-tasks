"""Video rubric for the tray stripe and mug dot. Bearings are degrees CCW from above."""

from collections import deque

import numpy as np

HZ = 20
WINDOW = 10              # ten frame intervals at 20 fps
MIN_WINDOW_DEG = 4.0
MAX_BACKTRACK_DEG = 4.0
MAX_SLIP_DEG = 8.0


class LoadedTurn:
    def __init__(self):
        self.best_deg = 0.0
        self.resets = 0
        self.recent = deque(maxlen=WINDOW + 1)
        self.clear()

    def clear(self, bearings=None):
        self.previous = bearings
        self.travel = np.zeros(2)
        self.peak = np.zeros(2)
        self.recent.clear()
        if bearings is not None:
            self.recent.append(self.travel.copy())

    def sample(self, tray_deg, mug_deg, eligible):
        bearings = np.array([tray_deg, mug_deg])
        eligible = eligible and np.isfinite(bearings).all()
        if not eligible or self.previous is None:
            if self.previous is not None:
                self.resets += 1
            self.clear(bearings if eligible else None)
            return False

        increment = (bearings - self.previous + 180) % 360 - 180
        travel = self.travel + increment
        stalled = len(self.recent) >= WINDOW and np.any(travel - self.recent[-WINDOW] < MIN_WINDOW_DEG)
        if (np.any(np.abs(increment) >= 90) or stalled
                or np.any(self.peak - travel > MAX_BACKTRACK_DEG)
                or abs(travel[0] - travel[1]) > MAX_SLIP_DEG):
            self.resets += 1
            self.clear(bearings)
            return False

        self.travel = travel
        self.peak = np.maximum(self.peak, travel)
        self.previous = bearings
        self.recent.append(travel.copy())
        self.best_deg = max(self.best_deg, float(travel.min()))
        # Signed unwrapped travel crosses all four start-relative quarter turns
        # in order. An unsampled quadrant is rejected above.
        return bool(travel.min() >= 360 and travel.max() <= 360 + MAX_SLIP_DEG)
