"""Read the painted tray stripe, mug dot and pilot lamp in an overhead camera frame."""

import cv2
import numpy as np

PLAN_SIZE = 576
# Centers of the left and right 139 mm deck ticks in the uncaptioned plan image.
PLAN_TICKS = (61.5, 263.5, 483.5, 263.5)
PLAN_LAMP = (562, 463)


def lamp_lit(image, point):
    x, y = point
    patch = image[y-2:y+3, x-2:x+3]
    # The middle channel is green in both RGB and BGR.
    a, g, b = np.median(patch, axis=(0, 1))
    return bool(g >= 150 and g > max(a, b)*1.4)


def centroid(mask):
    count, _, stats, centers = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    if count < 2:
        return None
    largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    return centers[largest] if stats[largest, cv2.CC_STAT_AREA] >= 10 else None


class PlanMarks:
    def __init__(self, width, height, ticks):
        left, right = np.asarray(ticks).reshape(2, 2)
        self.center = (left + right) / 2
        px_mm = np.linalg.norm(right - left) / 278
        if px_mm <= 0:
            raise ValueError("The two opposite ticks must be distinct.")
        self.x0, self.y0 = np.maximum(0, np.floor(self.center - 133*px_mm).astype(int))
        self.x1, self.y1 = np.minimum((width, height), np.ceil(self.center + 133*px_mm).astype(int))
        if self.x1 <= self.x0 or self.y1 <= self.y0:
            raise ValueError("The calibrated turntable lies outside the recording.")
        y, x = np.ogrid[self.y0:self.y1, self.x0:self.x1]
        radius = np.hypot(x-self.center[0], y-self.center[1]) / px_mm
        self.tray_annulus = (radius > 90) & (radius < 133)
        self.mug_annulus = (radius > 25) & (radius < 85)

    def bearings(self, image, *, bgr=False):
        crop = image[self.y0:self.y1, self.x0:self.x1]
        h, s, v = cv2.split(cv2.cvtColor(crop, cv2.COLOR_BGR2HSV if bgr else cv2.COLOR_RGB2HSV))
        blue = (h >= 103) & (h <= 128) & (s > 125) & (v > 120) & self.tray_annulus
        yellow = (h >= 22) & (h <= 37) & (s > 130) & (v > 155) & self.mug_annulus
        result = []
        for mask in (blue, yellow):
            xy = centroid(mask)
            if xy is None:
                return None
            dx, dy = xy + (self.x0, self.y0) - self.center
            result.append(float(np.degrees(np.arctan2(-dy, dx))))
        return result
