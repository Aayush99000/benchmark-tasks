"""Shared physical latch and sensor delivery checks for every controller."""
import math
from latch_geometry import InsertionGate

class LatchMonitor:
    def __init__(self):
        self.gate = InsertionGate()
        self.attached = False

    def update(self, offset, angle_rad, speed, spring_displacement_m):
        # A previously seated shoe leaving the receiver invalidates entry history.
        inside = (sum(float(v)**2 for v in offset) <= .006**2
                  and abs(angle_rad) <= math.radians(5))
        if self.gate.latched and not inside:
            self.gate = InsertionGate()
        seated = self.gate.update(offset, angle_rad, speed)
        # Retraction removes locking immediately. Releasing the button while still
        # seated can re-engage mechanically, without an artificial reinsertion.
        self.attached = bool(seated and inside and
                             -.0005 <= spring_displacement_m < .0012)
        return self.attached

class FrameDeliveryMonitor:
    def __init__(self):
        self.previous_frame = None

    def update(self, frame_id, valid_pixels):
        frame_id = int(frame_id)
        fresh = self.previous_frame is not None and frame_id > self.previous_frame
        self.previous_frame = frame_id
        return bool(fresh and valid_pixels)

def sensor_frame_id(sensor):
    frame = sensor.frame
    frame = frame.torch if hasattr(frame, 'torch') else frame
    return int(frame[0].item())
