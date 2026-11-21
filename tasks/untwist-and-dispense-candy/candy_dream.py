"""Run the official Inspect Robots Dream policy against the candy station."""
import numpy as np

from candy_submission import INSTRUCTION

ROWS_PER_REQUEST = 8
SETTLE_ACTIONS = 100


class CandyDream:
    """Read the standard three YAM camera views and the measured joints."""

    def __init__(self, base, grippers):
        from inspect_robots_dream import DreamPolicy
        from yam_arena.inspect_embodiment import YamArenaEmbodiment

        self.policy = DreamPolicy(rows_per_request=ROWS_PER_REQUEST, preroll_steps=SETTLE_ACTIONS)
        self.embodiment = YamArenaEmbodiment(
            scene='station', cam_width=320, cam_height=176, control_hz=20, object_jitter=0.
        )
        self.embodiment._base = base
        self.embodiment._ids = {
            side + '_robot': (arm_ids, finger_ids)
            for side, (_, arm_ids, finger_ids) in grippers.items()
        }
        self.embodiment._instruction = INSTRUCTION

    def start(self):
        from inspect_robots import Scene

        self.policy.bind(self.embodiment.info)
        self.policy.reset(Scene(id='untwist-and-dispense-candy', instruction=INSTRUCTION))

    def action(self, previous_step):
        self.embodiment._t = previous_step / 20
        chunk = self.policy.act(self.embodiment._observation())
        if len(chunk.actions) != 1:
            raise RuntimeError('Dream must return one YAM target per control step')
        action = np.asarray(chunk.actions[0].data, dtype=np.float32)
        if action.shape != (14,) or not np.isfinite(action).all():
            raise RuntimeError('Dream returned a malformed target')
        return np.clip(action, self.embodiment._low, self.embodiment._high)

    def transcript(self):
        result = self.policy.transcript()
        result.pop('server_url', None)
        for request in result['requests']:
            request.pop('session_id', None)
        return result

    def close(self):
        self.policy.close()
