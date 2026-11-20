# Verification

All runs use this folder's code as committed, the fixed start layout (`object_jitter=0`), 20 Hz control and the
2,400-step limit. Simulator: Isaac Sim 6.1, Isaac Lab 3.0, Isaac Lab-Arena 0.3.1, yam_arena embodiment; one
RTX 4080 (16 GB).

## Offline tests

```
python -m pytest tests -q
16 passed
```

They check: the task file loads; real sizes (1-1/2 in pipe, 42 in top rail, 21 in midrail, 4 in column); the
hook's bowl takes the guardrail pipe and the hook fits the gripper; `judge()` on hand-placed poses (the start
pose: 0; on the D-ring, shut, locked, released and still: 1.0; on the D-ring with the gate open, the lock pressed,
a gripper close or the hook moving: 0.5; beside the D-ring without its bar in the loop: 0; round each guardrail
post and rail: 0; round both: 0); the lanyard reaches the anchor with slack; the stiff pull past full length
points toward the worker and is capped; the webbing model keeps its length and stays on the deck while the hook is
carried; the numbers in the scoring text match the code.

## Simulated runs

| Run | Command | Score | Steps | End state |
|---|---|---|---|---|
| Dream-1 | `inspect-robots run ... --policy dream --scorer task_score --max-steps 2400` | 0 | 2,400 | hook never closed round anything (run limit reached) |
| Full demo, recorded | `python demo.py task.py OUT` | 1.0 | 754 | on the D-ring, gate shut, lock out, grippers > 3 cm away, still |
| Guardrail | `DEMO_VARIANT=rail python demo.py task.py OUT --no-record` | 0 | 491 | shut and locked round the middle post, released, still |
| Hold | `DEMO_VARIANT=hold python demo.py task.py OUT --no-record` | 0.5 | 838 | on the D-ring, gripper still squeezing the gate open |
| Do nothing | `DEMO_VARIANT=nothing python demo.py task.py OUT --no-record` | 0 | 61 | hook untouched on the deck |
| Do nothing, through the runner | `inspect-robots run ... --policy yam_scripted --scorer task_score` | 0 | 61 | as above (the empty `demo()` stops the policy) |

Demo timeline (steps): right arm squeezes the handle and the gate opens 25 degrees (125); lifts and carries the
hook (155 to 261); lays it down mid-deck and lets go, the gate shuts and locks (261 to 300); left arm squeezes it
open (477), lifts it to the anchor's height (517), brings it beside the ring's tip (597), past the gate (647) and
home into the bowl (697, score 0.5 while held); lets go (713), the gate shuts and locks round the ring; the hook
settles on the ring within about 1 s, the lanyard keeping at least 4 cm of slack; the trial ends at 754 once full
score has held for 1 s. Full score needs every gripper at least 3 cm from the hook, so both were clear for that
whole last second; at the end the nearest gripper point was 10.5 cm from the hook (the left arm is still moving
back toward its ready pose in the last frames).

The scripted demo reads only the hook's pose and the arms' joint angles while it runs. Nothing is teleported and
the scene is not edited. Largest inverse-kinematics shortfall: 0.7 mm (left arm), 5.2 mm (right arm, during the
carry).
