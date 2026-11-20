# Put the jester back in a jack-in-the-box: hold it down, bring the lid over it, slip the hand out as the lid takes it, and press the lid shut until the catch holds

Push the jester into the box, close the lid until its catch holds, then withdraw both hands without moving or tipping the toy.

## Instruction

> The jack-in-the-box has popped open. Push the jester back down into the box and shut the lid over it so that the catch holds the lid shut, then let go and move both arms away. Leave the box where it is.

The instruction above is the exact string used in the recorded model run.

## Start scene

Schylling Jester Jack in the Box: 133 × 133 × 140 mm closed, 454 g. Box centre (360, 0) mm. Lid initially 105° open. Head top 254 mm above the table; rim at 135 mm. The box is loose on tape marks.

Coordinates are millimetres: x forward from the line through the arm bases, y to the robot’s left, z above the table. The two arm bases are at (0, +305) and (0, −305) mm. Both arms start at the standard ready pose with grippers open. Layout and physical parameters are fixed between the demonstration and model trial.

[task.json](task.json) gives the complete start poses, dimensions and materials. [TECHNICAL.md](TECHNICAL.md) lists product sources, estimated parameters, the build procedure and asset licences.

## Successful outcome

Lid within 3° of shut, jester inside, box within 15 mm and 10° of its marks, grippers ≥50 mm clear for 2 s. Tipping the box more than 30° at any time invalidates the trial.

## Coordinated operation

One hand compresses the jester while the other lowers the lid. Transfer the spring load to the lid, withdraw the first hand, then engage the catch and release.

The scorer evaluates the physical outcome. It does not award points for using a particular arm or for following the reference script.

## Run and inspect the result

Use Linux with an NVIDIA RTX-capable GPU and the pinned [YAM Lab-Arena runtime](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1): Python 3.12, Isaac Sim 6.1 and Isaac Lab-Arena 0.3.1. Follow that runtime’s installation instructions and source its `yam.env` before running these commands from this task folder. Isaac Sim is an external dependency, not part of this package.

```sh
uv pip install 'inspect-robots==1.0.0' 'inspect-robots-yam==0.38.0'
uv pip install 'dreamlabs-client==1.0.14' 'inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339'
uv pip install 'numpy>=1.26' 'scipy>=1.11' 'pillow>=10' 'imageio[ffmpeg]'
uv pip install --no-deps .
python -m rdb_jack_in_box.run scripted --out runs/scripted
python -m rdb_jack_in_box.run dream --out runs/dream
```

Configure Dream Labs credentials privately on the run host before running the `dream` trial. Both commands use Inspect Robots and the same scorer in `rdb_jack_in_box/grading.py`. The scripted controller may read simulator state; it is a feasibility reference, not a policy with the same information as Dream-1.

Outputs: `RESULT.json` (score and diagnostics), `media/setup.png`, `media/top_cam.mp4`, `media/side_cam.mp4` and `media/grader-trace.jsonl`. Read the final score in `RESULT.json`; the trace explains when each score changed. Dream-1 receives the overhead and two wrist RGB images at 320 × 176 plus standard joint/gripper position readings. The side camera is recorded for review only. Grader state and diagnostic contact forces are not model observations.

Fixed seed 0, zero object jitter, 20 Hz control and 240 Hz physics. Run limit: 1,600 steps (80 simulated seconds). Wall time includes loading and recording and is longer than robot time.

| Recorded trial | Score | Control steps | Wall time |
|---|---:|---:|---:|
| Scripted demonstration | 1.0 | 668 / 1,600 | 132.91 s |
| Dream-1 | 0.0 | 1,600 / 1,600 | 289.50 s |

These are individual recorded simulation trials. [validation.json](validation.json) records the run settings, scores and evidence scope. No hardware trial is reported.

Tests without launching Isaac Sim, using the installed YAM Python environment:

```sh
uv pip install pytest
python -m pytest tests -q
```


## Scoring

The scorer is `rdb_jack_in_box/grading.py`; `task.json` states the same rules in words.

- **Lid shut**: the lid within 3 degrees of shut (its front edge at most 7 mm above the rim).
- **Head below the rim**: the head's top no more than 1 mm above the rim.
- **Jester inside**: the head's top no more than 4 mm above the rim (under a shut lid: 3 degrees open is 3.5 mm
  at the head) and the head wholly inside the walls, so nothing of the jester is caught between lid and rim.
- **Hands clear**: no part of the robot touches the toy, and each gripper's centre point is at least 50 mm from
  the box (crank included). Unreadable grippers count as not clear.
- **On the marks**: the box's middle within 15 mm and its turn within 10 degrees of the tape marks, upright.
- **0**, latched: the box tipped more than 30 degrees at any time.
- **1.0**: lid shut, jester inside, hands clear, box on its marks, all held for 2 s. The spring pushes the lid up
  with about 3 N at its front edge, so a lid that stays shut hands-off for 2 s is held by the catch.
- **0.6**: lid shut with the jester inside at the end (still held by a gripper, or the box moved).
- **0.3**: at some moment the head was below the rim while the lid was 30 degrees or less from shut (latched).
- otherwise **0**.

Outcomes: nothing done, 0; jester pressed down with the lid still wide open, 0; lid brought down onto the
jester and the head bent out over the front edge, 0 (or 0.3 if it was below the rim with the lid nearly shut);
lid shut on the jester but held shut by a hand, 0.6; lid shut and caught, hands away for 2 s, 1.0.

### Scoring a real trial

1. From the video: was the head ever wholly below the rim with the lid within 30 degrees of shut (front edge at
   most 67 mm above the rim)? Note it (0.3 is earned by that moment alone).
2. At the end: is the lid down (front edge at most 7 mm above the rim, measured with a ruler) with the whole
   jester inside the walls and under the lid (head top at most 4 mm above the rim, which a lid within 3 degrees
   of shut allows) and nothing of it caught between lid and rim? If not, 0.3 if step 1 was met, else 0.
3. Is a gripper still on or within 50 mm of the box, or has the box moved more than 15 mm or turned more than 10
   degrees from its tape marks? Then 0.6.
4. Otherwise, if the lid has stayed down with the robot clear for 2 s: 1.0.

## Reset

Nothing is consumed. Open the lid (press the catch, or turn the crank until it releases), let the jester spring up, lean the lid back against its stop, put the box back on its tape marks with the catch toward the robot, and return both arms to their ready pose with grippers open.

For a simulation reset, start a new run with a fresh output directory. The adapter reloads the fixed scene, resets velocities and joints, and clears the scorer’s event history. The physical preparation and replacement checks above apply before every real trial; estimated properties must be measured on the purchased objects before hardware replication.

## Simulation model

The head and sleeve are rigid bodies on springs rather than plush and cloth. The spring bends fore/aft only; wall and lid colliders are thicker than tin. The hook catches by contact. Internal geometry, stiffness and friction are estimates; cloth bunching and snagging are absent.

See the full `fidelity` field in [task.json](task.json) and [TECHNICAL.md](TECHNICAL.md) for the collision geometry and parameter sources. The demonstration establishes solvability in this simulation; it does not measure correspondence to real hardware.

## Files and licence

| File | Purpose |
|---|---|
| `task.json` | Task definition and submission fields |
| `rdb_jack_in_box/fixed_scene.py` | Fixed instruction, layout and run settings |
| `rdb_jack_in_box/adapter.py` | Inspect Robots embodiment and sensor/scorer integration |
| `rdb_jack_in_box/run.py` | Scripted and Dream-1 trial entry point |
| `rdb_jack_in_box/scripted.py` | Feasibility controller |
| `rdb_jack_in_box/grading.py` | Scoring implementation |
| `validation.json` | Recorded scores and settings |
| `TECHNICAL.md` | Dimensions, sources, modelling details and real-trial preparation |
| `tests/` | Existing scene, controller and scoring tests |

Task-specific source and generated geometry are provided under [MIT](LICENSE). No third-party product meshes or textures are bundled. Robot, station and simulator dependencies retain their upstream licences; see the asset list in [TECHNICAL.md](TECHNICAL.md).

The submission card uses short title/setup/fidelity fields to meet the board’s length limits. [REFERENCE-SPEC.json](REFERENCE-SPEC.json) retains the full specification used by the recorded trials; [TECHNICAL.md](TECHNICAL.md) explains its dimensions and modelling assumptions. The instruction and scoring are unchanged.
