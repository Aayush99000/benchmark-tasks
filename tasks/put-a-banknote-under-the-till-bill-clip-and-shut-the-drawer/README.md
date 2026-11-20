# Put a banknote under the till hold-down and close the drawer

Put the banknote that is lying on top of the cash drawer into the till: lay it flat in the middle note compartment, on top of the notes already there and under that compartment's hold-down clip, with the clip let down on it. Then push the drawer shut and take both arms away.

## Run one trial

Use Python 3.12, Isaac Sim 6.1, Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0,
inspect-robots-yam 0.38.0 and yam-arena 0.1.0 on Linux with an NVIDIA GPU.
The stock YAM station, arm meshes and cameras are runtime dependencies; they are not bundled.
Install this folder in that environment:

```sh
uv pip install --no-deps .
python -m rdb_till_note.run scripted --out out/scripted
python -m rdb_till_note.run dream --out out/dream
```

The Dream run requires a Dream Labs credential configured through its client. No credential is included.
Both runs use Inspect Robots, the same scene and scorer, seed 0 and zero object jitter.
Run limit: 2,400 steps (2 minutes at 20 Hz). Use the default layout; additional diagnostic variants are not the submitted trial.

| Recorded run | Score | Steps | Wall time |
| --- | --- | --- | --- |
| Scripted physical-feasibility reference | 1.0 | 968 | 308 s |
| Dream-1 | 0.0 | 2,400 | 809 s |

Dream-1 receives only 320 x 176 RGB from the overhead and two wrist cameras, plus joint and gripper positions.
The scripted reference uses simulator object state for feedback; it is not an observation-matched model baseline
or a demonstration of perception from RGB. It commands robot joints and grippers without writing object poses.
The review-only side camera is never a model input.

Each run writes `RESULT.json`, `media/setup.png`, `media/top_cam.mp4`, `media/side_cam.mp4`
and `media/grader-trace.jsonl`. Read `score` in `RESULT.json` for the final score.
The scripted command exits successfully only at full score. Startup and model latency add wall time.

## Fixed setup, scoring and reset

`task.json` gives every object, the fixed starting layout, numeric partial-credit/failure rules and reset steps.
The setup screenshot uploaded with this task is the recorded starting scene.
`validation.json` records the two scores and matching scene/scorer hashes.

Reset: Nothing is consumed. Pull the drawer open to its stop, take the marked note out, square up the 50 notes in the middle compartment and let the hold-down down on them, lay the marked note on the case top on its tape marks (centred on y -144, 50 mm over the front edge), and return both arms to ready with grippers open. Replace a note that has been creased or torn. In simulation: seed 0, zero object jitter; drawer, hold-downs, stack and note are returned to their start state and the scorer starts fresh.

## Physics and reproduction

PhysX, 240 Hz physics/20 Hz control. Note: eight rigid strips with sprung hinges; bends lengthwise only, cannot crease or tear. Collider 2 mm, mass 8 g (real 0.11 mm, 1 g); hinge stiffness approximates paper droop. The 50-note stack is one loose block. Sprung hold-downs; drawer slide has 2 N friction, no ejector spring or latch. Dimensions, stiffness and friction partly estimated. Full assumptions: REFERENCE-SPEC.json and TECHNICAL.md.

[TECHNICAL.md](TECHNICAL.md) contains the detailed build list, product/dimension sources, low-cost
reproduction options, physical scoring procedure, controller stages and measured limitations.
[REFERENCE-SPEC.json](REFERENCE-SPEC.json) preserves the full specification used for the recordings.
Only the card title, context and fidelity summary were shortened; instruction, setup, scoring, reset and run limit are unchanged.

## License and tests

Task code and procedural assets: MIT, see `LICENSE`. No manufacturer CAD, images or meshes are included.
Third-party runtime dependencies retain their own licenses, listed in TECHNICAL.md.
Run `python -m pytest -q tests` for the local checks. Dry runs are unit-level checks, not simulator evidence;
the scores above come from the recorded Isaac runs.
