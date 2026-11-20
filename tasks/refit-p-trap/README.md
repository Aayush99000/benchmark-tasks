# Refit a sink trap's J-bend onto both pipes, hand-tight

Under a bathroom vanity's basin (doors open; the station table is the cabinet's floor), the basin's chrome drain
tailpiece hangs down from the drain, and a white PVC trap arm comes out of the back wall and turns down beside it. The
1-1/2 in J-bend that joins them has been taken off and cleared, and lies on the cabinet floor on its side. Each pipe
still carries its slip nut, its beveled washer inside it, slid up out of the way.

The robot must refit the J-bend the way a plumber or a maintenance worker does after clearing a clogged sink: offer
it up onto both pipes at once (the tailpiece into one leg, the trap arm's end into the other), hold it there, run each
slip nut down onto its leg's thread and turn both hand-tight, so that the trap hangs from its nuts by itself.

Why it matters: clearing a clogged drain by pulling the trap is routine work for plumbers (O*NET 47-2152.00,
"Maintain or repair plumbing by replacing defective washers, replacing or mending broken pipes, or opening clogged
drains") and for general maintenance and repair workers (49-9071.00, about 1.5 million in the US; core task
"Assemble, install, or repair wiring, electrical or electronic components, pipe systems, plumbing, machinery, or
equipment") in homes, apartments, hotels and offices. Refitting is the hard half: one rigid part goes onto two fixed
pipes 88 mm apart at once, under the basin, their ends hidden inside its legs; the first joint has to stay seated,
undisturbed, while the second is made; and each plastic slip nut is run four turns onto its thread by hand, square,
without cross-threading.

## Files

| File | What |
|---|---|
| `refit_p_trap.py` | The yam_arena task file: the objects, our slip-joint model, `success()`, `score()` and the scripted `demo()` |
| `ptrapkit.py` | Compound rigid bodies, joints and per-substep force models with hooks into yam_arena's scene and reset, adapted from `fittingkit.py` of this repository's accepted `compression-joint` task (MIT licence; see Credits) |
| `check.py` | Offline checks (no GPU): the demo's reach with the official IK and its length |

The task runs with the standard `yam_arena` embodiment and the `task_score` scorer. No meshes or other third-party
assets are used; every shape is a box or cylinder at the real part's size.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=refit_p_trap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Refit the J-bend under the sink: fit it onto the tailpiece and the trap arm, then tighten both slip nuts by hand." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/trap-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. One trial takes
3,600 control steps (3 minutes at 20 Hz). No seeds: the layout is fixed (`-E object_jitter=0`). To render the trial's
videos, run `inspect-robots video ~/yam-eval/trap-dream/<log>.json`.

## Scripted demo

Same command with `--policy yam_scripted`. The demo is open loop: it uses only the fixed layout. It reaches 1.0 in
about 134 s (2,690 steps). The demo video shows the scene camera on top and the left and right wrist cameras below,
all from the same run:

- 0-15 s: the left hand grips the lying J-bend by its outlet leg, stands it upright, brings it in under the pipes and
  pushes it up onto both at once.
- 15-27 s: the right hand runs the tailpiece's nut down onto the inlet leg's thread and turns it half a turn in two
  90-degree strokes (grip, swing round the pipe, let go, draw back off the nut, swing back, come in again); the thread
  now holds the J-bend.
- 28-39 s: the left hand lets go of the J-bend and runs the trap arm's nut down onto the outlet leg's thread.
- 40-118 s: both hands turn their nuts in step until both are snug (four turns each).
- 118-134 s: both hands back off and go clear; the J-bend hangs from its nuts.

## Our model (what the simulator cannot do)

PhysX has no threads, so the slip joints are our model (`SlipJoint` in `refit_p_trap.py`), run before every physics
substep. Everything the robot does is physical:

- The pipes, the J-bend and the nuts are rigid bodies at real size; each pipe slides into its leg's bore physically
  (1.65 mm clearance on the radius). A real leg's mouth is bevelled (its washer seats in the bevel), and a real tailpiece
  and trap arm have a little play (a long tailpiece flexes; the trap arm gives a little in its wall adapter's slip
  joint), so a leg offered up a few millimetres off its pipe still goes on. Our pipes are rigid and fixed, so the bevel
  and the play are a rule: pushed up onto a pipe's end within 4 mm of the bore's axis (about the bevel's width), the leg
  is centred by a force of at most 15 N, only while the pipe's end is at the mouth. The robot still has to bring both
  legs within 4 mm of both pipes at once and push the J-bend up; further off, the leg's rim stops on the pipe's end, or
  misses it, and it does not go on.
- Off the thread, each nut is a free body riding its pipe, square and centred, held where it is put by its washer's
  drag; the hand slides it down by gripping it.
- A nut stops at its leg's thread's start. It goes onto the thread only if the leg is on its pipe (at least 10 mm in),
  coaxial with it within 2 mm and upright within 3 degrees; otherwise it sits on the thread's start, not started.
- On the thread the nut turns only with a hand gripping it (both pads closed on it, the TCP on its axis at its height)
  as the hand swings round the pipe's axis: clockwise from above, it runs down 2.21 mm a turn (11-1/2 threads per
  inch) and stops after four turns, its washer squeezed home (hand-tight). From a quarter turn on, the thread holds the
  J-bend's leg on its pipe.
- The beveled washers are modelled inside their nuts.

## Scoring

| Part | Credit | Real-trial check |
|---|---|---|
| Seated | 0.2 | the J-bend on both pipes at once, each pipe at least 10 mm into its leg (a pencil line 10 mm from each pipe's end is hidden in the leg), coaxial within 3 mm, the J-bend upright within 5 degrees |
| Each nut started | 0.15 | at least one full turn (2.2 mm) onto its leg's thread (a paint ring 2 mm below the thread's top is hidden under the nut) |
| Each nut snug | 0.15 | run its four turns, the washer home: no thread shows below the nut, and it will not turn further by hand |
| Hanging | 0.2 | seated and both nuts snug, nothing holding the J-bend but its nuts (no gripper within 10 mm of it or a nut, the J-bend not moving more than 2 mm), held 1 s |

Score = the sum at the end of the trial; the trial ends as a success once hanging holds. A nut cannot start on a leg
that is not on its pipe and coaxial with it: pushed down there, it stops on the thread's start and scores nothing.

## Real trial

Parts (about $40): a 1-1/2 in PVC tubular P-trap kit (e.g. Oatey HDC9682: J-bend, trap arm, slip nuts, beveled
washers, trap adapter), a 1-1/2 in x 8 in chrome-plated brass tailpiece or a pop-up drain with its tailpiece, and a
basin or a plywood board with a drain hole. Fix the drain and tailpiece and the trap adapter (on a 1-1/2 in PVC stub
through a plywood back wall) at the heights in `task.json`, on a plywood mock-up of the cabinet clamped to the table.
No water is used. Plastic slip nuts are tightened by hand only, with no tools; the gripper turns each by its four
wings. Reset: unscrew both nuts, slide them up to their tape marks, lay the J-bend on its marked place.
Replace the kit when a nut no longer runs on freely.

## Credits

`ptrapkit.py` is adapted from `fittingkit.py` in this repository's `tasks/compression-joint/` (MIT licence, its authors'
own code): its bodies, rings, joints, force-model hook, reset hook and finger points. Our changes are listed in its
docstring.
