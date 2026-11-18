# Change a camera's lens through its bayonet mount

A camera body lies on its back on the table, its lens mount facing up, with its kit zoom lens locked on it, and a new
50 mm lens stands on a lens cloth beside it. The instruction asks for the lens to be changed the way the camera's maker
describes: press the lens release button and turn the zoom lens until it stops, lift it off and stand it on the cloth,
then line the new lens's red index up with the camera's, set it on the mount and turn it until it clicks. One hand holds
the release button in while the other turns the locked lens off.

## The real step
Photographers change lenses for each shot they plan, and every owner of an interchangeable-lens camera learns how. Canon,
[EOS 70D instruction manual, Attaching and Detaching a Lens](https://www.manualslib.com/manual/543207/Canon-Eos-70d.html?page=40):
"Align the lens' red or white mount index with the camera's mount index of the same color", "Turn the lens as shown by
the arrow until it clicks in place"; to detach it, "While pressing the lens release button, turn the lens as shown by the
arrows", "Turn the lens until it stops, then detach it." Done by: photographers, camera owners and camera shop staff.

What goes wrong in real life: a lens not turned to the click isn't locked and falls off the camera when it is picked up;
a lens forced on out of line, or turned without the release pressed, damages the mount.

What we simplified: the camera is fixed (as if held flat on the table) and switched off (the maker's first step); the
lenses' rear caps and the camera's body cap are already off; the bayonet is a model (below); the lenses are rigid
cylinders; the throat's chamfer is a model (a steering push, below); the release button sits 5 cm from the mount's
middle, a little further out than on a real body, so a fingertip can reach it beside the lens.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=camera_lens.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Change the camera's lens for the new one. While pressing the lens release button, turn the zoom lens until it stops, lift it off and stand it on the cloth. Then line the new lens's red index up with the camera's red index, set it on the mount and turn it until it clicks." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/camera-lens-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=camera_lens.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Change the camera's lens for the new one. While pressing the lens release button, turn the zoom lens until it stops, lift it off and stand it on the cloth. Then line the new lens's red index up with the camera's red index, set it on the mount and turn it until it clicks." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/camera-lens-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/camera-lens-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the finished state has held for 1 s; otherwise it
scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| camera | an interchangeable-lens camera body (13 x 10 cm), lying on its back, fixed, its mount facing up; its top points to the robot's right, so its red mount index is on that side of the mount; its grip on the far side | the mount's middle at (27, 0), its face 6.6 cm above the table |
| release button | the lens release button, 11 mm across, on a spring: pressed 1.5 mm in (about 2.7 N) it frees the lock; 3 mm of travel | 5 cm from the mount's middle, on the side towards the arms: (22, 0) |
| zoom lens | a kit zoom lens (Canon EF-S 18-55 mm IS STM size: 69 mm across, 75 mm long, 205 g), on the mount, locked | on the mount |
| new lens | a 50 mm lens (Canon EF 50 mm f/1.8 STM size: 69.2 mm across, 39.3 mm long, 160 g), standing on its back end, its red index towards the robot's right | (19, -17.5) |
| lens cloth | a cloth on the table, 24 x 12 cm | (25, -17.5) |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
From the mount's model and the parts' poses.

| Score | When, at the end |
|---|---|
| 1.0 | the new lens locked on the camera, the zoom lens off it and set down (on the table or the cloth, still), the release button let go, and both grippers more than 2 cm from both lenses, held 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.5 | the new lens on the mount but not locked, or locked with the zoom lens not set down |
| 0.25 | the zoom lens off the mount |
| 0 | anything else (the zoom lens still on) |

- A lens is on the mount when its back is within 1 mm of the mount's face and 2.5 mm of its axis, tilted under 6
  degrees, or when it is in the bayonet.
- Over the mount with its index more than 6 degrees from the camera's, a lens rests on the mount's tabs, 2.5 mm up (a
  stiff stop, up to 40 N).
- Within 6 mm of the mount's axis and less than 12 mm above its face, the throat's chamfer steers a lens's back towards
  the axis (1,500 N/m, up to 6 N).
- Seated and turned 4 degrees clockwise (seen from the front, here from above), it is in the bayonet: held on the
  mount's axis and face, free only to turn, between the index and the stop 60 degrees on. At the stop the lock pin clicks
  in and holds it there.
- With the release button pressed 1.5 mm in, the pin lets go: the lens turns back, and within 2 degrees of the index it
  is out of the bayonet and lifts off.
- Set down: off the mount, its centre of mass under 6 cm above the table, moving under 2 cm/s.

## The mount model
`Mount` in `camera_lens.py` runs before every physics substep (240 Hz). It reads the button's travel and each lens's
pose: how far it is turned from the index about the mount's axis, how far its back is from the mount's face and how far
off the axis. A lens near the axis gets the chamfer's push towards it, one not lined up the tabs' push up; a lens in the bayonet has its pose written back
every substep, on the mount's axis and face and turned as far as the hand has turned it (within the index and the stop,
or at the stop while locked). The button is a rigid body on a sprung PhysX prismatic joint in the camera
(`fittingkit.Slider` with a drive: 1,800 N/m). The button sits in a well in the body's front: a PhysX joint doesn't
stop a body colliding with the fixed body it slides in.

## The scripted demo
Open loop, both arms, on one shared timeline:
1. The right hand comes down round the zoom lens near its front and holds it, its fingers closing across it.
2. The left hand, fingers closed, pointing down and leaning 30 degrees out towards the arms, presses the release button
   in (aimed 9 mm under its top; the arm pushes like a spring, and the button goes in about 2.8 mm).
3. The right hand turns the zoom lens anticlockwise (seen from above) off the lock; half way round the left lets the
   button go, and the right turns the lens on to its index, lifts it off, stands it on the cloth and lets go.
4. The right hand picks up the new lens, brings it over the mount with its red index on the camera's, sets it on the
   mount, and turns it 60 degrees clockwise until it clicks; it lets go, and both hands move clear.

In the sim only one of the YAM's fingers closes on a lens this wide: the other stays at its open stop, and the lens is
pushed against it, so a held lens's axis is 12.4 mm from the TCP along the fingers' line (measured in runs cl2 and cl3;
the new lens also turns 4.5 degrees in the fingers as they close). The demo holds the hand that far off the lens's axis,
turns it about the lens's axis rather than the TCP, and carries the new lens turned back by 4.5 degrees.

The poses were checked offline against the YAM's gripper and wrist collision hulls (URDF forward kinematics,
`plan_check.py`, with each lens moving with the right hand while it is held): neither hand touches the camera, the cloth
or a lens it doesn't hold; the hands stay at least 10 mm apart; the zoom lens is turned back past its index before it is
lifted, and the new lens goes on the mount on its index and is turned past the stop. Both arms reach every pose within
0.3 mm and 0.1 degree, and no step exceeds 75% of the per-step joint limit.

## Part kit
`fittingkit.py` is copied from the accepted robocurve/benchmark-tasks task `tasks/compression-joint` (commit de817b9),
under the MIT licence its authors state in that task's README, with one addition noted in its docstring: `Slider`
takes an optional spring drive (`stiffness`, `damping`, `target`), used here for the release button. `plan_check.py` is
the offline check of the demo described above.

## Runtime and seeds
- Official runs (2026-11-17, the Robocurve Brev launchable, 1,200 steps, `object_jitter=0`): the scripted demo scores
  1.0: the release button frees the lock at 8.65 s (it goes in up to 2.8 mm), the zoom lens is out of the bayonet at
  12.28 s and stands on the cloth, the new lens goes into the bayonet at 31.89 s and clicks at 33.97 s, and the trial
  ends at 38.35 s (767 steps). Earlier runs of the demo, while the task was being built: 0.0 (the button couldn't move,
  sitting on the body's front), 0.25 and 0.25 (the zoom lens off; the new lens brought down off the mount's axis sat on
  the ring). Dream-1 scores 0.0 over the full 1,200 steps: both hands move about the camera and the cloth, but neither
  presses the release button or picks up a lens, and the zoom lens stays locked on the camera.
- The scripted demo's plan is 38.9 s of simulated time.
- The mount model, the scoring, the sizes, reach and step-guard checks, and the demo against the real collision hulls
  run without the simulator: `python -m pytest -q test_camera_lens.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
