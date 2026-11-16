# Tighten a circular saw's blade bolt a quarter turn past hand-tight, holding the spindle lock in

A cordless circular saw lies on its side on the table with its blade facing up. A new blade is on, and the clamping
bolt at its centre is hand-tight: a white paint line across the bolt's head lines up with one on the clamping flange.
A 6 mm hex key stands in a holder nearby. The instruction asks for the maker's tightening: a further quarter turn with
the key, then the key put down.

The bolt turns on the spindle's own axis, so a key on the bolt just spins the blade and bolt together. The bolt only
tightens against the blade while the spring-loaded spindle lock button is held in, and the lock only catches when the
spindle comes round to one of its two notches. So one hand holds the lock in while the other turns the key: two hands,
each doing a different job, at the same time.

## The real step
Bosch, [PKS 16 Multi operating instructions](https://www.bosch-diy.com/storage/diy/zip-manuals/pks-16-multi-100034817-zip-424533/en-GB/934869387.html),
changing the saw blade: "Press and hold the spindle lock button (12)", tighten the clamping bolt with the hex key, and
"The tightening torque should be 6–9 Nm, which corresponds to hand-tight plus ¼ turn." Done by: carpenters (BLS:
about 959,000 jobs in 2024), who change blades whenever one dulls or the cut changes.

What goes wrong in real life: a bolt left short of the maker's tightening, or one turned well past it. The paint lines
show it: the angle between the line on the bolt's head and the one on the flange is how far the bolt has turned past
hand-tight.

What we simplified: the saw is fixed. The blade, bolt and spindle lock are a model (below) rather than contact on a
thread and a gear. The thread's torque is scaled down so the YAM can turn it. Scoring is by angle, which is what the
maker's "hand-tight plus ¼ turn" checks.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=saw_blade_bolt.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The new blade is on the saw and its clamping bolt is hand-tight. Tighten the bolt a further quarter turn with the hex key, as the saw's maker says, then put the key down." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/saw-blade-bolt-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=saw_blade_bolt.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The new blade is on the saw and its clamping bolt is hand-tight. Tighten the bolt a further quarter turn with the hex key, as the saw's maker says, then put the key down." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/saw-blade-bolt-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/saw-blade-bolt-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the finished state has held for 1 s; otherwise it
scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| saw | a cordless circular saw for 184 mm (7-1/4 in) blades, fixed, lying on its side with the blade facing up: the motor and gear housing (10 cm across) under the blade, a rear handle, and the shoe standing on edge on the far side | spindle axis at (26, 14) |
| guards | cast aluminium, closed all round the blade's teeth: a ring 7.6-10.1 cm from the axis, 1.4 cm above and below the blade (the upper guard, and the lower guard closed) | round the blade |
| blade | 184 mm, 1.6 mm thick, 9.5 cm above the table, with a red label; under it, the spindle's gear has two notches half a turn apart; at the start the spindle is 25 degrees (clockwise from above) short of one | on the spindle |
| clamping flange | steel, 30 mm across, 4 mm thick, on the blade, with a white paint line | on the blade |
| clamping bolt | steel, a round head 13 mm across and 8 mm tall with a 6 mm hex socket 5 mm deep; hand-tight; a white paint line across its head lined up with the flange's | at the blade's centre, its head's top 10.8 cm above the table |
| spindle lock button | red, 24 mm across, standing 7 mm proud of a pad on the gear housing, 4.5 mm of travel, sprung (about 2 N at rest, 600 N/m); a white mark beside it | (26, -2.5), its top 8.2 cm above the table |
| hex key | black-oxide steel, 6 mm across the flats, L-shaped: short arm 30 mm, long arm 100 mm, 32 g | standing in its holder, short arm down, long arm along +x |
| holder | wood, fixed: a 13 x 2.6 cm base with a post holding the short arm and a post under the long arm's end | short arm at (20, 30) |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
Measured on the bolt: how far it has been turned clockwise (tightening, seen from above) past hand-tight, in degrees:
the angle between the paint line on its head and the one on the flange. Turning it back counts against it.

| Score | When, at the end |
|---|---|
| 1.0 | 72 to 108 degrees past hand-tight (a quarter turn: 6-9 N m by the maker's own equivalence), the key off the bolt (more than 2 cm from its socket) and put down (lying still on the table, its lowest point within 6 mm of it, no gripper within 1 cm of it), and the spindle lock released (pressed in less than 1 mm), held for 1 s, which ends the trial |
| 0.9 | 72 to 108 degrees, but not finished: the key still on the bolt or not put down, the lock still held in, or time ran out before the finished state had held for 1 s |
| 0.5 | 36 to 72 degrees (under-tight) or 108 to 144 degrees (over-tight) |
| 0.25 | 15 to 36 degrees |
| 0 | anything else, including more than 144 degrees or any loosening |

The bolt turns against the blade only with the spindle lock caught; otherwise the key turns the blade and bolt
together and the angle stays 0. So no credit is possible without one hand holding the lock in while the other turns
the key. Nothing else (the fingers on the bolt's head, pushing the blade) turns the bolt.

## The saw model
`SawBladeBolt` in `saw_blade_bolt.py` runs before every physics substep (240 Hz). It poses the blade and the bolt from
two angles: the spindle's, and the bolt's past hand-tight.

**The spindle lock.** The button is a body on a sprung slider in the saw (a PhysX prismatic joint with a drive). Pressed
1.2 mm or more as the spindle reaches a notch, the lock catches and holds the spindle exactly at the notch. Away from
a notch the pin rides on the gear's face, and the model stops the button at 1.5 mm. Let go, the spring pushes the
button out and the lock releases.

**The key in the socket.** The key goes in when its short arm is within 1.5 mm of the axis, at least 1.5 mm into the
socket, within 10 degrees of upright and lined up with the socket's flats within 12 degrees. A key off its flats is
pushed back out of the socket. While in, the key is held on the axis, and its turning (about the vertical) turns:
- with the lock caught: the bolt, against a thread torque rising to 0.5 N m at a quarter turn (scaled down from the
  maker's 6-9 N m);
- with the lock free: the blade and bolt together, against a light drag; pressed in, the lock catches at the first
  notch the spindle reaches, and the rest of that turn goes to the bolt.

Lifted to less than 0.5 mm in the socket, or pulled more than 4.5 mm off the axis or 20 degrees off upright, the key comes off.
The key's arms collide as round bars inside the hexagon (3 mm radius), and the socket's collision hole has 1.5 mm of
play round them, so the held key never fights the contact solver (the accepted compression-joint task does the same).

## The scripted demo
Open loop, both arms:
1. The left hand takes the key from its holder, its fingers closing across the long arm 55 mm from the corner, and
   lifts it until the key's tip is 3.6 cm above the bolt's head (3.5 cm above the guard) before crossing.
2. It turns the key so its short arm lines up with the socket, comes down to 5 mm above the socket, holds still there,
   and puts the short arm 4 mm into the socket.
3. Meanwhile the right hand comes down on the spindle lock button and presses it, holding it in. Away from a notch it
   only goes in to where the pin rides on the gear.
4. The left hand turns the key clockwise in 16 steps of about 7.4 degrees, 118 degrees in all. The blade turns with it for 25
   degrees, until the lock catches at the notch; then the bolt turns past hand-tight, about a quarter turn.
5. The left hand lifts the key out, carries it over the guard and lays it on the table; the right hand lets go of the
   lock and moves clear.

The plan allows for the arm as the official sim measured it (each `WHY` comment in `saw_blade_bolt.py` gives the run):
the key crosses 3.5 cm above the guard, and the insertion aims 1 mm off the axis against the arm's measured offset.

## Part kit
`fittingkit.py` is copied from the accepted robocurve/benchmark-tasks task `tasks/compression-joint` (commit de817b9),
under the MIT licence its authors state in that task's README, with one addition noted in its docstring: `Slider`
takes an optional spring drive (`stiffness`, `damping`, `target`), used here for the lock button. It builds rigid bodies
from boxes and cylinders, poses bodies and applies forces from a model every physics substep, and hooks yam_arena's
scene and reset.

## Runtime and seeds
- Official runs (2026-11-16, the Robocurve Brev launchable, 1,200 steps, `object_jitter=0`): the scripted demo scores
  1.0: the key goes into the socket at 11.5 s, the lock catches at the notch at 14.5 s, the key comes out with the bolt
  95.7 degrees past hand-tight at 19.7 s, and the trial ends at 24.8 s (about 495 steps) once the key has lain on the
  table, the hands clear, for 1 s. Earlier runs of the same demo (a state trace, and one before the key had to be put
  down) gave the same three events at the same times. Dream-1 scores 0.0 over the full 1,200 steps: its left hand
  moves about near the key's holder but never picks the key up.
- The scripted demo's plan is 25.0 s of simulated time (the right arm's, 22.4 s).
- The scoring, saw model, geometry, reach and step-guard checks, and a rigid replay of both hands' paths through the
  model run without the simulator: `python -m pytest -q test_saw_blade_bolt.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
