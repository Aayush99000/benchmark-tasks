# Load a strip of nails into a framing nailer

A pneumatic 21-degree framing nailer, not connected to air, lies on its side on the bench with its magazine empty, and
a strip of 25 framing nails lies beside it. The instruction asks for the nailer to be loaded the way its maker
describes: feed the strip into the rear end of the magazine, front first with the nail heads in the head track, push
it in past the nail stop, then slide the pusher all the way back past the strip and ease it forward against the nails
without letting it snap. One hand feeds the strip and pushes it home; the other works the spring pusher.

## The real step
Framers and other carpenters reload their nailers all day: a strip holds 25 nails. RIDGID, R350RHA 21-degree framing
nailer operator's manual, Loading the tool with nails: "feed a strip of nails into the magazine. Be sure the nails are
pointed downward and at the angle shown", "Slide the pusher all the way to the rear of the magazine. Release the pusher
and allow it to push the nails up to the driving mechanism", "Do not allow the pusher to snap back into place"; and
from its jam-clearing steps, the strip comes out only by pressing "the nail stop tab near the end of the magazine".
NIOSH and OSHA, Nail Gun Safety: A Guide for Construction Contractors (NIOSH 2011-202): nail guns are "used every day on
many construction jobs, especially in residential construction". Done by: carpenters (BLS and O*NET, 2025: 889,700
jobs), framers in particular.

What goes wrong in real life: a strip not pushed past the nail stop slides back out as the pusher goes back over it,
and the pusher never gets behind it; a pusher let go snaps forward into the strip.

What we simplified: the nailer has no air line (RIDGID's manual connects the air before loading, against unintended
cycling; here nothing can fire), and it lies fixed on the bench, as if held there; the magazine is a model (below), with
its last 55 mm open on top and a 15 mm-deep pocket under part of that opening, so the gripper's closed fingertips can
push the strip's back past the nail stop where a carpenter would use one finger; the strip is one rigid body, and it
starts lying on a 2x4 offcut, so a gripper can get its fingers under its edges.

How this differs from the board: robocurve#1467 puts a brad nailer away without pointing it at anyone (where a tool
points); this one is the loading sequence: a strip turned the right way into a tight track, a one-way stop, and a
spring feed that must get behind the strip and be let forward under control.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=nailer_strip.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load the strip of nails into the framing nailer. Feed it into the rear end of the magazine, front first with the nail heads in the head track, and push it in past the nail stop. Then slide the pusher all the way back past the strip and ease it forward against the nails without letting it snap." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/nailer-strip-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=nailer_strip.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load the strip of nails into the framing nailer. Feed it into the rear end of the magazine, front first with the nail heads in the head track, and push it in past the nail stop. Then slide the pusher all the way back past the strip and ease it forward against the nails without letting it snap." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/nailer-strip-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/nailer-strip-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the finished state has held for 1 s; otherwise it
scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| nailer | a red 21-degree pneumatic framing nailer, not connected to air, lying on its side, fixed; its aluminium magazine (34 cm long) along the table's y, its nose and driver to the robot's right, its handle and cylinder head on the far side of the magazine | the magazine's rear end at y 13, its track's middle line at x 17 |
| magazine track | open at its rear end: a 7.5 mm slot the nails' shanks run in, and beside it, on the far side (towards the handle), a head track 11.5 mm deep the heads run in (the heads, 7.1 mm across, don't fit the slot); 83 mm across, its walls opening out 15 degrees over their last 15 mm; its floor's edge at the rear end and its cover's edge at the window's front chamfered 45 degrees (5 and 2.5 mm); its last 55 mm open on top (the loading window), with a pocket 32 mm wide and 15 mm deep in the window's floor; the nail stop 35 mm in | the track's middle 3.4 cm above the table |
| pusher | a black slider on a rail beside the head track, its knob (12 x 14 x 16 mm) 25 mm behind its finger; a spring pulls it towards the driver (about 3 N there, 6 N at the rear); empty, its finger rests at the driver | its knob at (22.5, -18.5), 6 cm up |
| nail strip | 25 plastic-collated 21-degree full round head nails, 3 in x .131 in (Grabber: 25 a strip), 21.5 cm long, 7.6 cm across, 130 g; orange collation | lying flat on the offcut, along x, its middle at (38.5, 26), its front (the first nail's point) away from the arms, its heads to the robot's left |
| offcut | a 2x4 offcut (38 x 89 mm, 20 cm long) standing on edge, fixed, the strip lying across its top | its middle at (38.5, 26) |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
From the magazine's model and the parts' poses.

| Score | When, at the end |
|---|---|
| 1.0 | the strip up to the driver with the pusher against it, the pusher never having snapped, and both grippers more than 2 cm from the strip and the pusher's knob, held 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.5 | the strip fed up to the driver, but the pusher snapped |
| 0.25 | the strip in the track the right way round and past the nail stop, not fed |
| 0 | anything else, a strip in back to front included |

- In the track: the strip's front inside the rear end, within 4 mm of the track's line and of lying on the slot's
  floor, within 10 degrees of the track's direction, and its heads towards the head track.
- Past the nail stop: the strip's back more than 36 mm in.
- Snapped: the pusher met the strip, or pushed it forward short of the driver, faster than 0.25 m/s.
- Up to the driver: the strip's first nail within 1 mm of the driver.

## The magazine model
`Magazine` in `nailer_strip.py` runs before every physics substep (240 Hz), from the strip's and the pusher's poses:
- A strip coming in lined up with the track has its front steered to the slot's line through the rear end and the
  loading window (a damped pull, up to 10 N); the chamfered edges lift a front that comes in low. With its heads on
  the points' side it stops at the rear end: its heads meet the shanks' slot (a damped stop, up to 40 N).
- Once the strip's back is past the nail stop, it can't come back out past it: it is held there.
- The pusher's finger, pulled back over the strip, rides on it and drags it back with it (up to 2 N). Pulled back past
  the strip's last nail, it drops in behind it.
- Let forward, the finger meets the strip, and its speed then is noted. From there the pusher pushes the strip along
  the track (the strip's pose is written back every substep, in the track), the strip's friction (up to 1.3 N)
  against it, and at the driver it is held there.
- A strip in back to front doesn't feed.

The pusher is a rigid body on a sprung PhysX prismatic joint on the nailer (`fittingkit.Slider` with a drive: 10 N/m,
its target 30 cm past the driver). The two hard stops (the nail stop and the driver) hold their part's pose rather
than push on it, as a stiff spring acting once per substep would make a light part ring.

## The scripted demo
Open loop, both arms, on one shared timeline, with fixed planned poses (no correction from the sim's object poses):
1. The left hand comes down over the strip, takes it across its width 70 mm from its back end, lifts it, and turns it
   a quarter turn clockwise, so its front points at the driver and its heads at the head track.
2. It brings the strip in line with the track behind the rear end, feeds it in until its fingers are 5 mm short of
   the rear end (the strip's front about 12 cm in), and lets go.
3. Its closed fingertips go down behind the strip's back, into the window's pocket, and push it 42 mm in, past the
   nail stop.
4. The right hand takes the pusher's knob at the driver, pulls it back until its finger is behind the strip's last
   nail, and eases it forward over 3.5 s until the strip is up to the driver; it lets go and both hands move clear.

The poses were checked offline against the YAM's collision hulls (URDF forward kinematics, `plan_check.py`: the
grippers, fingers and last three links): neither arm touches the nailer, the pusher or the offcut except where it is
meant to, the strip the left hand carries doesn't touch the nailer, and the arms stay at least 15.7 mm apart.

## Part kit
`fittingkit.py` is copied from the accepted robocurve/benchmark-tasks task `tasks/compression-joint` (commit de817b9),
under the MIT licence its authors state in that task's README, with two additions noted in its docstring: `Slider`
takes an optional spring drive (`stiffness`, `damping`, `target`), used here for the pusher, and a box `Shape` takes an
optional `roll` (a turn about its own x axis), used here for the magazine's chamfers. `plan_check.py` is the offline
check of the demo described above.

## Runtime and seeds
- Official runs (2026-11-18, the Robocurve Brev launchable, 1,200 steps, `object_jitter=0`): the scripted demo scores
  1.0: the strip goes past the nail stop at 27.0 s, the pusher's finger drops in behind it at 32.3 s and meets it at
  0.11 m/s at 33.0 s, the strip is up to the driver at 35.0 s, and the trial ends at 38.9 s (777 steps) once both
  hands are clear. Dream-1 scores 0.0 over the full 1,200 steps: its left hand goes to the strip on its offcut and
  knocks it about without picking it up, its right hand goes to the nailer's nose, and the strip never goes into the
  magazine.
- A probe of the snap rule (the same demo, with the right hand letting the knob go as soon as it has pulled the pusher
  back): the pusher meets the strip at 1.36 m/s and drives it to the driver, and the run scores 0.5.
- The scripted demo's plan is 41.5 s of simulated time.
- The magazine model, the scoring, the sizes, reach and step-guard checks run without the simulator:
  `python -m pytest -q test_nailer_strip.py`; the demo against the real collision hulls: `python plan_check.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
