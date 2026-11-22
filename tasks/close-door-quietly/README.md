# Shut a bedroom door without the latch clicking

A true-size interior door stands ajar beside the station, opening towards the robot. The robot must shut it so it
latches, without a sound: turn the knob and hold it so the latch bolt stays in the door's edge, push the door against
its stop, then let the knob turn back slowly so the bolt slides into the strike's hole. Pushed shut the ordinary way,
the bolt rides over the strike's lip and clicks; that scores 0.5.

Instruction (the outcome only; it does not say how): `Shut the door all the way so it latches, without making a sound.`

## Why it matters
Closing a door silently, outside a sleeping baby's room or a partner's, is one of the most familiar things people do
at home. The method is the everyday one wikiHow gives ("Shut a Door Quietly"): retract the latch with the knob, keep
it held while the door closes slowly, then ease the knob back. It is a held control while something else moves, then a
speed-limited release. Plain door-closing tasks exist in public suites (Meta-World door-close, RoboCasa
close-single-door): they push a door shut and never touch the latch. Here a push-shut is exactly what scores 0.5.

## The door
- **Door:** a 24 x 80 in (61 x 203 cm) hollow-core slab, 1-3/8 in (34.9 mm) thick, 9 kg (a 20 lb hollow-core slab,
  as retail listings give), on a vertical hinge axis behind the arm bases (x -0.25 m), its centre plane at y -0.75 m
  when shut. It opens towards the robot and starts ajar 6 degrees (latch edge 64 mm off the frame; past about 7
  degrees the slab would meet the table's side).
- **Knob and latch:** a Kwikset Tylo passage knob (200T), 2-3/8 in backset (Kwikset gives 2-3/8 or 2-3/4 in), 36 in
  above the floor (0.154 m above the table top), modelled 66.7 mm across with its face 66 mm off the door (from a
  retailer's 2-5/8 x 2-19/32 in). A cam on the knob's spindle draws the 13 mm bolt flush at 45 degrees either way; a
  spring (300 N/m, 1.2 N preload) pushes it out.
- **Frame:** a 4-9/16 in (116 mm) latch jamb 3.5 mm beyond the shut latch edge, running away from the robot; a brass
  strike with a hole and a 45 degree lip on its robot side; stop moulding on the far side; hinge jamb and head.
- Joint stops: the door's hinge stops at 0 (shut); the knob at ±0.80 rad.

Real trial: any 24 x 80 in hollow-core prehung interior door with a 4-9/16 in jamb (for example Masonite or JELD-WEN at
a home store) and a Kwikset Tylo passage knob, the unit screwed to a floor-standing frame beside the station.

## Scoring
Read from the door's three joints each control step (`door_scoring.py`). The bolt is hidden in the door, so each sound
is judged two ways, and either counts: **by sight**, from the knob and the door's edge, as a real trial reads them off
video; and **by the bolt**, which stands in for the real trial's microphone (the bolt's motion is what makes the sound).

| Sound | By sight (knob and door) | By the bolt (the microphone's stand-in) |
|---|---|---|
| Click | the knob turned less than 36 degrees on any step on which the latch edge closes within its last 35 mm of the stop | the bolt pushed in more than 1.5 mm beyond what the knob holds |
| Snap | with the door shut, the knob turning back towards rest faster than 90 degrees a second (over one 0.05 s step) | with the door shut, the bolt coming out faster than 30 mm/s |

| Score | When |
|---|---|
| 0 | a gripper within 15 mm of the bolt or strike; or the latch edge meets the stop above 0.3 m/s (slam); or the door ends more than 3 mm off the stop |
| 0.25 | shut but not latched (bolt more than 5 mm in) |
| 0.5 | latched, with a click |
| 0.75 | latched without a click, but with a snap, or the door met the stop above 0.1 m/s |
| 1.0 | latched, no click, no snap, gentle; knob back within 5 degrees of rest; the door still; both grippers 3 cm off the knob; held 1 s |

`score()` never gives more than 0.75; 1.0 comes only from `success()` holding 1 s.

Where the numbers come from. The lip touches a free bolt from 3.2 degrees open, the latch edge 34 mm off the stop, and
pushes it up to 10.7 mm in; the cam holds 10.7 mm from 36 degrees (the plan-view check in the offline tests). The
official-sim demo held the knob at about 40 degrees through the close. At its steepest the cam lets the bolt out 19 mm a
radian, so 90 degrees a second is the bolt's 30 mm/s. The 30 mm/s limit was set from the simulator: the demo's 2 s
release peaks at 10 mm/s, turning the knob back in one step peaks at 49 mm/s.

**Real trial, without seeing the bolt:**
- **Latched:** a pull on the door without turning the knob does not open it.
- **Video:** 60 fps from a fixed camera square to the knob; a strip of tape across the knob's face as a pointer; floor
  marks 35 mm and 3 mm from the shut latch edge. The knob's angle and the edge's position are read frame by frame,
  speeds over 3 frames (0.05 s): that gives both sight checks and the door's speed at the stop.
- **Sound:** a microphone fixed to the latch jamb 10 cm above the strike, on the robot's side, in a closed room whose
  background (the robot powered and still, ventilation off) is under 40 dBA over the 10 s before each trial. A click or
  snap is heard when the A-weighted fast (125 ms) level peaks 10 dB or more above that background: a click while the
  door is still moving onto its stop, a snap once it is still on it.
- **Set up once, before the session:** five ordinary push-shuts must each exceed the 10 dB margin, and five closes with
  the knob held at its stop must not. The 36 degree turn is the modelled latch's; on the real door it is replaced by the
  least turn, in 2 degree steps, at which five slow knob-held closes are all silent.

## Results (official simulator, 2026-11-22)
yam-isaac-lab-arena at 0059d02 on the program's L40S launchable; task file SHA-256 prefix 2c2f76cb58d5; seed 0;
`object_jitter=0`; 20 Hz; 1,200-step limit.

| Run | Score | What happened |
|---|---|---|
| Scripted demo | **1.0** | success at step 294: knob gripped, turned (bolt held 11.9 mm in), door pushed shut at 0.03 m/s, knob eased back (bolt out at 10 mm/s) |
| Dream-1 (`dreamlabs/dream-1-beta`) | **0.0** | ran all 1,200 steps; both arms lifted and hovered near their bases and never reached the door, which stayed ajar |
| Probe: knob not turned (`DOOR_DEMO_TURN_DEG=0`) | 0.5 | the bolt rode the strike's lip (click at step 147), then latched |
| Probe: knob turned back in one step (`DOOR_DEMO_RELEASE_S=0.05`) | 0.75 | latched without a click, but the bolt came out at 49 mm/s |

Rerun with the revised scorer (sight and bolt checks; the task file unchanged), same settings, 2026-11-22 23:51 UTC:

| Run | Score | What happened |
|---|---|---|
| Scripted demo | **1.0** | success at step 294; the knob held at 40.4 degrees through the last 35 mm (36 needed) and went back at most about 32 degrees a second; no click or snap either way |
| Probe: knob not turned | 0.5 | click by sight at step 142 (knob at rest as the edge closed inside 35 mm), then by the bolt at step 147 |
| Probe: knob turned back in one step | 0.75 | snap both ways: the knob went back at about 150 degrees a second, the bolt came out at 49 mm/s |

## Run and score
```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=close_door_quietly.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Shut the door all the way so it latches, without making a sound." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames
```
Swap `--policy dream` for the Dream-1 run. Set `DOOR_DEBUG=<file>` to log the door's joints, the record and the right
hand's position every step.

## Offline checks (no simulator)
```bash
PYTHONPATH=<yam-isaac-lab-arena checkout>:. python -m pytest -q test_door_scoring.py test_mechanism_chain.py test_close_door_quietly.py
```
Needs pytest, NumPy and `usd-core`. The latch is checked in plan view at bolt height: a free bolt is pushed up to
10.7 mm in by the lip between 3.2 and 0.4 degrees open and fits fully out when shut; a bolt held flush passes at every
angle; a knob turned 36 degrees or more clears. The scripted demo's plan solves to under 1 mm of IK error, and replayed
through the scorer with the door following the hand it scores 1.0 (0.5 with the knob not turned).

## Files
- `close_door_quietly.py`: the task file (scene, record hook, scoring hooks, scripted demo).
- `door_scoring.py`: the scorer, NumPy only.
- `mechanism.py`: articulated fixed mechanisms for yam_arena task files, from robocurve#223 (MIT; see its header),
  extended here for links on moving links, links that touch the base, start positions and turned boxes.
- `test_door_scoring.py`, `test_mechanism_chain.py`, `test_close_door_quietly.py`: offline tests.
- `tools/check_reach.py`: offline reach check of the knob through the close and release.

## Known limits
- The top camera sees the knob and the latch edge only at its right border: the door stands beyond the table. The
  right wrist camera shows the grip, the turn and the close.
- The bolt is square-ended and the slant is on the strike's lip (real latches have it on the bolt). Spring rates, the
  knob's 45 degree turn and the 13 mm throw are estimates until measured on the real set.
- The hinge axis is on the door's centre plane, not at its face corner.
- No sound is simulated: the microphone's click and snap are read from the bolt's motion, beside the knob and door checks
  a real trial reads off video. The two agree on the modelled latch (36 degrees holds what the lip asks; 90 degrees a
  second is the cam's 30 mm/s); on a real latch the turn is measured on the door before the session, as above.

## Licence
MIT. `mechanism.py` keeps its original MIT notice.
