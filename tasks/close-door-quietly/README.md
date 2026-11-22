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
Read from the door's three joints each control step (`door_scoring.py`):

| Score | When |
|---|---|
| 0 | a gripper within 15 mm of the bolt or strike; or the latch edge meets the stop above 0.3 m/s (slam); or the door ends more than 3 mm off the stop |
| 0.25 | shut but not latched (bolt more than 5 mm in) |
| 0.5 | latched, with a click: the bolt was pushed in more than 1.5 mm beyond what the knob holds |
| 0.75 | latched without a click, but the bolt came out faster than 30 mm/s, or the door met the stop above 0.1 m/s |
| 1.0 | latched, no click, no snap, gentle; knob back within 5 degrees of rest; the door still; both grippers 3 cm off the knob; held 1 s |

`score()` never gives more than 0.75; 1.0 comes only from `success()` holding 1 s. The 30 mm/s snap limit was set from
the simulator: the demo's 2 s release peaks at 10 mm/s, turning the knob back in one step peaks at 49 mm/s.

**Real trial:** a pull on the door without turning the knob (latched); a dry-erase stripe across the bolt's face
towards the strike, wiped where it scraped (click); 60 fps video for the knob's return and the door's speed at the
stop.

## Results (official simulator, 2026-11-22)
yam-isaac-lab-arena at 0059d02 on the program's L40S launchable; task file SHA-256 prefix 2c2f76cb58d5; seed 0;
`object_jitter=0`; 20 Hz; 1,200-step limit.

| Run | Score | What happened |
|---|---|---|
| Scripted demo | **1.0** | success at step 294: knob gripped, turned (bolt held 11.9 mm in), door pushed shut at 0.03 m/s, knob eased back (bolt out at 10 mm/s) |
| Dream-1 (`dreamlabs/dream-1-beta`) | **0.0** | ran all 1,200 steps; both arms lifted and hovered near their bases and never reached the door, which stayed ajar |
| Probe: knob not turned (`DOOR_DEMO_TURN_DEG=0`) | 0.5 | the bolt rode the strike's lip (click at step 147), then latched |
| Probe: knob turned back in one step (`DOOR_DEMO_RELEASE_S=0.05`) | 0.75 | latched without a click, but the bolt came out at 49 mm/s |

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
- No sound is simulated: the click and the snap are read from the bolt's motion.

## Licence
MIT. `mechanism.py` keeps its original MIT notice.
