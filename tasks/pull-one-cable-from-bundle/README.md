# Pull one decommissioned cable out of a clipped bundle

A white open-ended cable tray (40 cm long, left-right in front of the robot, 8 cm wide inside, 3 cm walls) holds four
6 mm cables lying side by side, held in by two black bridge clips spanning the tray. Three grey cables are "live";
the blue one, second from the robot (not the nearest one), is decommissioned. All four leave the tray's right end and
end in identical black plugs on the table, staggered so that neither the plug nearest the robot (grey A) nor the
one sticking out furthest (grey C) is the blue one. The robot must find the blue cable, draw it out lengthwise
through both clips and out of the tray's right end (its left end lies inside the tray, so it cannot simply be lifted
out: the clips hold it down), lay it with its plug on the table entirely outside the tray (not hanging off the
table edge, not lying across the live cables or their plugs) and let go, while every grey cable stays under both
clips with its plug within 3 cm of where it started. The cables are 15 mm apart, so a regrasp of the blue cable
near the tray end has to avoid the neighbouring cables.

The scripted demo is bimanual (the left arm holds the live cables down while the right arm pulls), as a technician
would do. In the simulation the loose, untied grey cables are barely dragged along (< 5 mm), so the hold matters
little and one arm could succeed; what goes wrong in a careless pull is the blue cable itself: pulled straight towards
the robot across the tray's end, its free end rides up onto the live cables and stays there (variant `yank`, 0). The
skill tested is identifying the dead cable, drawing it out through the clips with pulls and regrasps that fit between
cables 9 mm apart and do not leave it on them, then laying it down where it rests on clear table.

## How this differs from the closest accepted task

The closest accepted task is "Pull a cable out to a length mark": one cable, free in an open feed channel, is drawn
out until its end reaches a ruler mark, and the skill scored is controlling pull distance. Here the pull length is not
scored at all. The ability tested is selective removal from a live bundle: find the one dead cable among identical
ones, get it out from under restraints that hold the whole bundle, and clear it onto free surface without pulling out
or ending up on the live cables. The length-mark task has no other cables, no restraint over the cable and no
placement. In detail:
- **Which cable**: four cables with identical plugs, and the target is neither the nearest plug nor the one sticking
  out furthest, so it must be identified by colour and traced, not picked by position. Pulling a grey cable scores 0
  (shortcut variant `nearest`).
- **Through restraints**: the blue cable is held down under two bridge clips and its far end lies inside the tray, so
  it cannot be lifted out; it has to be drawn lengthwise under both clips and out of the tray's open end.
- **Live neighbours**: any grey cable pulled from under a clip, or its plug moved more than 3 cm, scores 0, so grasps
  and regrasps must fit between cables 15 mm apart (centre to centre); and the blue cable must not end up lying on
  them, which a careless pull towards the robot does (variant `yank`).
- **Where it ends**: the removed cable and its plug must end resting on clear table outside the tray: not on the live
  cables, not hanging off the edge and not held (variants `yank`, `propped`, `in_air`).

## Why this task (domain: impact on AI timelines)

Building and refreshing the data centres that AI training and serving run in means a steady stream of moves, adds and
changes in the cable trays above the racks: cables for retired links are disconnected and must come out of a tray
whose other cables stay live. The US National Electrical Code requires this: the accessible portion of abandoned
communications cables must be removed (NFPA 70, 800.25; the IT-room article has the same rule for abandoned
interconnecting cables, 645.5(G), unless they are in a raceway). A cable is abandoned when it is not terminated at
equipment and not tagged for future use (NEC definitions, e.g. 645.2). Left in place, dead cable fills pathways
(ANSI/TIA-569 guidance plans trays at about 25 % calculated fill, 50 % maximum) and obstructs cooling airflow;
data-centre operators call its removal "cable mining". The task is the core manipulation of that job: identify the
one dead cable in a clipped live bundle, draw it out lengthwise through the clips without disturbing its neighbours,
and leave it lying on the work surface, not dangling. A robot that can do this safely in a live tray is part of
automating data-centre build-out and maintenance labour. In a data centre the tray runs overhead or under a raised
floor, and the technician works it from a ladder or through a lifted floor tile, with the open tray in front of them
at arm's reach and the work done from above, as here. The scene keeps what constrains the work at the tray: the tray
walls, clips holding the bundle down, live cables 9 mm away on both sides, identical plugs, an open tray end the cable
must be drawn out of, and only clear surface to lay the removed cable on.

Sources:
- NFPA 70 (National Electrical Code) 800.25 and 645.5(G); wording and definitions as quoted in
  https://www.ecmweb.com/national-electrical-code/code-basics/article/20903841/understanding-requirements-for-it-equipment
  (645.2, 645.5(G)) and https://codebookcity.com/codearticles/nec/necarticle800-1.htm (2023 NEC 800.25)
- Tray fill (TIA-569 / BICSI): https://open-exam-prep.com/study-guides/bicsi-dcdc/ch10-communications-cabling/sec04-pathways-fill-ratios-emi-separation
- Airflow and cable mining: https://www.energy-manager.ca/feature-overhead-cabling-saves-energy-in-data-centres-1278/
  and https://dataspan.com/?p=2979

## What changed in this revision

Review feedback: "Scoring (not met): The simulator treats any blue cable point within the tray footprint and a broad
height range as supported, including points resting on grey cables. What to change: The nonzero-score support
requirement from the previous feedback is still open in the simulator: cable points anywhere within the tray footprint
and a broad height range count as supported, even if they rest on the grey cables rather than the tray. Update the
simulator check for the cable and plug so partial credit requires actual support by the table or tray, as specified."

- "Supported" now means contact with a real support surface. For every blue cable point the scorer looks up the
  surface directly below it (table top, tray floor between the walls, or a wall top) and requires the point's centre
  to be within 3 mm of lying on that surface (at most cable radius + 3 mm above it). A cable lying on a 6 mm grey cable
  sits 6 mm too high and fails. On top of that, a point is never supported if a grey cable or grey plug lies directly
  beneath it (overlapping it in top view and lower), whatever its height.
- The plug gets the same test with its real shape: its long axis must be level (within about 6 degrees) with both
  ends' undersides within 3 mm of the surface below them (or it stands on end on a surface), and no grey cable or plug
  may lie beneath it. A plug tilted up on something, or lying across the live cables in the tray, is not supported.
- Partial credit (0.5) therefore needs the blue cable out of both clips with every point and the plug resting on the
  tray floor or the table. New variants, run on the final code (table under Scripted demo): `yank` pulls the blue
  cable out of both clips the careless way and leaves its free end lying on the grey cables inside the tray, which the
  old rule counted as supported: now 0. `propped` sets the plug down across two grey cables inside the tray: 0.
  `partial` stops pulling once the cable has left both clips, its free end still lying on the tray floor: 0.5.
- The full-credit test (whole cable and plug resting on the table outside the tray) uses the same contact test.
- Demo re-recorded on the final code (still 1.0); the instruction and scene are unchanged.

Earlier feedback, still addressed:
- Human reviewer: "Change the domain to Impact on AI timelines and explain the data-center cable-removal context.
  Update scoring so the blue cable and plug must be supported by the table or tray for any nonzero score, and must be
  fully resting on the table outside the tray for full credit." Domain and context as above; scoring as below.
- "A robot can pull the blue cable off the table and still earn full credit": a cable or plug beyond the table's
  edges has no support surface under it and scores 0.
- "Demo should only use observable information": the demo's motions use only the top camera image, the arms' own
  poses and the fixed layout.

## Building the real station

All parts are ordinary and cheap (well under US$50). Tray: 3D-print it from the setup's dimensions (a 88 x 400 x 3 mm
floor plate with two 4 mm walls 33 mm tall along its long sides, open at both ends) or glue it from 3-4 mm white
plastic sheet; screw or tape it to the table. Clips: two 80 x 20 x 5 mm black plastic bars (3D-printed or cut
acrylic) resting on 10 mm spacers on the tray floor at each wall, glued, so the cables pass under them with 10 mm
clearance. Cables: 6 mm round PVC-jacketed cable (e.g. LiYY 4 x 0.5 mm2, grey and blue) cut to the setup's lengths;
plugs: black 30 x 10 x 10 mm blocks (3D-printed) glued on the cable ends. Tape marks on the tray floor and the table
give the reset positions.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- No third-party assets: every object (tray, clips, cables, plugs) is built from primitives in `task.py`.
- `cablekit.py`, `cabledemo.py` and `vision.py` are in this folder (the cable simulation, the scripted-demo runner and
  the demo's top-camera helpers).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "The blue cable in the white tray is decommissioned. Pull it out of the tray through the two black clips and lay it, with its plug, on the table completely outside the tray, then let go. The three grey cables are live: they must stay in the tray under both clips."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Closed-loop on camera estimates only: the demo's motions use the top camera image, the arms' own poses and the
fixed layout from the setup (tray, clips, where to hold the bundle, the blue cable's length); it never reads cable or
plug poses from the simulator. The left arm presses the live cables onto the tray floor at a fixed spot left of the
blue cable's end (fingers straddling the bundle). The right arm parks low and to the right, out of the top camera's
view, and the demo finds the blue plug in the image (the blue run outside the tray gives its line, the dark plug at
its end its position). The right arm grasps it, pulls it up and to the right (towards the robot) and lets go; it
parks again, and the demo traces the blue cable in the image outward from the tray's right end and re-grasps it at
the first point at least 3 cm beyond the tray and at least 3 cm from every grey or dark pixel (other cables, plugs);
it pulls again until the camera sees no blue over the tray. The pulled-out cable's free end then lies across a grey
plug, so the left arm lets go and the right arm lays the cable flat on clear table: the demo traces the blue cable
in the image from its free end, picks it up at the first point 3 cm clear of anything grey or dark, lifts it 13 cm,
carries it to a line on the robot's side of the tray and the grey plugs (x 14 cm; the plug end trails), lowers it
while moving towards the robot so the free end is laid out sideways rather than folded under, and lets go.
Success (score 1.0 held for 1 s) ends the episode at step 525 (~26 s of the 2,400-step limit); task_score 1.0. At
the end every blue cable point lies within 0.0-2.2 mm of lying flat on the table and the plug centre 5.0 mm above it
(lying flat); grey plugs moved at most 1 mm. `demo/` holds its videos, `setup.png` and `result.json`.

Shortcut variants (test only; `BUNDLE_CHEAT=<name> python cabledemo.py task.py out`; they may read the simulator,
they only have to show what they score), each run once on the final code.

| Variant | What it does | Score (steps) | Why |
|---|---|---|---|
| `yank` | the careless way: no hand holds the live cables; one hand takes the blue plug and pulls it straight towards the robot across the tray's open end, then lets go | 0.0 (250) | the cable comes out of both clips, the plug lies on the table and the grey cables stay put (grey A moved 4 mm), but the blue cable's free end is dragged up onto the live cables inside the tray and stays there (up to 9 mm above lying on the tray floor; a cable lying on another is 6 mm up): not supported by the tray. The old height-band rule counted those points as supported |
| `propped` | pull the blue cable along its row out of both clips, then carry the plug back and set it down across grey cables B and C inside the tray, right of the right clip, and let go | 0.0 (322) | the plug rests on the live cables (centre about 12 mm above the tray floor; 5 mm when lying on it), not on the tray; carrying it back also drew the blue cable back under the right clip |
| `partial` | pull the blue cable straight along its row until it has left both clips, its free end still lying on the tray floor; lay it down and let go | 0.5 (260) | every point of the cable and the plug is in contact with the tray floor or the table (cable centres 0.0-3.1 mm above lying on the table; the tray floor is 3 mm above it), and 81 % of the cable lies on the table outside the tray |
| `in_air` | the demo's extraction and lay-out, then the right hand picks the plug up 12 cm and keeps holding it | 0.0 (712) | not let go; the plug is 11 cm above the table |
| `nearest` | pull the cable nearest the robot (grey A) the way the demo pulls blue | 0.0 (836) | grey A disturbed: its plug moved 26 cm from its mark |
| `nothing` | do nothing | 0.0 (100) | blue still under both clips |

## Scoring (`score()`, evaluated at the end; `success()` = score 1.0, must hold 1 s)

Positions come from the simulator: every blue cable point (segment centres plus points every 4 mm between them, both
tips), the blue plug's centre and its long axis (the plug is welded coaxially onto the cable's last segment), and the
same for the grey cables and plugs. The table top is at z = TOP and spans x -0.19..0.91, y -0.65..0.65. Cable radius
R = 3 mm; the plug is 30 x 10 x 10 mm.

- **Support surface** under a point (top view): a wall top (WALL_TOP) over either tray wall, the tray floor (FLOOR =
  TOP + 3 mm) between the walls along the tray's 40 cm length, the table top (TOP) elsewhere inside the table's
  edges; none beyond the table's edges.
- **Grey beneath**: some grey cable point lies within 5 mm of the blue point in top view (the two 6 mm cables
  overlap) and its centre is at least 2 mm lower, or a grey plug overlaps it in top view and is at least 2 mm lower.
  Then the blue cable is lying on a live cable or plug there.
- A blue cable point is **supported** if its centre is within 3 mm of surface + R (in contact with the surface below
  it) and no grey item is beneath it. A point lying across a grey cable is 6 mm too high and also has grey beneath it.
- The plug is **supported** if it lies level (its long axis within about 6 degrees of horizontal) with both end faces'
  undersides within 3 mm of the surface below each, or stands on end with its lower face within 3 mm of the surface,
  and no grey cable or plug is beneath it. A plug tilted up on something, or resting across grey cables in the tray,
  is not supported.
- A blue cable point **lies on the table** if it is supported, the surface below it is the table top, and it is
  inside the table's edges by R and outside the tray's footprint (walls included, plus R). The plug **lies on the
  table** if it is supported, both end faces are over the table top outside the tray's footprint and at least 5 mm
  inside every table edge (no part overhangs).
- The blue cable is **still** if its segments' mean speed is under 2 cm/s.

Score:

- **0** if either gripper (point between the finger pads) is within 4 cm of any cable or plug (not let go).
- **0** if any grey cable no longer passes under both clips, or any grey plug centre is more than 3 cm
  (horizontally) from its starting position.
- **0** unless the blue cable is still and every point of it and its plug is supported (so a cable or plug dropped
  or dragged off the table edge, hanging, held or propped up, or lying on a live cable or plug, inside or outside the
  tray, scores 0).
- Otherwise **1.0** if every blue cable point and the plug lie on the table (the instructed end state); **0.5** if
  the blue cable passes under neither clip but is not yet all on the table outside the tray (e.g. part of it still
  lies on the tray floor); **0** if it still passes under a clip (e.g. untouched).
- `distance()`: blue cable length not lying on the table (m), + 0.05 if its plug is not; 1 m if a grey cable is
  disturbed.

## Real-trial check for "lying on the table"

Checked from two photos taken after the robot has let go and nothing has moved for 1 s (the trial video shows the
stillness): a top photo and a side photo taken with the camera at table level, looking along the tray. Top photo:
(1) no part of the blue cable or its plug is over the tray or beyond a table edge; (2) nowhere does the blue cable or
its plug cross over or lie on a grey cable or grey plug. Side photo: (3) the blue cable and plug lie on the table with
no visible gap under them (nothing propped up, held or hanging). All three: 1.0. Otherwise 0.5 if the blue cable is
out of both clips (top photo), crosses no grey cable or plug, and every part of it lies on the table or on the tray
floor with no visible gap (side photo). Grey plugs are checked against their tape marks with a ruler (3 cm), and each
grey cable must still pass under both clips (top photo). The sim's contact tolerance (3 mm) is about the smallest gap
the side photo shows clearly under a 6 mm cable.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). Wall time per trial depends heavily on GPU load.
- Layout is fixed with `object_jitter=0`; cables are never jittered (they always start straight), so every trial
  starts identically.
