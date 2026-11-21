# Wrap a burger to go in a foil-laminated sheet

Instruction: `Wrap the burger in the sheet for takeout.`

A burger replica sits crown up in the middle of a foil-laminated wrap sheet (37.5 cm front to back, 25 cm left to
right) laid on a prep board. A counter worker wraps it in a few seconds: the front edge up over the bun and creased
onto the crown, the back edge over that, then each side folded up flat against the bun to the height of the crown, closing the ends. The sheet has no glue and nothing
ties it shut. Only its creases hold the parcel, so every fold has to be made sharp, in an order that keeps the
earlier folds in place, and the hands have to let go and leave it standing wrapped.

## Who does this and why it matters

- **Occupation:** fast food and counter workers (O*NET 35-3023.00), 3.85 million US jobs (BLS OEWS, May 2025).
  Their core task list includes "Wrap menu items such as sandwiches, hot entrees, and desserts for serving or for
  takeout." Every takeout burger is wrapped by hand, so this is repeated all day at every counter.
- **The material:** foil-laminated (dead-fold) sandwich wrap is aluminium foil bonded to paper or tissue. It keeps a
  crease where it is folded and does not spring back, which is why it is used: a wrap stays shut without tape.
- **Why it is hard for a robot:** a thin sheet has to be picked up by its edge where it hangs off the board, carried
  over a 5 cm bun without dragging the burger, laid down past the middle and creased, and let go. A 25 cm edge lifted
  by one corner skews and leaves the bun showing, so the front and back edges each take both hands at their two
  corners. The sides are folded last, through three stacked layers: each side has to be gripped by all its stacked
  edges where they hang off the board and stood up against the bun to crown height without pulling the front and
  back folds open or dragging the first folded side back down.
  Finally both hands have to let go and move away without the parcel springing open.

## Scene (sim and real)

Counter top = reference; x forward from the line through the arm bases, y left of their midpoint (cm).

| Object | Sim | Real |
|---|---|---|
| Prep board | fixed box 28 x 16 x 1.5 cm, centred at x 28, y 0, white | white HDPE cutting board 28 x 16 x 1.5 cm, clamped to the counter |
| Wrap sheet | reduced position-based sheet drawn with a 2.5 cm green and silver check, 37.5 cm along x by 25 cm along y, centred on the board (hangs 4.75 cm over the front and back edges, 4.5 cm over the sides) | checker-print foil-laminated burger wrap (dead-fold, green and silver check on the foil side), cut to 37.5 x 25 cm, foil side up, 37.5 cm front to back |
| Burger | free rigid cylinder 9 cm wide, 5 cm tall, 120 g, crown up at the sheet centre (crown edge rounded 1.2 cm for the sheet) | sesame-bun cheeseburger display replica (foam or resin) of the same size, weighted to 120 g |

## Scoring (end of trial)

**Score 0** if any of these holds (a move, tip or tear counts if it happens at any time in the run):

- the burger has moved more than 3 cm on the board or tipped more than 20 degrees;
- the sheet is torn;
- a gripper still holds the sheet at the end;
- the grippers have not both been open (finger gap 2 cm or more) and at least 3 cm from the sheet for the last 3 s.

**Otherwise**, judged as the parcel stands over those last 3 s (the lowest value counts):

| Score | End state |
|---|---|
| 0.25 | no part of the crown is visible from above |
| 0.5 | also the middle of the sheet's front edge and of its back edge is raised to the top of the bun (within 1.5 cm of it), and no bun is visible from the front or the back |
| 0.75 | also one side is folded up: the midpoint of that side edge is 2.5 cm or more above the board |
| 1.0 | both sides folded up, and no bun is visible from any side |

In simulation, "visible" is checked by casting rays from sample points just inside the burger's top and sides (top
within 92% of its radius; sides at 12-88% of its height) straight up and straight out toward the front, back, left
and right; a point is hidden when the sheet crosses its ray. The front and back edges are "raised to the top" when at least 80%
of the middle 9 cm of each is higher than 1.5 cm below the top of the bun (a height test; with the sheet flat on the
board at the start, an edge only gets that high by being carried up over the bun). A side is "up" when the midpoint of that side
edge (the median of the three sheet nodes in its central 2.5 cm) is 2.5 cm or more above the board.

**Real-trial check:** photos from straight above and from the front, back, left and right after the hands have been
clear for 3 s; a ruler held beside each side for the 2.5 cm; tape marks on the board around the burger's footprint
for the 3 cm move limit; the video for release and clearance, and for the front edge's height, judged at the last
frame it is visible as the back edge covers it (once covered, the back fold holds it down). "Visible" uses the same
zones as the simulation, marked on the replica: the crown within 92% of its radius and the bun sides from 12% to 88%
of its height.

## Sim model

The burger and board are ordinary rigid bodies. The sheet (sheet.py) is a reduced position-based model stepped once
per control step from the rigid simulator's burger pose and the measured gripper poses and jaw openings. It exerts
no force on the robot or the burger: the fingers feel no drag from a lifted sheet, and the sheet cannot pull or tip
the burger (a real sheet pulled hard enough would slide it; here a hard pull tears the sheet instead).

- 1.25 cm grid of nodes; in-plane springs stiff in tension that crumple (shorten plastically) under compression.
- Bending springs with fold memory: past a small yield band a bend becomes a permanent crease, as dead-fold wrap
  does; pulling a fold flat again unfolds it.
- Contact with the burger (cylinder with a 1.2 cm rounded crown edge), the board, the counter and the finger pads,
  with friction.
- Grasp: a jaw that closes (gap under 12 mm after being open) grips the sheet edge(s) between its fingertips: one
  edge, or several if earlier folds stacked them. A jaw that is already shut presses and picks nothing up. A gripped
  node lets go when the jaw opens past 23 mm or is pulled 3.5 cm from the fingers.
- No sheet-on-sheet collision: each node is layered by the fold in which it first left the board, so later folds lie
  over earlier ones.
- Tear: stretched past 1.8x for 0.25 s, or 3x at once.

The model is uncalibrated: it stands in for a dead-fold sheet's behaviour, not its exact stiffness.

## Requirements

yam-isaac-lab-arena and Isaac Lab-Arena 0.3.1 (the YAM launchable), plus NVIDIA Warp (bundled with Isaac Lab).
sheet.py must sit next to task.py.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 3600 \
  --instruction "Wrap the burger in the sheet for takeout."
```

Run limit 3,600 steps (180 s at 20 Hz). The layout is fixed with object_jitter=0; the sheet is always laid centred
on the burger.

## Demo video

The full-score demo follows the fixed layout. Both hands grip the front edge's corners where it hangs off the board,
lift it in an arc over the bun, lay it 4.5 cm past the middle and let go; one closed hand presses the new crease on
the crown edge. The back edge goes over the same way, on top. Both closed hands press the two open ends of the tube
flat onto the board. Then the right hand grips the right side's stacked edges where they hang off the board, stands
the side up against the bun to crown height, lets go and presses it; the left hand does the same on the left.
Both hands park open, away from the parcel. It ends at 1.0 in 1,038 steps (about 52 s).

Shortcuts and partial wraps checked in simulation, each scored by the same rules:

| Variant | Score |
|---|---|
| doing nothing | 0 |
| front edge only (bun half covered) | 0 |
| one arm only (each edge lifted by one corner) | 0 |
| full wrap, then gripping the back edge again and holding it | 0 |
| front and back over, sides left flat (ends not folded) | 0.5 |
| front and back over, only one side folded up | 0.75 |
