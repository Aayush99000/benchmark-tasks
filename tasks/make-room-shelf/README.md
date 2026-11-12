# Shelve a carton that fits no single gap: move the boxes together first

A low shelf holds four product boxes, spread along it with a few centimetres between each other and the end walls.
A brown carton waits on the table in front of it. The instruction only says to put the carton on the shelf and that
the four boxes have to end up on it too, standing. The carton is 18 cm long, and the shelf has 23 cm free in all, but no single gap
is wider than 5.5 cm. So the robot has to work out that the boxes must be moved together first, and pack them
tightly: with them packed, there are 5 cm to spare in all along the shelf.

## The real step
Stockers put new merchandise on shelves that already hold stock, and the free space is rarely in one piece: they
slide the products standing there together to open room for the new case. O*NET 53-7065.00 (Stockers and Order
Fillers): "Stock shelves, racks, cases, bins, and tables with new or transferred merchandise." Done by: stockers and
order fillers.

What goes wrong in real life: a case set on top of the stock or left on the floor blocks the aisle, and products
left off the shelf or lying on their side can't be sold from it. A photo of the shelf shows the result.

What we simplified: the shelf is a fixed board with end walls and a back lip, with no shelf above it; the boxes and
the carton are rigid blocks, lighter than real stock because the arms are weak.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=make_room_shelf.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the brown carton on the shelf. The four boxes already on the shelf have to end up on it too, standing up." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/make-room-shelf-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=make_room_shelf.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the brown carton on the shelf. The four boxes already on the shelf have to end up on it too, standing up." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/make-room-shelf-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/make-room-shelf-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the carton and all four boxes have been on the
shelf, still, for 1 s; otherwise it scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| shelf | fixed, white melamine-faced particleboard: a board 2 cm thick, 13 cm deep, 45 cm long between two grey end walls, a grey lip along its back; walls and lip 6 cm above the board | board x 26.5 to 39.5, y -22.5 to 22.5 |
| box_1 | loose red product box (printed paperboard), 6 cm deep, 5 cm wide, 8 cm tall, 120 g | (33, 17) |
| box_2 | loose green product box (printed paperboard), 6 x 6 x 8 cm, 120 g | (33, 6) |
| box_3 | loose yellow product box (printed paperboard), 6 cm deep, 5 cm wide, 8 cm tall, 120 g | (33, -4.5) |
| box_4 | loose blue product box (printed paperboard), 6 x 6 x 8 cm, 120 g | (33, -15.5) |
| carton | loose brown corrugated-cardboard carton, 7 cm deep, 18 cm long across the table, 10 cm tall, 250 g | (18, 0), on the table |

The gaps from the left end wall, between the boxes and to the right end wall are 3, 5.5, 5, 5.5 and 4 cm. Every
loose object has friction 1.2 static, 1.0 dynamic. Start: both grippers open at the ready pose. One fixed layout:
`-E object_jitter=0`, so the seed doesn't change the start.

## Scoring
On the shelf means resting on the board itself (the lowest corner within 3 mm of the board's top, so not on another
box or on the carton) with every corner inside the board's front edge, back lip and end walls, to 2 mm. Each box must
end up upright (within 10 degrees); the carton may rest on any face (within 10 degrees of flat). Still means moving
under 1 cm/s and turning under 0.5 rad/s.

| Score | When, at the end |
|---|---|
| 1.0 | the carton on the shelf and all four boxes on it, upright, every object still, held for 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.25 | all four boxes on the shelf, upright, with a stretch of at least 18 cm that no box reaches into, but the carton not on the shelf (and not off the table) |
| 0 | any box off the shelf, on another box or on the carton, or lying over |

Only the end of the trial counts: a box may be lifted off the board while room is made, as long as it is back on the
shelf, upright, at the end. A carton set on top of the boxes, or hanging over the board's front edge, isn't on the
shelf. A look at the shelf checks it.

## The scripted demo
Every grip closes across the table, over a box's 6 cm depth or the carton's 7 cm, so no finger goes into a gap
between two boxes. The left arm lifts the red box and sets it 4 mm from the left end wall, then the green box 4 mm
from the red one, and moves back over its own side. The right arm, starting one move later so the two grippers are
never both near the middle, does the same with the blue and yellow boxes at the right end wall. That opens 21.4 cm in
the middle. Then the left arm lifts the carton off the table, carries it in from the front 2 cm above the board,
between the packed boxes (1.7 cm clear of each), lowers it to 6 mm above the board and lets go.

Boxes are carried only 2.5 cm above the board, and no box passes over another: with the grip turned across the table
the middle of the shelf is out of reach much higher. Empty-handed, the fingers come and go 12 cm above the board, their
tips clear of every box top: an arm that moved off lower swept a box off the end of the shelf.

## Runtime and seeds
- The scripted demo's plan takes 58 s of simulated time (1,158 steps at 20 Hz), within the 2,400-step limit; it ends
  on success at about 55 s, once the carton has been let go and has rested on the shelf for 1 s.
- Official runs (2026-11-08): the demo 1.0, the boxes landing within 2 mm of their marks and the carton 0.4 mm from
  its aim, flat; with the carton aimed 1 cm to either side, also 1.0 (it landed 9.8 and 10.1 mm off centre); Dream-1
  0.0 over the full 2,400 steps (in its video it turns the carton about on the table; the four boxes end where they stood).
  That Dream-1 run was given the first wording of the instruction ("…have to stay on it, standing up."); the revision
  (2026-11-12) changed it to "…have to end up on it too" and scores the end state only. A rerun with the new wording
  will replace it.
- The scoring and reach checks run without the simulator: `python -m pytest -q test_make_room_shelf.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
