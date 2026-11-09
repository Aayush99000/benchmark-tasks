# Set a board up on a drill press to drill through its middle, with scrap backing under the exit

A bench drill press stands on the table: its table has a hole straight under the spindle, and the bit hangs 10 cm
above it. An oak board and a piece of scrap pine lie beside it. The instruction only says to set the board up so the
bit will drill straight down through its middle. Woodworkers know the rest: a piece of scrap goes under the board
where the bit comes out, or the bit splinters the underside as it breaks through and runs into the table. So the
robot has to put the scrap under the bit first, then the board on it, centred on the spindle's axis. The bit is in the
way of a grip from above, so each piece has to be held towards one end and carried in under it from the side.

## The real step
Before drilling through a workpiece on a drill press, woodworkers and machinists put scrap under it where the bit will
come out. The RIDGID DP15501 drill press operator's manual ([Positioning Table and Workpiece, p. 31](https://www.manualslib.com/manual/260985/Ridgid-Dp15501.html?page=31)):
"Always place a piece of backup material (wood, plywood, etc.) on the table underneath the workpiece." Done by:
cabinetmakers, carpenters and machine operators.

What goes wrong in real life: without backing, the bit splinters the underside as it breaks through and runs into the
table (here, through its hole). A look under the bit before drilling checks it.

What we simplified: the drill press is a fixed model that doesn't run or lower, and no hole is drilled: only the
set-up is scored. The pieces are rigid blocks, lighter than real wood because the arms are weak.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=drill_backer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the oak board up on the drill press so that the bit will drill straight down through the middle of the board." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/drill-backer-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=drill_backer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the oak board up on the drill press so that the bit will drill straight down through the middle of the board." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/drill-backer-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/drill-backer-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the set-up has been right, and still, for 1 s;
otherwise it scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| press table | fixed, grey, 26 x 26 cm, 3 cm thick, with a 3 cm square hole straight under the spindle | centred at (26, 0) |
| column | fixed, dark, 7 cm across, rising behind the table | (46, 0) |
| head, chuck | fixed; the head reaches forward over the table to 3 cm in front of the spindle, 30 cm above the table | over (26, 0) |
| drill bit | fixed, 8 mm across, on the spindle's axis, its tip 10 cm above the press table | (26, 0) |
| board | loose oak board, 24 x 6 x 2 cm, 100 g, lying along x | (25, 30), on the station table |
| backer | loose scrap pine, 24 x 6 x 2 cm, 70 g, lying along x | (25, 21), on the station table |

Both pieces have friction 1.2 static, 1.0 dynamic. Start: both grippers open at the ready pose. One fixed layout:
`-E object_jitter=0`, so the seed doesn't change the start.

## Scoring
Measured from the spindle's axis, across the table. The board must lie flat (within 3 degrees) with its middle within
1 cm of the axis for any credit.

| Score | When, at the end |
|---|---|
| 1.0 | the board flat on the scrap pine, its middle within 3 mm of the axis; the pine flat on the press table, the axis at least 2 cm inside every one of its edges (the bit comes out into it); both still, held for 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.5 | the board on the pine under the bit, but its middle 3 mm to 1 cm off the axis, or the axis less than 2 cm inside the pine's edge |
| 0.25 | the board under the bit straight on the press table, with nothing under it |
| 0 | anything else, including the pine laid on top of the board |

## The scripted demo
The left arm does it all. Each piece is gripped 10 cm from its middle, at the end nearer the arms (the fingers close
across its 6 cm width), lifted, carried to the left of the press, lowered to 2.5 cm above where it will rest and
carried in sideways under the bit, then set down and let go: first the scrap, centred under the bit, then the board on
it. The gripper stays 10 cm in front of the spindle throughout, clear of the chuck, and of the head with the arm's
upper links. Each piece's aim is corrected by where it came to rest on the official stack when aimed straight at the
axis (up to 4.2 mm across).

## Runtime and seeds
- The scripted demo's plan takes 30 s of simulated time (600 steps at 20 Hz), within the 2,400-step limit.
- Official runs (2026-11-08): the demo 1.0, ending on success at 28 s, the board's middle 0.5 mm from the axis; with
  both pieces aimed 2 mm off (towards +x and +y, then the other way), also 1.0 (the board 2.6 mm off at most);
  Dream-1 0.0 over the full 2,400 steps.
- The scoring and reach checks run without the simulator: `python -m pytest -q test_drill_backer.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
