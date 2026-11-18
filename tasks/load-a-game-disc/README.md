# Open a game case, take the disc out and load it into a slot-loading console

A games console stands upright on the table, with a slot-loading disc drive whose slot is in its front face. Beside
it, a game case lies shut on a storage box with the game's disc inside. The robot has to open the case, take the disc
out and put it into the console.

**Instruction:** "Open the game case, take the disc out and put it into the console."

Three real mechanisms make this a two-handed, millimetre-precise job:

- **The case's catch.** A game case snaps shut. Here the catch lets go when the cover's lip is pulled up with 1.2 N.
  The whole case with its disc weighs 86 g, so a pull of 0.42 N at the lip already lifts that edge of the case off the
  box: pulling the lip alone picks the case up, still shut. One hand has to hold the tray half down while the other
  lifts the cover's lip. The cover then has to be swung up and over the spine to lie open beyond the case.
- **The case's hub.** A game case holds its disc on a hub with spring teeth that grip the edge of the disc's centre
  hole. The disc comes off only while the hub's centre button is pressed, which draws the teeth in. Lifting the disc
  without pressing the button just lifts the whole case with it (and with a real disc, risks cracking it). So one
  hand presses the button while the other lifts the disc.
- **The console's slot.** The slot is 5 mm wide behind a short flared mouth, for a disc 1.2 mm thick and 120 mm
  across. The disc has to go in edge first, standing upright, square to the slot and centred in it, until the drive
  takes it (7 cm in) and draws it the rest of the way.

The disc is wider (120 mm) than the gripper opens (94 mm) and only 1.2 mm thick, so it can only be held by its edge,
which the case's thumb cut-out leaves free, and it has to be turned from lying flat to standing on edge on the way.

## Where this comes up

- **Home entertainment.** Loading a game, film or music disc is an everyday task in homes with games consoles,
  Blu-ray players and car stereos with slot-loading drives, and it always starts with a shut case.
- **Care and assistance.** People with limited hand function find opening a snapped-shut case, the
  press-the-hub-and-lift step and lining a disc up with a slot difficult; it is a typical small request for a
  home-assistance robot.
- **Media handling in offices and libraries.** Libraries lend discs in the same cases, and archives load discs into
  slot and tray drives for reading.

## Potential impact

- **Opening a light snap-shut package.** The force that opens the catch is three times the force that lifts the
  package, so the hold and the pull have to happen together, a few centimetres apart.
- **Two hands with different jobs on one small object,** twice: holding the tray while lifting the cover, and
  pressing a 12 mm button on the hub while lifting a 1.2 mm disc by its edge.
- **Handling a thin, wide, fragile object by its edge,** and turning it from flat to upright in the fingers.
- **Millimetre alignment for insertion.** The slot leaves under 2 mm either side of the disc; a disc a few degrees
  off square catches the console's face.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: scene, the case's catch, the hub's teeth, the console's drive, scoring, the instruction |
| `disckit.py` | Builds the disc, the case (an articulation: tray, spine and cover on two hinges, hub button on a sprung slide, three teeth on radial slides) and the fixed parts |
| `demo.py` | The scripted demo, and its control runs |
| `test_scoring.py` | Offline tests of the scoring rule, the catch, the disc geometry and the slot rule (no simulator) |
| `case_template.svg` | 1:1 cutting template for the two openings a stock game case needs (see Real-world build) |
| `task.json` | The submission's details |

## Requirements

The yam_arena embodiment and Isaac Lab-Arena, as set up by the program's GPU instance image (an NVIDIA RTX GPU is
needed). `test_scoring.py` needs only Python with numpy, pytest and the `yam_arena` package.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the game case, take the disc out and put it into the console." \
  --scorer task_score --max-steps 2400
```

Run limit: 2,400 steps (2 minutes at 20 Hz). Second layout: set `DISC_LOT=B` in the environment (default `A`).

## Scripted demo

```bash
python demo.py task.py out_dir                  # layout A; DISC_LOT=B for layout B
python demo.py task.py out_dir --mode nohold    # control: lift the cover's lip without holding the tray down
python demo.py task.py out_dir --mode open      # control: open the case and stop
python demo.py task.py out_dir --mode nopress   # control: open the case, then lift the disc without pressing the hub
python demo.py task.py out_dir --mode partway   # control: everything, but stop with the disc 3.5 cm into the slot
```

1. The left hand, closed and leaning out to its own side, presses its fingertips on the tray's finger ledge, which
   shows through the recess in the cover's left edge: it holds the case down.
2. The right hand, level and closed, slides its fingertips under the cover's lip at the tray's thumb cut-out and
   lifts. The catch lets go. The left hand lets go of the ledge and moves clear.
3. The right hand raises the cover to 60 degrees. The left hand comes in level under the raised cover, takes it from
   the right hand, and pushes it up and over the spine. The cover falls open and lies flat beyond the case.
4. The left hand comes straight down on the hub, fingers closed, and presses its centre button 2.5 mm: the teeth draw
   in.
5. The right hand, level and pointing forward with its fingers one above the other, reaches through the case's thumb
   cut-out and pinches the disc's edge (18 mm of it), then lifts the disc straight up off the hub's teeth. The left
   hand lets go of the button.
6. The right hand lifts the disc clear of the hub, carries it, still flat, in front of the console and turns its
   wrist a quarter turn: the disc stands upright, square to the slot.
7. It slides the disc straight into the slot. Once 7 cm is in, the drive takes it; the hand lets go and backs away,
   and the drive draws the disc in.

The script is open loop: every pose comes from the layout constants in `task.py`, and object states are read only for
the result file. It does not read the catch's state: the hands move on a fixed schedule.

## Scoring

Tools: the trial video and a ruler.

- **0 for the whole trial** if at any time the disc touches the table (dropped), outside the console.
- Otherwise:
  - **1.0** once the drive has taken the disc: it went in upright (within 10 degrees), centred in the slot (within
    2.5 mm across it and 12 mm up or down) and 7 cm deep, and was drawn in;
  - **0.75** if the disc stood in the slot at least 2 cm deep: its leading edge 2 cm or more behind the console's
    front face while the disc was upright and square to the slot (within 15 degrees) and every part of it behind
    the face was inside the slot's opening, between the slot's two sides and between its top and bottom ends. A
    disc held over the console, beside the slot or lying flat (still in its case, say) is not in the slot, however
    far forward it is;
  - **0.5** if the disc came off the hub (its centre 1 cm or more from its place on the hub);
  - **0.25** if the case was opened where it lies (its cover swung upright, 90 degrees from shut, or further, while
    its tray half lay flat on the box, within 20 degrees and 1 cm);
  - **0** otherwise. Freeing the catch without swinging the cover open earns nothing.

In code: `_watch()` (the catch, the teeth, the drive and what happened during the trial), `slot_depth()` (whether
and how far the disc is in the slot, from its position, height and orientation) and `score()` in `task.py`.
The catch lets go once the cover has turned 1.5 degrees (3.7 mm at the lip) against its spring. The teeth draw in
while the button is pressed 1.5 mm or more.

## Verification

| Run | Score | Notes |
|---|---|---|
| Scripted demo, layout A | 1.0 | Catch free at step 115, case open at 360, disc off the hub at 585, 2 cm into the slot at 835, drawn in by the drive at step 875 (44 s) |
| Scripted demo, layout B | 1.0 | Same script; drawn in at step 870 |
| Control: stop with the disc partway into the slot | 0.75 | The disc stood 3.3 cm into the slot, upright, short of the 7 cm at which the drive takes it |
| Control: open the case, then lift the disc without pressing the hub | 0.25 | The case was opened; the teeth held the disc, the case came away with it and the disc never left the hub |
| Control: lift the cover's lip without holding the tray | 0.0 | The whole case tipped up 18 degrees on the fingertips, still shut; the catch never let go (run before this revision; the rule it tests is unchanged) |
| Dream-1, 2,400 steps | 0.0 | Both arms folded down near their bases and stayed there; neither touched the case, which stayed shut (run before this revision: the disc never left its shut case on the box, so the slot rule was not in play) |
| Arms still | 0.0 | The case stays shut |
| `pytest test_scoring.py` | 13 passed | Score tiers, the drop gate, the opened rule, the catch's numbers, the case's fit, the disc's geometry, the slot rule (a shut case carried over the console at any height earns no insertion credit; an upright disc over or beside the slot does not count; only an upright disc inside the opening does), and constants |

A shut case cannot be carried over this console by these arms in the simulator (the console is 30 cm tall), so that
case is covered by the offline test, which places the disc flat over the console's top with its leading edge 5 cm
beyond the front face and on the slot's line, the pose the earlier rule would have credited, and at every height
below it.

## Runtime and seeds

About 4 minutes per demo run and 11 minutes per Dream-1 run on one L40S, including about a minute of start-up.
`object_jitter=0`; the layout is fixed per lot (A or B), and there is no randomness in the task.

## Fidelity

- The disc (120 mm, 1.2 mm, 15 mm centre hole, 16 g) and the case (135 x 171 x 15 mm shut, 70 g) are rigid, with
  real dimensions. The case opens the way a real one does: the spine turns on the tray's far edge and the cover on
  the spine's top edge, a quarter turn each, so the opened cover lies flat beyond the tray. The spine's hinge has a
  slight spring toward shut (0.008 N m/rad).
- The catch is not modelled as moulded clips. It is a spring at the cover's hinge (6.5 N m/rad) that the task file
  removes for good once the cover has turned 1.5 degrees, which takes 1.2 N at the lip; the spine cannot turn while
  the catch holds. The catch does not snap shut again if the cover is put back down.
- The hub's teeth are three small blocks on radial slides, moved in by the task file while the button is pressed
  1.5 mm or more; the button is a sprung slide (600 N/m, 2.5 mm travel).
- The console's drive is not modelled mechanically: once the disc is 7 cm into the slot, upright and centred, the
  task file draws it in at 8 cm/s and holds it. The console and the storage box are fixed to the table.
- Scratches and fingerprints on the disc's data side are not judged.

## Real-world build

A standard Blu-ray-style game case (135 x 171 x 15 mm shut) with a press-to-release hub and a 120 mm game disc on the
hub, snapped shut and laid cover up on a storage box 8.5 cm tall, 28 cm long and 18 cm wide, its opening edge toward
the robot and overhanging the box by 2.75 cm. A slot-loading games console standing upright, its slot 10 cm above the
table, at the positions in `task.json`. Check the disc for scratches after each trial.

### Preparing the case

The case is the ordinary single-disc game case with a push-button hub, sold empty as a replacement case. Its hub is
used as it comes. A stock case has only a shallow finger notch at its opening edge, so the task's two openings are
cut, the same way every time, with the template `case_template.svg` (print it at 100% and check its 50 mm bar):

1. **Thumb cut-out in the tray half.** In the middle of the tray's opening edge, cut out a rectangle 44 mm across
   and 32 mm deep, through the tray's floor and its rim. It leaves the disc's edge free to be pinched there.
2. **Finger recess in the cover.** In the cover's left edge, cut out a rectangle 32 mm along the edge and 30.5 mm
   deep, starting 22 mm from the opening edge. Remove the printed sleeve's corner with it.
3. **Ledge.** Under the recess, glue a block 20 mm (along the edge) by 25 mm on the tray's floor against its left
   rim, built up from offcuts of step 1 until its top is level with the rim's top (about 6 mm). It shows through the
   recess: this is the ledge the holding hand presses on. It must stay 2 mm clear of the disc's edge.
4. **Tools and finish.** The case is polypropylene: a sharp craft knife and a steel rule, or tin snips, cut it.
   File the cut edges smooth so they cannot scratch the disc. Cuts within 1 mm of the template are close enough.

Then check each prepared case before it is used:

- **Catch.** Hold the tray half down and pull the cover's lip up at the thumb cut-out with a spring scale: it should
  let go at about 1.2 N (1.0 to 1.4 N passes). A catch that is stiffer can be eased by shaving the cover's two
  moulded clips with the knife, a little at a time. Lifting the lip of the unheld shut case must still pick the
  whole case up (it weighs 86 g with its disc): that is the reason the task needs two hands.
- **Hub.** The disc must stay on when lifted by its edge, and come off freely while the button is pressed.
- **Marks.** Tape marks on the box for the case's corners; the hub's axis at the position in `task.json`.

## Third-party assets

None: every object is built from primitive shapes in `disckit.py` and `task.py`. No brand, logo or product name is
modelled.
