# Change the toner cartridge in a desktop laser printer and close its front door

A desktop mono laser printer stands on the table with its front door fallen open. The used toner cartridge is in
the bay behind the door; a new one stands on the table to the right. The robot has to take the used cartridge out and
leave it on the table, put the new one in all the way to the back stop, and shut the door until it catches. The
instruction gives only the goal.

**Credit.** This task builds on the idea of PR 1809, "Add swap-printer-toner task" (TK-M8Z76X, revised as TK-K9NTED
and TK-XN4PP6), whose 48-hour revision hold lapsed. Its three reviews raised these points, each addressed here:

| Review point | What this version does |
|---|---|
| TK-M8Z76X: objects should be more representative of a realistic printer and toner cartridge, especially the handles. | The printer has the published outside size of an HP LaserJet Pro M404n (381 x 357 x 216 mm), a front-loading toner bay and an L-shaped front door (front panel plus the front of the top), as on front-loading desktop laser printers. The cartridge is a 33 cm moulded body with end caps, label and shutter strip, and a carry handle along the top of its front: a 16 cm bar on two posts with a 15 mm finger gap under it. The hands grip that bar, as a person does. |
| TK-K9NTED: the setup screenshot and full-score demo showed earlier geometry. | The screenshot, demo video and Dream-1 video all come from runs of the `task.py` in this folder, with nothing else in it. |
| TK-XN4PP6: name an obtainable printer and compatible dummy cartridges with the guided seating and door, or give fabrication files. | The real trial uses an HP LaserJet Pro M404n, unplugged, and two spent HP 58A (CF258A) cartridges, the printer's own cartridge, as the dummies (spent cartridges are routinely collected for recycling). The printer's own bay guides, door and catch are the mechanism, so nothing has to be made. The new one is marked with orange tape on its handle. |

All of the code here is new. Nothing from PR 1809's files was used.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. Other files:
- `parts.py` builds the printer as one fixed articulation (body plus a hinged door) and the two cartridges as loose
  multi-shape rigid bodies, and runs the per-step callbacks.
- `tilt.py` lets the scripted demo lean the gripper (used only for the door; see below). It affects only the
  scripted demo, never Dream-1 or scoring.
- `near_misses.py` scores near-successes and wrong outcomes without a simulator (24 cases).

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the new toner cartridge in the printer in place of the used one, leave the used one on the table, and close the printer."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 2 --store-frames --log-dir ~/yam-eval/toner-dream

# Scripted demo (reaches 1.0 at about 35 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 2 --store-frames --log-dir ~/yam-eval/toner-demo
```

- **Runtime per trial:** at most 45 s of simulated time (900 steps at 20 Hz), about a minute of wall time on an L40S
  once Isaac Sim has started.
- **Seeds:** one fixed layout. Run with `-E object_jitter=0`. The two epochs are two trials of the same layout; the
  score is their mean.

## Scoring in the simulator

`score()` and `success()` read a record that an `on_step` callback updates every control step. Exact definitions:

- **Hand points:** every vertex of the collision meshes of both grippers and all four fingertips, plus each triangle
  face sampled about every 3 mm. "No hand within 5 mm" means every such point is more than 5 mm from every box of the
  part (signed box distance).
- **Fails the whole trial:** either cartridge's vertical velocity below -0.5 m/s at any step; either cartridge's up
  axis more than 30 degrees from vertical at any step; any corner of a cartridge's boxes more than 2 cm below the table
  top; or the hand or object state unavailable at any step (fail closed).
- **Used cartridge out (0.25):** no corner of its boxes overlaps, in plan view, the printer's footprint (x 0.24 to
  0.597 m, y within 0.1905 m) or the open door's (x 0.047 to 0.24 m, y within 0.174 m); its lowest corner at most
  5 mm above the table; tilt at most 5 degrees; no hand within 5 mm.
- **New cartridge seated (0.5):** body origin (bottom centre) within 4 mm of (0.296, 0) in x and in y, within 3 mm of
  the bay floor height, yaw and tilt each at most 3 degrees; no hand within 5 mm.
- **Door shut and caught (0.75):** the door reached within 1.5 degrees of shut at some step (the catch engaged), and
  is now within 2 degrees of shut.
- **Success (1.0):** all of the above, no hand within 5 mm of the door either, and over the last 1 s no corner of
  either cartridge moved more than 2 mm and the door turned less than 0.01 rad.
- Limits are compared with a 1e-6 margin, so a value exactly at a stated limit passes. The record resets on scene
  reset (`on_reset`), never during a trial.

The scoring text in `task.json` is the same rule for a person with a tape measure, the video and a final photo.

## The printer and its door

- The bay is 34 cm wide between its side walls, 5 mm wider than the cartridge at each end, and 10.8 cm deep to its
  back stop. Its floor is 3.5 cm above the table, over the paper cassette.
- The door is hinged along the bay floor's front edge. Open, it lies flat with its inner face level with the bay floor,
  so a cartridge slides out of the bay onto it, and its top lip stands up 10.8 cm at its end. It turns 90 degrees to
  shut. With the lip, its centre of mass is behind the hinge when shut: from the geometry it falls shut within about
  7 degrees of shut and falls open beyond that. Within 1.5 degrees of shut a catch engages (in the simulator, a
  200 N m/rad hold to shut switched on in `on_step`); nothing in the task releases it.
- Shut, the door's inside is 6 mm in front of a seated cartridge's front, so it cannot shut over a cartridge standing
  out further than that.
- The walls are too tall to lift a cartridge out of the bay straight up within the arms' reach, so a cartridge comes
  out and goes in along the bay floor, over the open door, as in the real printer.

## Real-world trial

Printer: HP LaserJet Pro M404n, unplugged, on its taped outline (8.2 kg; the arms do not move it). Dummy cartridges:
two spent HP 58A (CF258A) cartridges, one with orange tape round its handle. Use the printer's own door release and
let its door fall open. Score from the video and a final photo with a tape measure; "seated" can be checked by the
cartridge sitting fully home on its guides. The printer stays unplugged, and spent cartridges hold only residual
toner, so tape over each one's drum opening to keep it inside; a drop is a scored failure.

The simulated sizes of the bay, door and cartridge are a model of a front-loading printer of this size, not a
measured copy of the M404n's parts. On the real printer its own guides, door travel and catch decide what "seated"
and "shut" mean; reviewers should expect the stated millimetre tolerances to apply to the simulator.

## The scripted demo

`demo()` is fixed waypoints from the published layout; it reads no simulator state. The left hand turns its wrist
round (yaw -90 degrees, fingers still closing across the bar), takes the used cartridge's handle, slides it out onto
the open door, lifts it 3.5 cm, carries it to the left of the printer and sets it down. The right hand picks the new
cartridge up by its handle, waits in front of the printer's right side until the left side is clear, sets it on the
open door, slides it in 8 mm past the seated position (the arm gives way; the back stop holds it), lets go and backs
out. Then, fingers shut and leaning 45 degrees forward (`tilt.py`), it presses on the door's lip 6 cm from the panel's
end and follows the lip's arc, computed from the published door geometry, from open to shut, aimed 25 mm into the lip
so the yielding arm keeps pressing. It lifts off and parks. A straight-down hand could not do this: the lip rises
above the straight-down reach (about 0.20 m) before the door is near shut, and a horizontal push at a fixed height
loses the lip after about 26 to 29 degrees (two traces).

## Traces (L40S, Isaac Lab-Arena, `-E object_jitter=0`)

1. Straight-down push at 13 cm: swap done (0.5), the door rose to 26 degrees and fell back.
2. Straight-down push at 9.5 cm: 0.5, the door rose to 29 degrees and fell back.
3. Leaning arc push: door caught at 33.0 s, 1.0 from 35.0 s.

Final runs (2 trials each): demo 1.0 and 1.0; Dream-1 0 and 0 (both hit the step limit).

## What is simplified

- The printer is fixed; the real one weighs 8.2 kg on rubber feet.
- Cartridge body, end caps, handle, bay and door are boxes; the cartridge's rounded top is visual only.
- The catch is a switched stiff hold, not a modelled latch. The door release button is not used: the door starts open.
- No chip contacts, drum, toner or electronics. One fixed layout, so a replay of this layout cannot be ruled out;
  there is no choice in the task that a fixed habit could win.
