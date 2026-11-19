# Hand a drink out a self-closing drive-thru window: hold the slider, set the cup on the outside shelf, let it shut clear

A painted plywood wall stands across the counter with a self-closing drive-thru slider in it (Quikserv SC-4030). A full, lidded 16 oz cold cup stands on the counter inside. Outside, under the window, is a stainless hand-off shelf. The sliding panel pulls itself shut, so one hand must hold it open by its pull bar while the other carries the cup out through the opening and sets it on the shelf. The panel's closing path runs through the serving arm: the window may be let go only once that arm is back inside, or the panel strikes the arm or the cup.

In the demo the left hand takes the pull bar, its fingers pointing 45 degrees to the right, and slides the panel 30 cm open. The right hand takes the cup by its upper body, lifts it, carries it out through the opening and lowers it onto the shelf, lets go and comes back inside. Then the left hand opens its fingers and draws back; the bar slides out of them and the panel shuts.

**Why it matters.** Fast food and counter workers (O*NET 35-3023.00) have the Core task "Serve customers in eating places that specialize in fast service and inexpensive carry-out food." At the drive-thru the window shuts itself: Quikserv's spec sheet for its self-closing sliders says "By simply releasing the window after serving the customer, the door panel closes automatically." The shelf stands in for the driver's hand, so no person is within the robot's reach in a real trial.

**Ability tested:** holding a self-closing panel open against its pull with one hand while the other reaches through a 10 cm wall with a full cup, and timing the release to the serving arm's return.

**Nearest tasks.** Restocking a spring-pusher shelf lane while holding the pusher back also holds a sprung part open while the other hand works. Here the work is a reach through a wall to an outside shelf with a full cup, and success needs the closing panel to find the arm gone. A lid held open while an item is taken out has no closing path through the arm.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Serve the drink through the drive-thru window. With your left hand, slide the window open by its pull bar and hold it. With your right hand, carry the cup out through the opening and set it down upright on the shelf outside, then let go. Bring your right arm back inside, then let go of the window so it closes by itself without its edge touching either arm or the cup, and draw your left hand back."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                                                 # full score, with a video
python demokit.py task.py OUT --no-record --no-cameras --mode one_arm,early_release,no_close   # the check runs below
```

The demo reads the poses of the cup and the panel to plan its grasps and to bring the cup over the shelf (the arm sags under the cup's weight). It reads no contact force and never moves an object except through the grippers.

## The scene and what is simplified

- **Wall:** painted plywood on studs, 10 cm thick (inner face at x 36 cm, outer face at x 46 cm from the midpoint between the arm bases), 180 cm long and 130 cm high, so there is no way round it.
- **Window:** a Quikserv SC-4030 self-closing horizontal slider at its catalogue size: a 48 x 36 in aluminium frame, a 20-1/4 x 29 in service opening, a fixed pane of 1/4 in tempered glass, and a sliding panel of 1/4 in tempered glass in an anodized aluminium frame (about 55 x 77 cm and 8 kg) hanging from its top track. It starts shut and unlocked; the lock is not modelled. Its pull bar is 25 mm round aluminium, 30 cm long, standing 3 cm off the glass, 3 cm from the leading stile.
- **Closer:** the panel is pulled shut by a constant 10 N with a 22 N s/m damper, so it shuts from 30 cm open in about 1 s. Both are simulation settings standing in for the slider's closer. The sheet gives no figure; a real unit's pull is read with a spring scale hooked on the bar.
- **Shelf:** a Ready Access 65198701 stainless hand-off shelf (13-gauge, 53-1/2 x 10 in), mounted on the outer wall under the window, its top 12.5 cm above the counter.
- **Cup:** a 16 oz paper cold cup with a flat polystyrene lid, 9 cm across the rim, 6 cm across the base and 14 cm tall, about 20 g empty and holding 0.47 kg of water. The cup and its water move as one rigid body, so a spill is read as a tip past 30 degrees, and crushing as a grip squeeze over 20 N (a setting). In a real trial both are judged on the video: water past the lid, or a dent in the cup wall deeper than 5 mm, read with a ruler laid across it.
- **Touch by the stile:** the stile's closing face is checked against the cup's surface, all 24 fingertip points and capsules round each arm's links from the elbow out, and a panel that stops short of shut with no gripper's fingers on it counts as having closed on something.
- Physics: rigid bodies at 240 Hz on the GPU.

## Building it for a real trial

Mount a Quikserv SC-4030 (or any manual self-closing slider of that size) in a 10 cm stud wall built across the robot's table, with the opening's sill 12.9 cm above the table, and screw a Ready Access 65198701 shelf to the outside face under it, its top 12.5 cm above the table. Read the panel's closing pull with a spring scale on the bar, and record it. Use a real 16 oz paper cold cup with a flat lid, filled with 0.47 kg of water. Film with two cameras: one outside at the shelf's right end, 40 cm up, looking along the wall, and one inside above the right jamb, looking down on the stile's face. The window's own closer moves only a light panel, and nobody stands outside within reach of the robot.

## Scoring

0, latched, if the face of the panel's leading stile ever touches the cup or any part of either arm, or the closing panel stops short of shut with no gripper's fingers on it (it rests on an arm or the cup), or the cup tips more than 30 degrees or falls (drops at 1 m/s or more, or ends off the shelf more than 3 cm below its top), or it is crushed. A gripper holding the pull bar is not a touch. Otherwise, at the end of the trial:

| Score | Condition |
|---|---|
| 0.25 | the panel was opened at least 15 cm at any time, and the cup was lifted at least 2 cm off the counter at any time |
| 0.5 | and the cup's centre is past the wall's outer face (x 46 cm), upright within 15 degrees |
| 0.75 | and the cup stands on the shelf (its base within 1 cm of the shelf top and wholly over it, upright within 15 degrees), with both grippers at least 8 mm off it |
| 1.0 | and both arms are back inside the wall's inner face (x 36 cm), the panel is shut (gap under 5 mm) with neither hand on it, and the panel and cup are still (no visible movement over 1 s on the video, held 1 s) |

Real trial: the two videos for any touch by the stile's face, the cup's tilt and anything caught in the closing window; a photo of the cup on the shelf; a tape at the closed panel's edge.

## Results

| Run | What it does | Score |
|---|---|---|
| Dream-1 | Run at 320 x 176, 2,400 steps | 0.0: the model moved its right hand to the cup and pushed it about the counter, never took the pull bar or opened the window, and ran to the step limit |
| `full` | The whole task (the film) | 1.0, at step 504 (25.2 s) |
| `one_arm` | The right hand opens the panel by its bar, lets go to fetch the cup, and the panel shuts | 0.0 |
| `early_release` | The cup set on the shelf, then the window let go while the right arm is still out: the panel closes on the arm | 0.0 |
| `no_close` | The cup served and both arms in, but the left hand still holds the window open | 0.75 |

## Low-cost real-trial window

Low-cost real-trial stand-in for the commercial window (about US$300 to 400 in total, an estimate and not a quote; the Quikserv SC-4030 is a sim-only reference). It keeps every value the simulation uses: a 55 x 77 cm sash of about 8 kg, 48 cm of travel, a constant closing pull of 10 N, a damper of about 22 N s/m, and the 25 mm x 30 cm pull bar 3 cm off the glass. Build: (1) Wall: a frame of 38 x 63 mm timber studs with 18 mm plywood on both faces (63 + 2 x 18 = 99 mm, the simulated 10 cm), 130 cm high, with a 51.4 x 73.7 cm opening whose sill is 12.9 cm above the counter; the opening is lined with 20 x 20 x 2 mm aluminium angle. (2) Fixed pane: 6 mm clear polycarbonate beside the opening, screwed to the plywood in the same plane as in the simulation (x 36 to 39 cm). (3) Sash: 6 mm clear polycarbonate cut 55 x 77 cm (3.05 kg), edged with 20 x 20 x 2 mm aluminium angle (0.54 kg), and 4.4 kg of steel flat bar 50 x 12 mm, two 465 mm lengths bolted along the bottom rail, so it weighs 8.0 kg (weigh it; trim to 8.0 +/- 0.5 kg). (4) Slider: a top-hung sliding-door track and carriages, Hawa Junior 80/B (rated 80 kg; hawa.com) cut to 1.2 m, mounted on a 40 x 40 mm timber header above the opening, with a 6 mm floor guide at the bottom; stops on the track limit the travel to 48 cm, which leaves the pull bar inside the opening. The sash's closed edge is 1 mm off the right jamb, which has a 5 mm foam stop. A cheaper alternative for the slider is two Accuride 3832-E20 ball-bearing slides (20 in, 100 lb, full extension, 508 mm of extension) fitted to the sash's top and bottom edges. (5) Closer: a constant 10 N pull toward the jamb, from a 1.02 kg weight (a steel pipe cap filled with lead shot) on a cord over a ball-bearing pulley at the far end of the header, which gives a constant 10 N at every opening; or a constant-force spring of 10 N from the Lesjofors constant force spring range (strip drum fixed at the far end, its end hooked to the sash), https://shop.lesjofors.com/global/en-US/all-products/flat-springs/constant-force-springs-p-1035/. A gas spring is not used, since its force changes with the stroke. (6) Damper: a small adjustable oil damper (a linear hydraulic damper with its needle valve set) between the sash and the header, tuned so that the sash released at 30 cm open shuts in about 1 s without a bang, which is about 22 N s/m. (7) Pull bar: 25 mm round aluminium tube, 30 cm long, from 20 to 50 cm above the counter, on two stand-offs 3 cm off the glass, 3 cm from the stile. Check before the trial: a spring scale on the bar reads 10 +/- 1 N at 0, 15, 30 and 45 cm of opening, and the stopwatch time from 30 cm to shut is 1.0 +/- 0.3 s. The shelf can be any 53 x 10 in stainless or plywood shelf at the same height. Protect the grippers and the rig from water: the real cup holds 0.47 kg of water, so work over a rimmed tray (25 mm lip, 60 x 40 cm) on the counter and a second under the shelf, with a towel or absorbent mat in each; keep the wrist cable exits and the controller under plastic sleeving or a drip cover; keep the power strip and controller off the floor and out from under the shelf; use a lid with a snap-fit rim; and for the first trials fill the cup with 0.2 kg of water, then the full 0.47 kg once the carry is clean.

## Runtime and seeds

The demo takes 25.2 s of simulated time (504 steps). There is no randomness: `object_jitter=0`, so no seed is needed.

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets.

- Window: Quikserv single-slider spec sheet rev 1.2, SC-4030 (https://www.webstaurantstore.com/documents/specsheets/qs-1sheet-scsinglesliders-rev12.pdf).
- Shelf: Ready Access 65198701, 53-1/2 x 10 x 8 in stainless steel shelf, for interior or exterior mounting (https://www.webstaurantstore.com/ready-access-65198701-53-1-2-x-10-x-8-stainless-steel-shelf-for-131-and-606-series/71A65198701.html).
- Occupation: O*NET 35-3023.00, Fast Food and Counter Workers (https://www.onetonline.org/link/details/35-3023.00). This includes information from O*NET OnLine by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the CC BY 4.0 licence; O*NET is a trademark of USDOL/ETA. BLS Occupational Outlook Handbook, Food and Beverage Serving and Related Workers (https://www.bls.gov/ooh/food-preparation-and-serving/food-and-beverage-serving-and-related-workers.htm).
- Stand-in parts: Hawa Junior 80/B top-hung sliding-door track (hawa.com); Accuride 3832-E20 ball-bearing slide (accuride.com); Lesjofors constant force springs (https://shop.lesjofors.com/global/en-US/all-products/flat-springs/constant-force-springs-p-1035/).
- Quikserv and Ready Access are named here for identification only; the names are their owners' trademarks.

## Licence

MIT
