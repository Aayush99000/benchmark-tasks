# Remove the tree supports from a 3D-printed robot part, holding it over a scrap bin

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena),
Isaac Lab-Arena 0.3.1) on the station scene: [twist_supports_off_printed_crank.py](twist_supports_off_printed_crank.py).
It needs [labkit.py](labkit.py), next to it, for object kinds yam_arena's task files don't have (a body made of
primitive colliders drawn with a mesh, a breakable joint between two bodies, a fixed model with box colliders), and the
meshes in [assets/](assets).

## What the robot does

A finished print stands on a 3D printer's bed: an orange crank link (two 28 mm bosses joined by a bar, one boss on the
bed and the other on top of the bar's far end) with four tree supports printed under its bar and under its top boss's
rim. The robot takes the print off the printer, removes every support into the white bin, and puts the part in the
blue tray. Each support is held to the part by its tips. In the simulation a support holds a straight pull of up to
15 N (60 times the part's weight) but breaks off when bent with 0.10 N m (estimated values: see the calibration below).
So one hand has to hold the part while the other takes each support and bends it off, reaching each from a different
side; the last one sits behind the raised boss, out of reach until the part is turned.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/twist-supports-off-printed-crank/twist_supports_off_printed_crank.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the orange print off the printer, remove all of its supports into the white bin, and put the part in the blue tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/twist-supports-off-printed-crank-dream
```

Scripted demo: the same command with `--policy yam_scripted`. It is open loop, so `-E object_jitter=0` is needed.

The run prints `task_score` at the end. Scoring: `score()` in the task file and `scoring` in task.json.

## Scoring, and why

- 0.10 for each support broken off the part, +0.05 more if it is in the bin (0.60 for all four in the bin); +0.40 for
  the part on the tray's floor with all four off. Breaking a support off is the hard skill, so it earns most of the
  credit even if the support misses the bin. A part trayed with a support still on gets no tray share: that is the
  habit move (pick it up, put it in the tray), and it scores 0.
- A support is off once its centre has moved over 5 mm from its place in the part's frame, so carrying the print
  around with its supports on never counts, and a few millimetres of bending don't either.
- Gates for the rest of the trial: the part falls off the table, falls freely (faster than 1.4 m/s down, a 10 cm
  drop), or ends in the bin.
- Success also needs everything still for 1 s and both grippers more than 2 cm from the part (labkit reads both TCPs
  every step), so a part still held in the tray doesn't count.

## The demo

Right arm: pinches the flat bar from above (flat faces, so the part can't spin in the grip), lifts the print off
the bed (its supports come with it) and holds it 15 cm over the scrap bin beside the bed. Left arm: for each support,
a forward grip (fingers pointing forward, closing sideways) on its trunk 4 mm above its foot, the fingers opened just
enough to clear the support's own branches and not its neighbours or the down boss; bend it about its top 12 degrees
each way, twice, until it snaps, pull it down clear of the part and open: it drops into the bin. Before the last
support, which sits behind the raised boss, the right wrist turns the
part 60 degrees and slides it 4.5 cm toward the middle, bringing that support to the front and the arms apart. Then
the right arm sets the part in the tray and lets go. The plan is 51.5 s; success holds from about 52 s (1,034
steps).

## Real-world setup and calibration

- Printer: Creality Ender-3 with its textured build sheet. Bed slid fully forward (to the front of its Y travel), Z
  raised 10 cm after the print, as the printer leaves it at the end of a job.
- [assets/make_print.py](assets/make_print.py) rebuilds the STLs and the simulation's meshes.
- Print: [assets/crank_with_supports.stl](assets/crank_with_supports.stl), the part with exactly the simulation's four
  tree supports modelled in (tips 0.2 mm under it), PLA, 0.2 mm layers, 20% infill, the slicer's own supports off.
- Break thresholds: the simulation uses 0.10 N m to bend a support off and 15 N to pull it off. These are estimates,
  not yet measured. To measure them on three prints (and set `BREAK_TORQUE` / `BREAK_FORCE` in the task file to the
  medians, then re-run the demo and Dream-1): hold the part in a vise by its top boss. (1) Pull: loop a string round a
  support's trunk 4 mm above its foot and hang weights in 1 N steps straight down from it; record the load that frees
  it. (2) Bend: clamp a 10 cm lever to the trunk at the same height and pull its end sideways with a spring scale;
  torque = reading x 0.10 m. If a straight pull frees a real support at a low load, the task still holds: the score
  counts supports broken off by any means.
- Scrap bin 9 x 9 x 5.5 cm (white) beside the bed, tray 10 x 14 x 2.5 cm (blue), positions as in task.json's setup.
- Scoring a real trial: look (is each support off, and where is it?) and a ruler (the part on the tray floor, inside
  the walls). The gates need a video: did the part fall or drop more than 10 cm?

## Results (Isaac Sim, L40S)

- Scripted demo: 1.0 in two runs out of two with this code (all four supports off and in the bin, the part on the
  tray floor; success after about 52 s, 1,034 steps). PhysX here runs non-deterministically, so runs vary slightly:
  the second bend of each support is there for a grip that doesn't snap it the first time.
- Dream-1: 0.15 over 2,400 steps (one support broken off into the bin).
- These runs used this exact code, labkit and meshes; two code comments were corrected afterwards (the thresholds
  are estimates; runs vary) and the description of the table corrected (where the bin stands), with no change to the
  code itself.

## Runtime and seeds

- Demo: about 52 s simulated (1,034 steps to success); a run takes about 3 minutes on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated); about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed.

## Checks without the simulator

```bash
python -m pytest -q tasks/twist-supports-off-printed-crank/test_twist_supports_off_printed_crank.py
```

They score made-up trials for each outcome (each partial credit, each gate, the habit move, the straight pull, a
part still held) and play the demo through the arm's IK to check every pose is reachable within the joint limits. They
need yam_arena but not Isaac Sim, and run without pytest too.

## Assets and licences

- `assets/ender3.usda`: the printer, from the manufacturer's open CAD of the Creality Ender-3
  (https://github.com/Creality3DPrinting/Ender-3, `Ender-3 Mechanical/3DXML/Ender3.3DXML`), licensed GPL-3.0 (full
  text in [assets/LICENSE-ender3.txt](assets/LICENSE-ender3.txt)). Changes: converted from 3DXML to a mesh, parts
  coloured as the stock printer, the bed carriage moved to the front of its travel and the X gantry raised 10 cm,
  pieces under 8 mm (screws, nuts) dropped, and the mesh simplified to about 54,000 triangles. The modified file is
  distributed under the same licence.
- `assets/crank.stl`, `assets/crank_with_supports.stl`, `assets/crank.usda`, `assets/support_*.usda`,
  `assets/make_print.py`: made for this task.
