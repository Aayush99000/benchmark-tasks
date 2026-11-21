# Raise an office chair's seat to 53 cm: hold its gas-lift lever up as the seat rises, and let go on the height

An IKEA MARKUS office chair stands on the bench with its seat at its lowest (47 cm). With no one sitting on it, lifting the lever under the seat's right edge lets the gas lift raise the seat by itself, about 8 cm a second here, and letting the lever drop locks it where it is. The seat cannot be pushed back down by hand, so one hand lifts the paddle from beneath, follows it up as the seat rises, and lets go as the seat's front edge reaches 53 cm on a ruler beside it. One hand does it; the other is not needed.

## Run

From this folder, on a GPU machine with the harness's Python:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,overshoot,hand,early,undo
python check_film.py OUT/full                      # black-frame count per camera and faststart
```

The harness's instruction (Dream-1 run): `--instruction "Raise the chair's seat so its front edge is 53 cm above the bench top, by the ruler beside it."`

## Scoring, and the routes that score less

Height is the seat's front edge top above the bench: 47 cm plus the column joint.

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | the right hand slides under the paddle, lifts it to 8 mm, follows the seat up, and lets go 8 mm below the height | 1.0 (seat 52.8 cm), read at the end of the run |
| none | nothing | 0 |
| overshoot | the lever held until the stroke ends | 0.25 (seat 58 cm) |
| early | lever let go at about 49 cm | 0.25 |
| undo | set at 53 as in full, then the lever lifted again until the seat is out of the band (56.5 cm) | 0.25 (the end state counts, not the first stop) |
| hand | the right hand pushes up under the seat's pan, about 9 N reached, lever untouched | 0 (the column stays locked) |

The demo closes the loop on the seat's height and the lever's lift, both read from the chair's joints: it finds the paddle by its first 1.5 mm of lift, holds the lever at 8 mm (valve open past 6 mm, stop at 10 mm) while the seat rises, and lets go when the seat is within 8 mm of 53 cm (the hand's lag at 8 cm/s).

## Runs

The score is read at the end of the run, so a seat set and then raised out of the band scores its end tier (undo mode, 0.25). Demo modes full, none, overshoot, hand, early, undo: 1.0, 0, 0.25, 0, 0.25, 0.25. README command: 1.0 on a lab PC (0 black frames, demo.mp4 faststart) and 1.0 on an RTX 5090. Dream-1 on the RTX 5090, 24 rows per request (slowest 30 steps 5.03 s): 0 in its 280 steps, one session throughout. Four jittered runs (chair and ruler shifted 10 mm in x and y with OC_DX and OC_DY, the demo aiming at the nominal place): 1.0 in all four.

## Real trial

An IKEA MARKUS (or any gas-lift office chair of the same seat height range), its casters in rubber caster cups on the bench, a 60 cm steel ruler in a weighted block 3 cm in front of the seat's front edge, and a side camera. Measure the real lever's travel, force and valve point and the unloaded rise speed: the figures here are estimates. Reset by sitting on the seat with the lever up until it bottoms at 47 cm.

## Size table

`python chairmesh.py` prints the model's sizes (the size check); its output on the final code is below. IKEA's listing: https://www.ikea.com/us/en/p/markus-office-chair-vissle-dark-gray-50588804

```
seat width  57.0 cm   (listed 57)
seat depth  49.0 cm   (listed 49)
seat front edge top at its lowest 47.0 cm   (listed 47 to 58 cm)
back top, seat lowest 129.0 cm (headrest 128.6)   (listed min. height 129)
base width x 69.9, y 66.5 cm   (listed 70 x 70)
armrest outer width 67.8 cm; armrest top 68.8 cm = 21.8 cm above the seat
caster wheel diameter 5.0 cm
```

| Dimension | In the scene | Source |
|---|---|---|
| Base width and depth | 69.9 x 66.5 cm (casters' outer edges, the five-star's own extents; the colliders' circle is 67 cm across) | IKEA listing, 70 x 70 cm (27 1/2 in) |
| Seat width x depth | 57.0 x 49.0 cm | IKEA listing, 22 1/2 x 19 1/4 in |
| Seat height, front edge top | 47 cm at the start, joint 0 to 11 cm | IKEA listing, 18 1/2 to 22 7/8 in (47 to 58 cm) |
| Back top, seat lowest | 129.0 cm (headrest top 128.6) | IKEA listing, min. height 50 3/4 in (129 cm) |
| Back width | 50 cm | estimate |
| Armrest top above the seat | 21.8 cm; pad 6.8 x 26 cm; outer width 67.8 cm | estimate (IKEA lists no arm size) |
| Caster wheel diameter | 5.0 cm (twin wheels) | estimate |
| Gas-lift outer tube, inner tube | 7.6 cm, 5.4 cm diameter | estimate |
| Lever paddle | 12 x 3 x 0.8 cm, tip 3 cm beyond the seat's side, 10 mm travel, 12 N, valve past 6 mm | estimate (IKEA publishes none) |
| Rise speed, spring push | 8 cm/s, 200 N | estimate |
| Ruler | 60 cm, mark at 53 cm | a steel ruler; our choice |

## What is modelled, and from what

IKEA publishes the sizes above and photographs the chair, but no drawing or CAD, and no open-licensed (CC0 or CC-BY) MARKUS model exists: the ones found (CGTrader, RenderHub, TurboSquid, SketchUp 3D Warehouse) are commercial or carry no open licence. So every mesh is built by us in `chairmesh.py` from the listed dimensions and the product photos, and none is a third-party mesh. They are the five-star base (tapered legs, hub, cone to the column) with five twin-wheel casters; the gas-lift's outer tube, inner tube and telescoping cover; the seat's tilt mechanism, shell and a 57 x 49 cm cushion with a shallow bowl and a rolled edge; the curved mesh back with its frame, lumbar bow, headrest and spine; two armrests; and the lever's rounded paddle. Shapes between the listed sizes (curves, tapers, the lumbar bow) are our reading of the photos.

Colliders are unchanged simple boxes and cylinders (pan, housing, back, hub, outer tube, legs, casters, paddle). Armrests have none: the arms never reach them (the hand works at the lever, 28 cm below the armrest tops). The base meshes reach 70 cm where the caster colliders sit on a 67 cm circle; nothing touches them. The seat, housing, paddle and back follow their colliders (the back's lean and lumbar bow move it within 2 cm of its collider's plane); the hub, column cover, base meshes and armrests are larger or outside theirs.

## Sources

- IKEA MARKUS office chair, US listing (https://www.ikea.com/us/en/p/markus-office-chair-vissle-dark-gray-50588804): width 27 1/2 in, depth 27 1/2 in, seat width 22 1/2 in, seat depth 19 1/4 in, seat height 18 1/2 to 22 7/8 in (47 to 58 cm), min. height 50 3/4 in, max. height 55 1/8 in, tested for 276 lb. IKEA publishes no lever, gas-lift force or rise speed.
- Estimates, not published: the lever paddle (12 x 3 x 0.8 cm, 10 mm tip travel, valve open past 6 mm, 12 N at the tip), the 8 cm/s rise, the 200 N spring push, masses and friction.
- All meshes are built in `chairmesh.py` from IKEA's published sizes and photos; no third-party meshes are used (no CC0 or CC-BY MARKUS model was found).

## Licence

MIT
