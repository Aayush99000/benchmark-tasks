# Move a toaster oven's rack down one rung

A Cuisinart Chef's Convection Toaster Oven (TOB-260N1) at its true size: cavity about 374 x 330 x 190 mm, four rack positions, and, on the upper positions, a stop that holds the rack halfway out. The wire rack (368 x 305 mm, 0.35 kg assumed) starts on position 3. The robot pulls it out to the stop, lifts the front so the side tabs ride over the stop, draws it clear of the ledges, takes it down one rung, slides it in level onto position 2 and pushes it home. Nothing is scaled. The oven stands on a 12 cm stand so that the arms meet the rack at a workable height (a layout move; the real oven would sit on a counter).

**What is modelled.** The rack is a rigid frame of boxes with a 5 mm square section: two side rods, a front wire, a rear wire, four cross wires and a small tab on each side rod. There is no added grip rail. The hands pinch the first cross wire, 62 mm behind the front wire (at the front wire the rack's moment, about 0.7 N m, pitched it out of the pinch as the rear left the ledges). The stop is a 4 mm block on each upper ledge. No door, no heat. All rigid; dry steel friction.

**How it differs from a shelf-height task.** A shelf lowered onto pins is set level and has no stop and no choice of rung. Here the rack comes out only if the front is lifted past the anti-tip stop, and it must then be put on a different rung. The stop and the rung change are the task.

**Scoring.** 0.25 out past the stop (front wire at least 150 mm out of the door line); 0.5 resting on position 2 inside the oven with both hands clear; 1.0 also level within 2 degrees, front wire from 30 mm inside to 20 mm outside the door line, never dropped, held 1 s.

## Size table

| Dimension (touched by a hand or the scorer) | In the scene | Source |
|---|---|---|
| Rack, width x depth | 368 x 305 mm (14.5 x 12 in) | retailer Q&A (Links) |
| Cavity, width x depth x height | 374 x 330 x 190 mm | retailer Q&A: 355 mm usable plus grooves of about 1/4 in each side |
| Rack positions | 4 | owner's manual |
| Rung pitch | 35 mm, position 1 at 40 mm | estimate, range 25 to 45 mm |
| Stop height; tab | 4 mm; 10 x 2.8 x 5 mm | estimate, stop 3 to 8 mm |
| Wire section | 5 mm square | estimate, 3.5 to 6 mm |
| Rack mass | 0.35 kg | estimate, 0.2 to 0.35 kg |

A cut stated as a cut: only the cavity is modelled (no door, heating elements or housing), at true size; the rack is whole and at true size.

## Doing it for real

Materials: a Cuisinart TOB-260N1 (or any toaster oven whose upper positions have a stop), its rack, a digital level, a steel rule. Start with the rack on position 3. A trial is scored by the same rule as `score()`: after the move, read the rack's pitch with the level (within 2 degrees), the front wire's distance from the door line with the rule (30 mm in to 20 mm out), and which ledge the side rods rest on. The 0.25 stage is the front wire at least 150 mm out of the door line at any time, filmed from the side with the rule in view.

## Assets and licence

No third-party meshes, textures or fonts: every part is a box built in `task.py`; `mechkit.py` and `demokit.py` are this task's own helper code.

## Licence

MIT

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1. `mechkit.py` and `demokit.py` are in this folder.

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                              # the full-score reference, filmed
python demokit.py task.py OUT --no-record --mode upper,short,tilted    # lazy routes: back on the upper rung, left half out, left tilted
```

## Results of the scripted demo (GPU physics)

| Route | Score | End state |
|---|---|---|
| full, rack 0.45 kg (run twice) | 1.0 | on position 2, level, front wire 8 to 10 mm inside the door line; 689 steps (34.5 s) of 1500 |
| full, rack 0.35 kg (the modelled mass) | 1.0 | 677 steps |
| full, rack 0.6 kg | 0.25 | out past the stop; the pinch on the 5 mm cross wire lets the heavier rack slip and it is not seated and level |
| upper (set back on the original position) | 0.25 | out past the stop, not on the lower position |
| short (left 10 cm short of home) | 0.5 | on position 2, front wire 92 mm out |
| tilted (released with the front lifted) | 1.0 | on position 2 there is no stop, so the rack settles level when let go: the end state is the same as the full route |

The demo is verified at 0.35 and 0.45 kg and fails at 0.6 kg. The modelled mass is 0.35 kg, the top of the estimated 0.2 to 0.35 kg range (see Sources). The filmed run reported 0 blank frames; blackdetect on its files finds no black at the start of any camera, and the two wrist cameras go black for 1.5 to 1.9 s while the fingers close on the wire (7 to 9 s) and for 6.3 s from 23 s, when the hands hold the rack in the cavity. Dream-1: one valid run on this build (1,500 steps), score 0.


## Sources and assumed figures

One make and model throughout: Cuisinart Chef's Convection Toaster Oven TOB-260N1.

- Stop and rungs: the maker's instruction booklet (TOB-260, Williams-Sonoma PDF) says the oven has two racks and "four positions to insert the racks"; "The upper positions have a stop feature so the rack stops halfway out of the oven. Racks can be removed by lifting the front of the rack and sliding out." Which positions have the stop is not numbered; positions 3 and 4 are **assumed**. It also gives a 13-inch pizza stone and a 12-inch pizza.
- Rack 14.5 x 12 in (368 x 305 mm) and usable cavity about 14 x 13 x 7.5 in (355 x 330 x 190 mm), grooves about 1/4 in deep each side: a Best Buy customer Q&A. The cavity here is 374 mm wide at the grooves, 330 deep, 190 high.
- Rack mass 0.35 kg: **assumed**; no rack mass is published (not on Cuisinart's part page for the rack, TOB-260WR, nor retailer pages; only whole-oven weights are given). Estimate: this model has about 2.8 m of wire (2 side rods 2 x 0.305 m, front and rear 2 x 0.368 m, four cross wires 4 x 0.368 m). Round steel wire of 3.5 to 4 mm (9.6 to 12.6 mm2) at 7850 kg/m3 gives 0.21 to 0.28 kg; a real rack has a few more wires, so the honest range is **0.2 to 0.35 kg**. The model's 5 mm square wire would weigh 0.55 kg at steel density, so its mass is set to the top of the real range instead. The demo is verified at 0.35 and 0.45 kg and fails at 0.6 kg; it is claimed only for 0.2 to 0.35 kg (0.35 tested; lighter racks not run).
- Rung pitch 35 mm and position 1 at 40 mm: **assumed** (four positions fill a 190 mm cavity); range 25 to 45 mm.
- Stop 4 mm high, tab 10 x 2.8 x 5 mm, stop 162 mm behind the start at the tab (the rack stops about 145 mm out, roughly half its depth): **assumed**; stop 3 to 8 mm. The maker gives no stop geometry.
- Wire 5 mm: **assumed**, typical rack wire 3.5 to 6 mm, modelled square. A pinch on wire this thin holds only 2.5 N per finger per mm, so the hands hold the rack by the first cross wire, close to its centre of mass.

## Links

- Cuisinart TOB-260N1 owner's manual (four positions, "upper positions have a stop feature so the rack stops halfway out", removed by lifting the front): https://content.abt.com/documents/166473/cuisinart-toaster-oven-TOB260N1NAS-owners-manual.pdf (page 4 of the PDF)
- Product page (retailer; Cuisinart's own page for this model was not found): https://www.bestbuy.com/product/cuisinart-chefs-convection-toaster-pizza-oven-stainless-steel/J7R2YV4T2L
- Cavity and rack size (rack 14.5 x 12 in, grooves 1/4 in, 4 positions, interior about 7.5 in high): https://www.bestbuy.com/site/questions/cuisinart-chefs-convection-toaster-pizza-oven-stainless-steel/5687001
- Parts listing, replacement rack TOB-260WR (fits TOB-260 and TOB-260N1; lists no mass): https://www.cuisinart.com/wire-rack-for-chef%E2%80%99s-convection-toaster-oven/TOB-260WR.html
