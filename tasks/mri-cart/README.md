# Clear the steel and electronics off a patient cart before it goes into the MRI room

A patient cart is about to go into an MRI scanner room. On its top shelf lie six items: steel bandage scissors
tucked between two blue foam positioning pads, the steel chest piece of a stethoscope, the patient's phone, and a
plastic water bottle. The robot screens the cart: whatever can't go into the scanner room goes in the grey bin, the
rest stays on the cart. The instruction doesn't say which items those are: the robot has to know that steel and electronics stay
out, and that the foam pads and the water go in with the patient. The scissors lie 1.7 cm from each pad, so the
fingers have to go down between them.

## The real step
Before a patient goes into the scanner room (Zone IV), MR staff take everything ferromagnetic or electronic off them
and their cart: "Only use equipment and devices (e.g., fire extinguishers, oxygen tanks, physiologic monitors, and
aneurysm clips) that have been tested and approved for use in MRI environments." (The Joint Commission, Sentinel
Event Alert #38, 2008, as reproduced in [Gilk and Kanal, J Magn Reson Imaging 37:531-543,
2013](https://edge.sitecorecloud.io/americancoldf5f-acrorgf92a-productioncb02-3650/media/ACR/Files/Clinical/Radiology-Safety/Sentinel-Event-Alert-38.pdf).) Done by: MRI technologists.

What goes wrong in real life: the magnet pulls steel in as a projectile; an oxygen cylinder taken into a scanner room
killed a six-year-old patient in 2001. A phone is damaged and can be pulled too. A photo of the cart and the bin shows
what went in.

What we simplified: the cart is its top shelf only, fixed on the table; the items are rigid blocks and cylinders; no
magnetic field is simulated, so what counts is where each item ends up.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=mri_cart.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Screen this patient cart before it goes into the MRI scanner room: put every item that is not safe to take into the scanner room in the grey bin, and leave the MRI-safe items on the cart." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/mri-cart-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=mri_cart.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Screen this patient cart before it goes into the MRI scanner room: put every item that is not safe to take into the scanner room in the grey bin, and leave the MRI-safe items on the cart." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/mri-cart-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/mri-cart-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all three steel or electronic items have been in
the bin, with the MR-safe items on the cart, for 1 s; otherwise it scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| cart shelf | fixed, light grey, 24 x 36 cm inside, 1 cm floor, 2.5 cm rim | centred at (37, 0) |
| bin | fixed, grey, open, 20 x 20 cm inside, walls 6 cm tall | centred at (13, 0) |
| scissors | loose steel bandage scissors, closed, 14 x 4 x 1.2 cm, 80 g, silver | (40, 8), along x |
| pad_left, pad_right | loose blue foam positioning pads, 10 x 5 x 4 cm, 25 g, MR safe | (40, 14.2) and (40, 1.8): 1.7 cm either side of the scissors |
| phone | loose phone in its case, 14.7 x 7.2 x 1.1 cm, 190 g, black | (37, -12), along x |
| chest_piece | loose stethoscope chest piece, a steel disc 4.4 cm across, 1.3 cm thick, 100 g, silver | (30, -5) |
| bottle | loose white plastic water bottle, 6 cm across, 10 cm tall, 120 g, MR safe | (29, 13) |

Every loose object has friction 1.2 static, 1.0 dynamic. Start: both grippers open at the ready pose. One fixed
layout: `-E object_jitter=0`, so the seed doesn't change the start.

## Scoring
An item is in the bin, or on the cart, when its centre is inside that tray's walls and its lowest point within 1.5
cm of the tray's floor (items may lie on each other); still means moving under 1 cm/s and turning under 0.5 rad/s.

| Score | When, at the end |
|---|---|
| 1/3 each | the scissors, the phone or the chest piece in the bin, still |
| 1.0 | all three in the bin and every item still, held for 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0 overall | a foam pad or the water bottle not on the cart (in the bin, on the table or held): taking everything off earns nothing |

An item put anywhere but the bin earns nothing. On a real cart, a look at the shelf and into the bin checks it.

## The scripted demo
The left arm comes down over the scissors with the fingers only 54 mm apart, between the two pads (7 mm from the
scissors, about 10 mm from each pad), grips them across their width, lifts them straight up and drops them into the
bin from 3 cm above its floor. Then the right arm takes the phone across its width and the chest piece across its
diameter (the fingers closing along x, 4 mm clear of the rim and the nearer pad) and drops each into the bin, the phone
turned flat across it. The foam pads and the water bottle are never touched.

## Runtime and seeds
- The scripted demo's plan takes 36 s of simulated time (710 steps at 20 Hz), within the 2,400-step limit.
- Official runs (2026-11-08): the demo 1.0, ending on success at 35 s, all three items flat in the bin and the foam
  pads and bottle moved at most 0.3 mm; Dream-1 0.0 over the full 2,400 steps.
- The scoring and reach checks run without the simulator: `python -m pytest -q test_mri_cart.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
