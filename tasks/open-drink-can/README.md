# Open a drink can by its stay-on tab: lever its thin tail up with a fingertip's edge until the seal breaks

A closed drink can stands on the table, held upright. The instruction is only "Open the can." Opening one is
everyday, but physically fiddly for a robot: the tab's tail is a 1.5 mm curled edge lying 1.3 mm above a lid that is
4.6 mm below the rim. It is too thin and too close to the lid to pinch. A fingertip's edge has to get under it inside
the rim, the tail has to be levered up on its arc about the rivet, and it takes force to break the seal before the panel
folds into the can. There is no rule to know, only the manipulation.

## The real step
The stay-on tab is a lever riveted to the lid: lifting its tail presses its nose down on the scored panel until the
seal breaks and the panel folds in. Wikipedia ([Beverage can](https://en.wikipedia.org/wiki/Beverage_can)): the tab
"uses a separate tab attached to the upper surface as a lever to depress a scored part of the lid". Done by: nearly
everyone, every day. A person does not pinch the tail: they slide a fingertip or nail under it and lever it up.

What goes wrong in real life: the fingertip slips off the tail before the seal goes, or the tab is pushed or pulled the
wrong way and bends with the can still shut. A look at the lid (panel in, tab up) shows the result.

## The can and its tab: sizes and sources
The simulated can is an ordinary 12 fl oz (355 ml) can with a 202 stay-on-tab end. For real trials use an unmodified
Coca-Cola Classic 12 fl oz can (the standard 202/211 x 413 can), chilled and not shaken, in a holder with a drip tray.

| Size | Value used | Source (pages opened 2026-11-18) |
|---|---|---|
| can body, height | 66.0 mm across, 122.7 mm tall | Ball, "202/211 x 413 12 oz industry standard" drawing: body 2.600 in, flanged height 4.812 in. [PDF](https://www.cansource.com/assets/Uploads/Products/Documents/Ball-12oz-std-can-specs.pdf) |
| lid panel below the rim's top | 4.6 mm | Can Manufacturers Institute, 202 end: countersink depth 0.270 in, panel height 0.090 in, so the panel is 0.180 in down. [cancentral.com/standards](https://www.cancentral.com/standards/) |
| end across, seamed | 54 mm | "a container necked to a 202 (2.125 inches)". [US 7,909,196 B2](https://patents.google.com/patent/US7909196B2/en) |
| tab length, width | 25.0 x 15.7 mm | "a conventional tab ... has a length ... of about 0.985 inch", "a width ... of about 0.620 inch", on a 202 end. [US 2021/0347526 A1](https://patents.google.com/patent/US20210347526A1/en). Also 0.990 x 0.625 in in [US 5,715,964](https://patents.google.com/patent/US5715964A/en) |
| rivet to nose's end | 8.0 mm (so 17.0 mm to the tail's end) | "Nose Length from Rivet" 0.305 to 0.317 in. [US 8,646,643 B2](https://patents.google.com/patent/US8646643B2/en), table 1 |
| tab's curled edge, thickness | 1.5 mm | No direct figure found. From US 5,715,964: nose curl 0.070 in, "about 8 to 20 % thicker than the thickest region of the pull ring", so 0.058 to 0.065 in. Tab stock itself is about 0.28 mm |
| tail's underside above the lid | 1.3 mm | No measured figure for a production end found. US 7,909,196 B2 gives the height of the tab's lift end above the panel as "typically greater than 0.050 inches", "preferably 0.050 to 0.100 inches" (1.3 to 2.5 mm). It does not say if that is to the underside or the top. The low end is used as the gap |
| force to open | not reproduced (see below) | 202 stay-on-tab end: pop at most 31 N (average 20 N), tear at most 45 N. [Maker's product page](https://www.eoedrd.com/beverage-ends---stay-on-tab/beverage-ends--202-sot-loe.html) |

The robot's fingertips, measured on the gripper's meshes: each ends in a tongue 15.5 mm wide and 3.95 mm thick. Its
end is rounded on the outside and square (about 0.25 mm) where it meets the flat pad.

What we simplified: the can is fixed in place. The tab and panel are rigid plates on hinges, so the tab does not
bend. The seal is a sprung catch under the panel's edge that the panel cams aside, not tearing metal. It gives way at
a few newtons at the tail (an estimate from the spring and the hinges' friction, not a measurement), less than a real
end's 20 to 31 N. The tab and panel weigh 3 g each (real ones are lighter) for stable contacts. The lid is flat: it
has no finger well and no countersink groove. The tab's nose is bent up 3 mm and presses the panel through one nub.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=open_drink_can.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the can." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/open-drink-can-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=open_drink_can.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the can." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/open-drink-can-demo

# A wrong way, which scores 0: the fingertips push the panel and the tab's nose straight down
CAN_DEMO=poke inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=open_drink_can.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the can." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/open-drink-can-poke

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/open-drink-can-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the can has been open, with the hands off it, for
1 s; otherwise it scores the partial credit below at the last step. With `CAN_DEBUG=<file>` set, each control step adds
a line to that file: the tab's and panel's angles, the grade, and where the left hand's two fingertips are.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| can | a closed 12 fl oz (355 ml) aluminium drink can with a 202 end, red, 6.6 cm across, 12.27 cm tall, upright in a holder (fixed) | (26, 0) |
| lid | aluminium, 5.1 cm across inside the rim, its panel 4.6 mm below the rim's top | on the can |
| tab | aluminium stay-on tab, 2.5 x 1.57 cm, its edge curled to 1.5 mm thick, its underside 1.3 mm above the lid; riveted at the lid's centre, 0.8 cm from its nose's end and 1.7 cm from its tail's end; tail away from the arms (+x), nose over the panel | on the lid |
| panel | the scored opening, 1.7 x 1.4 cm, on the side nearer the arms (-x), hinged 4 mm from the rivet | in the lid |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`. A fresh closed can for every
trial.

## Scoring
Measured on the can: how far the tab's tail has been raised about its rivet (the most this trial, recorded as it
happens and cleared at each reset), and how far the panel is pushed into the can about its hinge. **Hands off:** no
point along either gripper's finger pads, and neither fingertip's end, within 1.5 cm of the can's bounding box (rim
included). The pads and fingertips come from the simulator each step; in a real trial, from a photo or the video.

| Score | When, at the end |
|---|---|
| 1.0 | the panel pushed in at least 45 degrees, the tab having been raised at least 10 degrees (opened with the tab; it may be folded back down after), and both hands off the can. Held for 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.75 | opened so, with a hand still on the can |
| 0.5 | the seal broken (the panel in at least 10 degrees) with the tab raised at least 3 degrees |
| 0.25 | the tab raised at least 10 degrees but the seal holding |
| 0 | anything else. This includes a panel poked in with the tab never raised. It also includes any trial in which a fingertip goes below the rim's top on the opening's side of the rivet (over the tab's nose or the panel) while the seal still holds (the panel in less than 10 degrees) |

Why the last rule: pressing the tab's nose straight down also turns the tab and pushes the panel in, with no finger
ever under the tail. The can has to be opened by lifting the tab's tail. The fingers may go anywhere once the seal
has broken. In a real trial the video shows it: did a fingertip go into the lid's recess on the opening's side before
the can opened.

The tab only has to have been up 10 degrees because the panel turns about 4 degrees for each degree of tab: it is
fully in with the tab at about 25 degrees. The panel stays where it is pushed (it can't come back up), and the tab
where it is left, so many people fold the tab back down over the open can; the score counts the most the tab was
raised. The hands have to be off the can at the end, so the trial ends on an open can that nothing is holding.

## The scripted demo
It plays a fixed plan from the fixed layout, with the arm's own inverse kinematics. It reads nothing from the
simulator. It does not pinch anything.

1. The left hand, wide open, leans 15 degrees and turns 15 degrees. Its lower fingertip comes down onto the lid beside
   the tail's +y corner, 1.2 mm short of it and inside the rim. The other finger stays 2 cm above the can.
2. The fingertip slides towards the tail. Resting on the lid at that lean, its square pad edge is 0.55 mm up, under
   the tail's round edge, and its pad faces the tail like a ramp. It is aimed 8 mm past first touch, so the arm keeps
   it pressed against the tail's corner, 17 mm from the rivet.
3. The hand rises 9 mm. The pad carries the tail's corner up on its arc, the seal breaks, and the panel folds in. The
   tab stops near 24 degrees, with the panel 85 degrees in.
4. The hand comes off the tail, closes its fingers, and presses the tab back down flat, so the open panel is
   uncovered. Then it opens and hovers over the lid; its wrist camera looks down at the opened lid as the trial ends
   on success, 1 s after the hand is clear of the can. The right arm stays clear.

**The demo video** is the run's top camera (left) and the left wrist camera (right) side by side, each at twice its
320 x 176 size, from the same run. The opening is 17 x 14 mm, about 3 pixels in the top view, so the wrist camera is
the one that shows the panel pushed in: the dark opening beside the folded tab in the last frames. The setup image is
the top camera's first frame beside the wrist camera's last (the opened lid).

## Mechanism module
`mechanism.py` is copied unchanged from robocurve/benchmark-tasks pull request #223 (`tasks/spring-plunger-panel`),
by way of the accepted `tasks/install-processor`, under the MIT License (its header carries the licence). It adds a
fixed mechanism with hinged and sliding links, springs and friction to a `yam_arena` task file, and reports each
joint's position to the scorer.

## Revision notes
Answering the review of the second version ("the added 3.6 mm round tail grip enables a pinch on a substantially
thicker, raised feature than the real tab edge"):

- The round tail grip is gone. The tab now has an ordinary tab's length, width, edge thickness and height off the lid,
  each with its source in the table above. The lid panel's depth is the published one (4.6 mm; it was an estimated
  6.3 mm).
- The matching can for real trials is named above.
- The demo opens this can with the robot's own parallel grippers, unmodified, inside the run limit. The engagement
  changed from a pinch to a lever-up, because a real tail cannot be pinched; the title and text say so.
- The scoring gained one rule (a fingertip on the opening's side before the seal breaks scores 0), and the hands-off
  check now also counts the fingertips' ends.

## Runtime and seeds
- The scripted demo ends on success at step 261 (13 s), well inside the 600-step limit.
- Official runs (2026-11-19, this version): the demo 1.0 twice (seeds 0 and 1), each ending at step 261 with the tab
  raised 24 degrees at most, the panel 85 degrees in and the hand hovering clear; the wrong way (`CAN_DEMO=poke`, the
  fingertips pressing the nose and panel down) 0, although it pushed the panel 85 degrees in; Dream-1 0.0 over the
  full 600 steps (the tab and panel never moved).
- The scoring and reach checks run without the simulator: `python -m pytest -q test_open_drink_can.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
