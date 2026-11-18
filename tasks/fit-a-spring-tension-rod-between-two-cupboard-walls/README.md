# Fit a spring tension rod between two cupboard walls

Two cupboard walls stand 300 mm apart. A spring tension rod, 325 mm long at rest, lies in front of them. The robot
squeezes the rod shorter, sets it between the walls at the marked height and level, and lets go; only the rod's
spring pushing its rubber tips against the walls holds it up, hands-off for 10 s and under a 300 g test weight. Full
credit (1.0) also needs it in the band, level (tip centres within 10 mm in height) and square across the gap (within
10 mm front to back); a rod let go and holding itself up between the walls but outside those limits scores 0.5.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment
for Isaac Lab-Arena), on the `station` scene. It uses `parts.py` (in this folder) for the walls with their marked
band and for the rod, a loose part whose inner bar slides on a spring. `near_misses.py` lists near-successes and
wrong outcomes with the score each must get; they are plain-Python checks of `score()` and `success()` and need no
simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Fit the spring tension rod between the two white cupboard walls, level, at the yellow band."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/tension-rod-dream

# Scripted demo (reaches 1.0 in about 23.5 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/tension-rod-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 40 s of simulated time (800 steps at 20 Hz); about 1 minute of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Runs use `-E object_jitter=0`, so every object starts exactly where `task.py` puts it.
  With jitter on, seeded resets shift only the rod.

## The rod

**Off the shelf.** The model has the diameter of a 7/16 in (11.1 mm) round spring tension curtain rod for 11 to
18 in (279 to 457 mm) openings, for example Graber's 7/16 in round spring tension rod, 11 to 18 in (sold in packs of
two), and is set to 325 mm, within that range. Like other spring rods it is set about 1 in longer than the opening,
squeezed shorter, put in place and let go: Kenney's buying guide for its Twist & Fit shower rods says to extend a
spring tension rod 1 to 1.5 in beyond the opening. Here it is 25 mm over the 300 mm gap. The retail rod's spring
rate and tip size are not published; if one is used, measure and note them, since a different spring holds the rod
with a different force. The model's colours (blue outer tube, grey inner tube, black tips) are for the simulator's
camera view; the retail rod is white.

**Built to the model's sizes** (what the simulator uses):

| Part | What to use |
|---|---|
| Outer tube | Thin-wall (0.014 in) round brass tube, 7/16 in (11.1 mm) OD, 180 mm long (about 18 g), painted blue |
| Inner tube | Thin-wall (0.014 in) round brass tube, 13/32 in (10.3 mm) OD, 170 mm long (about 16 g). Hobby telescoping tube (K&S Precision Metals, for one) comes in sizes 1/32 in apart made to slide one inside the next; the outer tube's bore is the sliding guide, over at least 45 mm of overlap (95 mm at full squeeze). No separate bushing is needed |
| Squeeze mark | A red line painted round the inner tube 5 mm out from the outer tube's mouth with the rod at its full 325 mm. It disappears into the mouth once the rod is squeezed 5 mm |
| Spring | Music-wire compression spring, 0.7 mm wire, 9 mm OD, 21 active coils (23 total), 75 mm free length: about 200 N/m (1.1 lb/in), about 16 mm solid, so 50 mm of squeeze (25 mm long) never closes it. About 2 g |
| Spring seats | A plug glued in the outer tube 60 mm in from its tipped end, and a plug glued in the inner tube's inner end. The spring sits loose between the two plugs, unattached, just touching both at the free length (no preload) |
| Travel stop | A 1.5 mm steel pin pressed across the inner tube 5 mm from its inner end, its ends running in two opposite 1.6 mm by 51.5 mm slots cut along the outer tube (50 mm of travel for the 1.5 mm pin). At rest the spring holds the pin at the slots' outer ends (325 mm, free length); squeezed 50 mm, it reaches the inner ends. The pin also keeps the tubes from coming apart |
| Tips | A black rubber leg tip for 7/16 in tube on each end, 19 mm across and standing 10 mm beyond the tube end, about 4 g each. The inner tube is 0.4 mm per side smaller, so build it up with about three turns of 0.13 mm PVC tape first |
| Ballast | To reach the model's 60 g and 40 g: a lead slug 10 mm across and 39 mm long (about 35 g) glued in the outer tube between its spring plug and its tip, and a lead slug 9.5 mm across and 24 mm long (about 19 g) glued in the inner tube near its tip end. Each half then weighs within a few grams of 60 g and 40 g (100 g in all) |

## The physics

Fitted, the rod is 25 mm shorter than its free length, so its 200 N/m spring presses each tip on its wall with
5 N. With the rod's friction (1.0) and the walls' (0.6), averaged by the simulator to 0.8, the two tips can hold
about 8 N: the rod's 0.98 N weight plus the 2.94 N test weight about twice over. A retail rod's spring is probably
stiffer (its rate is not published). The model's rate is set by what the demo's two-arm squeeze can do: the arms
give under load: at 280 N/m (each arm moving 40 mm in) they compressed the rod only about 19 mm, too little to clear
the 300 mm gap; at 200 N/m (50 mm each) they reach about 30 mm. Set down without squeezing,
the rod does not fit between the walls; released before both tips bear, it drops; set crooked or at the wrong
height, it may hold but out of true, which scores 0.5.

## Scoring a real trial

Everything is judged by eye, with a sheet of paper, a ruler, a pencil and a 300 g weight, after the robot has let
go. The 10 s hold must finish within the 40 s run limit, as in the simulator, and no part of the robot may touch the
rod or come within 5 mm of it after the last release, to the end of the trial.

1. **Let go:** no part of the robot within 5 mm of the rod.
2. **Tips on the walls:** slide a sheet of office paper down each wall's inner face over the tip: the tip stops it
   (it cannot pass between tip and wall). Each tip's centre is between its wall's front and back edges and not above
   the top. A tilted or skewed rod counts if it passes; the placement limits in step 5 decide 1.0 or 0.5.
3. **Squeezed:** the red line on the inner tube is hidden inside the outer tube, so the rod is at least 5 mm shorter
   than its 325 mm free length and its spring is pressing both tips. The rod is at least 3 mm clear of the table.
4. **Holds itself up:** keep hands off for 10 s. For the first 5 s the rod must not visibly move. At 5 s mark each
   tip's edge on the wall in pencil and hang a 300 g weight (a loop of string with a 300 g bag) gently from the
   rod's middle; while it hangs, neither tip may move more than 2 mm from its mark and the rod must not slide or
   fall. Take the weight off at 10 s; the rod must then stay still.
5. Passing 1 to 4 scores at least 0.5. **Full credit (1.0)** also needs each tip wholly on its wall (centre at least
   9.5 mm, the tip's radius, inside both edges; top not above the wall top), both tip centres 120 to 150 mm above
   the table (the yellow band), within 10 mm of the same height and within 10 mm of each other front to back
   (ruler).

## How the simulator scores it

The simulator makes the same checks from exact geometry:

- **Tips on the walls (step 2):** the rod's bounding box reaches within 0.5 mm of each inner face (y = +-0.150 m)
  and not past it by more than 0.5 mm (`SEAT`). This allows for contact offsets; in the GPU runs a fitted rod's
  ends sit at the faces to 0.1 mm. A released, squeezed, still rod cannot rest with a gap at a tip, since only the
  walls can push back on its spring, so this matches the paper check. Tip centres come from the rod's two links
  (outer and inner tube) and its slide position: between x 0.250 and 0.420 m and not above the wall top.
- **Squeezed (step 3):** the slide is at least 5 mm in (`SQUEEZED`), the red line's position. The rod's lowest point
  is at least 3 mm above the table. The rod must also be still: moving under 1 cm/s and turning under 0.5 rad/s.
- **Holds itself up (step 4):** an `on_step` hook times how long the rod has met steps 1 to 3 continuously. From 5 s
  to 10 s into that hold it applies a downward force at the rod's middle, 2.94 N (the 300 g weight), rising over the
  first 0.5 s and falling over the last 0.5 s, as a weight is hung and taken off by hand. While it is on, the rod
  need not be still (a hung weight jiggles it), but if either tip moves more than 2 mm from where it was when the
  load went on, or any other check fails, the clock restarts; before and after the load the rod must be still. No credit
  (0.5 or 1.0) before the hold reaches 10 s. The force is a test load only; the robot never touches it.
- **Full credit:** the step 5 limits, then the 1.0 state must also hold for 1 s (`SUCCESS_HOLD_S`), after which the
  trial ends. Otherwise the last step is scored, and a rod whose hold has not reached 10 s scores 0.
- **Let go (step 1):** every control step the hook measures the closest distance from every link of both arms
  (base, links 1 to 5, gripper and both fingertips) to the rod. Each link's collision hulls (the URDF's) are
  sampled once over their whole surface, corners, edges and faces, with neighbouring points at most 2 mm apart,
  so a finger or forearm face lying flat on the rod counts even when its corners are far away; the samples
  overstate a distance by at most 2 mm / sqrt(3), about 1.2 mm. The rod's tubes and tips are measured as their
  bounding boxes, which is stricter near their round edges. Under 5 mm, the hold restarts and the rod earns
  nothing. Every limit is compared with a 1e-6 margin.
- **Fails closed:** before the first measurement after a reset, the release and hold checks give no credit.
  `near_misses.py` sets a flag so its offline cases stand for states with the robot away and the hold passed; its
  `_run()` (run `python near_misses.py`) also checks the release rule with an arm link at 0, 4.9 and 5.0 mm, and
  plays the hold clock: the load on from 5 s to 10 s, no credit at 9.9 s, credit at 10 s, and a restart when the
  tips slip 3 mm under the load, the rod moves before the load, or a robot part comes back;
  a brief jiggle while the load is on does not restart it. It also lays a link face flat over the rod 0.5 mm away
  with all its corners more than 5 mm off (a synthetic 100 mm plate, and the largest hull face of a fingertip,
  the gripper, link 5 and link 3): each reads 0.4 to 0.6 mm and scores 0. Measuring hull corners alone, as an
  earlier version did, reads 5.1 to 14.5 mm for the same faces and would have credited them.

## Scripted demo

`demo()` drives both arms with the fingers closing front to back across the rod (the left arm turned -90 degrees,
the right +90). Each grips near an end (13.5 cm either side of the midline), they lift the rod to the band's height,
bring it to 25 mm in front of the walls, squeeze it by moving 50 mm toward each other each (the arms give under the
spring; the trace shows the slide at about 30 mm, the rod about 295 mm long), carry it straight back between the
walls to their middle and open together. The carry is set 3.5 mm to the robot's left of the midline, a fixed offset
from the GPU traces, where the squeezed rod sat that far right and its right tip caught the right wall's front
corner. Within about 2.5 s of opening, the arms rise, back out of the gap and move aside, then wait. The rod springs
out to the walls (slide 25.3 mm) with its tip centres about 135 mm up, and holds. In the final demo run
the 10 s hold began at 12.5 s, the 300 g test load was on from 17.5 s to 22.5 s with no measurable movement (under
0.1 mm), and the trial ended at 1.0 at 23.5 s (470 frames).

## GPU traces while building

- The bar's riser first stood partly under the tube end, 3.5 mm too high there; the rod started tilted. The riser
  was moved clear of the tube.
- The grippers, turned 90 degrees, caught the front corner of a wall on the way down to the rod; the rod's start was
  moved 30 mm nearer the robot and an approach waypoint added. Offline check of the path: at least 16 mm from the
  walls while between them, and 12 mm between the two arms at the closest.
- The test load is real: in a copy of the task with the load raised to 5 N, the rod slipped in the first control
  step under it (the hold restarted and the load came off) and fell to the table, scoring 0.
- With a 280 N/m spring and a 40 mm squeeze from each arm, the arms squeezed the rod only about 19 mm, not enough to clear the 300 mm gap; it went in
  only by catching a wall's front corner. The spring was set to 200 N/m and the squeeze increased.
- Applied all at once, the 2.94 N load jolted the rod for one step past the 1 cm/s stillness limit and restarted the
  hold though nothing slipped. The load is now hung over 0.5 s, and stillness is not required while it is on (the
  2 mm slip limit still is).
