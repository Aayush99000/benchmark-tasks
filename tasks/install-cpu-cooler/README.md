# Install an AMD Wraith Prism cooler without lifting it off its pre-applied paste

An AM4 motherboard (a B550 board in real trials) lies on the table with a Ryzen processor in its socket. An AMD Wraith
Prism, AMD's boxed cooler for AM4, stands beside it on a foam support with a hole in the middle, so its pre-applied
paste touches nothing before the trial. It has its stock clip: a strap at each end with a
window that hooks over a lug on AM4's standard retention brackets. The left strap is fixed; the right one hangs loose
from a black cam lever that lies along the shroud's side. A precision flat-head screwdriver stands in a rack beside the board.

The cooler's base carries pre-applied thermal paste, as AMD's boxed coolers ship. AMD's instructions say that a
heatsink lifted off after installation must not be put back until both surfaces are cleaned and new material applied,
and that the material must not be scratched or have gaps (AMD FM2+ installation guide, "Read This First"; AMD Builder's
Guide for Opteron systems, doc. 30925 rev. 3.04, p. 23). So once the paste touches the processor the cooler must not be
lifted off again, nor slid so far that the paste is wiped off part of the processor. The finished install looks the
same either way, so the rule is scored during the trial.

The robot has to:
1. Set the cooler on the processor and hook the fixed strap: set it down a few millimetres towards that end, the
   strap outside its lug, and slide it back so the lug goes into the window. Once its paste touches it must not lift
   it off again or slide it more than 10 mm.
2. Press the lever down with the screwdriver until the loose strap rides over its lug and hooks.
3. Swing the lever 180 degrees over the top with the screwdriver until it snaps down, locked.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, `success()`, `score()` and the scripted `demo()` |
| `clip.py` | The clip's springs and the lever's over-centre spring and cam, applied before every physics step |
| `test_paste.py` | Checks of the paste rule on made-up cooler poses, without Isaac Sim: `python test_paste.py` |
| `touch.py` | Robot-to-cooler contacts from PhysX, for "released" (no part of the robot touches the cooler) |
| `scenekit.py` | USD bodies and articulations for yam_arena task files (hooks yam_arena without changing it) |
| `assets/board.usda` (6.9 MB) | The ATX board, static, with the retention frame's two lugs |
| `assets/cooler.usda` (0.1 MB) | The Wraith Prism, a floating-base articulation: body with the fixed strap, the loose strap (a slide and a flex), lever |
| `assets/screwdriver.usda` | The Wiha PicoFinish 260P 3.0 x 50 mm screwdriver, a rigid body |

The board's visual comes from pc-anatomy (MIT, github.com/Yoosseph/pc-anatomy), without printed text; its VRM row beside the socket is moved 14 mm away, leaving the clip room. The cooler is built from simple shapes
at the Wraith Prism's sizes (AMD's listing: 119 x 105 x 93 mm, 545 g, 92 mm fan, four heat pipes). Collision shapes
are boxes; the shroud's round opening is a square hole of the same width, with the fan as a slab 10 mm down. The
clip and lever follow LanOC's photos of the Wraith Max, which uses the same clip (lanoc.org, "Ryzen ITX cooler
roundup", 2017): the straps run down the fin block's ends under the shroud's edge, and the lever lies against the
shroud's side, its pivot about 8 mm above the shroud's bottom. Their positions and sizes are estimated from those
photos, not measured. The fixed strap is part of the cooler's body. The loose strap and the riser up to the lever's
pivot are one sprung slide, so pressing the lever down drives the strap down; the strap's lower part flexes outward
over the lug. The lever turns on the riser, and its over-centre spring and cam (`clip.py`) lock it and pull the strap
tight.

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Install the CPU cooler on the processor. Once its thermal paste touches the processor, don't lift it off again or slide it more than 10 mm. Hook the fixed end of its clip over the lug first, then use the screwdriver to press the lever down until the loose end hooks over its lug, and swing the lever over the top until it locks." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/cooler-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Install the CPU cooler on the processor." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cooler-demo
```

What the demo does:
- **Left arm, the cooler:** it puts its closed fingertips 8 mm down into the shroud's fan opening, above the fan, and
  opens them until the pads' backs press the opening's wall from inside. It carries the cooler over the processor and
  sets it down 5.5 mm towards the fixed end, the fixed strap outside its lug, then slides it back so the lug goes into
  the strap's window. It closes its fingers off the shroud's wall, lifts them out and moves away.
- **Right arm, the screwdriver:** it waits until the left arm has gone, then takes the screwdriver upright out of its
  rack. It presses the lever down with the tip until the loose strap rides over its lug and hooks. Then it puts the
  shaft beside the lever's raised end, low, and moves it towards the pivot: the shaft's side pushes the end, which
  rides up the shaft, until the lever is past straight up and its spring takes it down, locked. The hand stays low
  throughout. It drops the screwdriver in the cup.

At 20 Hz the demo reaches 1.0 (success) in about 1,040 steps (52 s). Once the base touches the processor it moves 6.3 mm
at most (the 5.5 mm slide onto the fixed strap's lug and settling) and never lifts off, inside the paste rule's 10 mm.
It is open loop with `-E object_jitter=0`.

## Scoring

`test_paste.py` checks the paste rule on made-up cooler poses.

The score at the end of the episode counts (`SCORE_OVER = "end"`).

- **Paste rule:** from the step at which the base first touches the processor (its lowest corner within 0.7 mm of the
  processor's top, the base overlapping it), its centre may move at most 10 mm sideways from there on every later step,
  at whatever height (sliding while hovering under the lift limit counts too), and the base must not lift off again
  (lowest corner more than 3 mm above the processor's top, or moved off it). Checked
  every control step from the cooler's pose (`paste_watch` in `task.py`); the paste itself isn't simulated.
- **Seated:** the cooler's base on the processor: its centre within 3 mm of the processor's, square with the socket
  within 5 degrees, tilted less than 3 degrees, its bottom within 1.5 mm of the processor's top. Seated, the fixed strap
  is hooked: lowered straight, it lands on its lug and holds that end about 5 mm up.
- **Released:** no part of the robot touches the cooler's body, from PhysX contacts between the robot's links and it,
  within 1 mm; and the screwdriver's tip is more than 80 mm (sideways) from the cooler's centre, so it isn't left
  resting on it.
- **Loose end hooked:** the loose strap has gone down 7.1 mm (+0.6/-1.0 mm), so its window's bottom edge is under its
  lug, and it hangs straight (within 1.5 degrees of upright), the lug through its window.
- **Locked:** the lever has turned more than 150 degrees of its 180 degree travel (past straight up its spring takes it
  there and holds it down along -x).

| Outcome | Score |
|---|---|
| The paste rule is broken (lifted off, or slid more than 10 mm), or the cooler is not seated (fixed end hooked) on the processor, or not released | 0 |
| Seated and released | 0.25 |
| ...and the loose end hooked | 0.5 |
| ...and the lever locked, for 1 s (success) | 1.0 |

How to check it on real hardware:
- Lift: a side camera on the board's far side, 80 mm from the processor and 16 mm above the table (just over the
  memory slots), looking along x: a lift shows as a gap between the fin block's bottom edge and the retention
  bracket's top, the 92 mm fin block giving the scale for the 3 mm limit.
- Slide: a top camera, and two 3 mm dots marked on opposite corners of the shroud's top; measure their movement from
  the frame in which the base first touches.
- The paste print on the processor when the cooler is lifted off for the reset: one clean
  square passes; a doubled or torn print (lifted off) or one wiped more than 10 mm breaks the rule.
- A top photo shows whether the cooler sits square on the processor.
- A side photo of the loose end shows whether its lug shows through the window and the strap hangs straight (its
  bottom no more than 1 mm further out than its top).
- A side photo shows whether the lever is down.

## Runtime and seeds

A filmed scripted-demo run takes about 10 minutes on the cluster's GPUs (RTX 5000 Ada, A40), including Isaac Sim's
start.

The layout is fixed (`-E object_jitter=0`), so there are no seeds to choose. Dream-1 is a hosted model without a seed,
so its trials can't be reproduced bit for bit; the demo can.

## Submitted videos and scores

- Dream-1 run, with the instruction above: 0.0 (2,400 steps; the left arm reached the cooler on its support and
  pushed against its side without lifting it, and the right arm took the screwdriver out of its rack and held it near
  the board, so the cooler never reached the processor).
- Demo: `task_score = 1.0`.
