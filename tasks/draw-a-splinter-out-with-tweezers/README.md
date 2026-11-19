# Draw a splinter out of a fingertip with tweezers

A life-size silicone hand lies palm up on a pine block with a 15 mm wooden splinter in its index fingertip: it went in
away from the robot at 25 degrees to the skin, 11 mm is buried and 4 mm stands out. Slant-tip tweezers lean in a
medicine cup beside it. The robot takes the tweezers, closes their slanted tips on the stub by squeezing the legs with
its own fingers, and draws the splinter back out along the line it went in, as MedlinePlus and Mayo Clinic advise. The
score climbs with how far it comes out (calipers at the ink line); full credit needs it out with the entry untorn
(the slit grown no more than 1 mm over the trial), the finger not dragged, and the tips lifted clear. A gripper finger
touching the splinter, pushing it in, pinching the skin between the tips, or pulling the hand off its block scores 0.
The right arm does the work; the left stays parked.

Instruction: "Take the tweezers from the cup and pull the splinter out of the fingertip, gripping it by its end and drawing it out at the same angle it went in. Use only the tweezers on the splinter and don't pinch the skin."

## Files

| File | What |
|---|---|
| `draw_a_splinter_out_with_tweezers.py` | The task file: the scorer first (`_track`, `grade`), then the props, then the scripted `demo()` |
| `splinter_geometry.py` | The props' numbers and frames: hand, splinter track, tweezers, cup, and where the tweezers rest |
| `benchkit.py`, `benchkit_sim.py` | Box-built tools with sprung slides and turns, turned boxes and joint frames, and per-step hooks, added without modifying yam_arena |
| `tiltgrip.py` | Tilted grips for the demo's waypoints (a wrist pitch; IK still checks joint limits) |
| `test_draw_a_splinter_out_with_tweezers.py` | Offline tests, no GPU: every outcome and loophole, text against code, a replay of the demo |

No third-party assets: every shape is built from boxes in the task files.

## Run a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), from this folder:

```bash
I="Take the tweezers from the cup and pull the splinter out of the fingertip, gripping it by its end and drawing it out at the same angle it went in. Use only the tweezers on the splinter and don't pinch the skin."
T=draw_a_splinter_out_with_tweezers.py
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream -E task=$T -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$I" --scorer task_score --max-steps 1800 --store-frames \
  --log-dir ~/yam-eval/splinter-dream
# The scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=$T -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=352 --instruction "$I" --scorer task_score --max-steps 1800 --store-frames \
  --log-dir ~/yam-eval/splinter-demo
for log in ~/yam-eval/splinter-*/*.json; do inspect-robots video "$log"; done   # one MP4 per camera
```

The run prints `task_score`. A trial ends at 1.0 (which includes 1 s at rest); otherwise it scores the last step.
Offline: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q` with `yam_arena` importable.

## Runtime and seeds

- The demo plans 27.1 s (602 steps with the scripted policy's 3 s hold): it takes the tweezers by 4.5 s, sets
  the open tips down either side of the stub at 16.3 s, closes them over 2 s while easing back 3 mm, draws the
  splinter out over 5 s, then holds still. In the simulator the recorded demo reached 1.0 at 25.3 s (505 steps),
  which includes the 1 s at rest, and the trial ended there, so its last frame is the finished, held state.
- The run limit is 1,800 steps (90 s at 20 Hz), 3.6 times the recorded demo, room for a learned policy's retries.
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.

## Props

| Object | Real item | In the sim |
|---|---|---|
| Hand | Pose-able left female silicone mannequin hand, life size, about 21 x 11.4 x 6.4 cm, platinum silicone over wire, about 0.5 kg (to weigh); sold as a display prop ("Pose-able Left Female Silicone Mannequin Hand"), about US$65. The fingertip pad takes about 4 needle tracks, 3 mm apart, before the hand is replaced | Colliders: boxes at full size (four fingers, palm, thumb, wrist), not drawn; drawn as the same shapes with rounded fingertips, softened edges and skin creases; loose on the block at 0.5 kg |
| Fingertip pad | The index fingertip's last 28 mm of silicone | A 4 mm pad that dents on a 1 N/mm spring (a model value), 3 mm at most |
| Splinter | 15 mm of a Royal Paper R820 round birch toothpick, 1.8 mm across, stained dark brown | Drawn round, 1.8 mm, dark; collides as a 2.4 mm square stick (thinner colliders tunnel) on a track along its axis, 1.0 N to slide, two sprung turns at the entry; once out it stays on the track, stopping 16 mm out, rather than coming free; it can't snap |
| Scale label | 25 x 7 mm printed paper mm scale, glued with silicone adhesive on the fingertip's right side | Drawn only, with 1 mm ticks |
| Side camera | 1080p, about 25 degrees field of view, floor tripod off the table's right edge at x 27, y -70 cm | Real bench only |
| Tweezers | Tweezerman slant tweezers, stainless, 95 mm long, 10 mm wide, 25-degree slant tips, about 15 g | Joined end plus two legs on sprung slides (3,000 N/m) rather than bending from the joined end, so the tips close as far as the grip section is squeezed; legs 2 mm thick (real about 1 mm), 6 mm tip gap; the grip section meets the fingers through five 3 mm raised pads; slanted tips as turned boxes |
| Cup | Choice 1 oz graduated medicine cup, polypropylene, 47.6 mm top, 28.6 mm bottom, 34.9 mm tall | Three stepped rings of 12 sides (floor, middle and rim widths), 2 mm walls, 3 mm floor, fixed |
| Block | Pine, 15 x 15 x 2 cm, screwed down | Fixed box |

## Scoring

Scored from the side video (level with the fingertip, the label's mm scale beside it) and the top video; after the
trial, calipers, a steel rule and a 0.5 mm blunt steel-wire probe. *Drawn out* is how far the splinter has come out
along its track, from what stands out at the ink line (4 mm at the start); *out* means no part under the skin (11 mm
drawn). Read at the end.

| Outcome | Score |
|---|---|
| Out, not torn, fingertip moved 5 mm or less, tips 5 mm or more off the finger, all still 1 s | 1.0 |
| Out, but torn (slit grew over 1 mm), the finger moved over 5 mm, or the tips still on the finger | 0.75 |
| Drawn out 6 mm or more, still in | 0.5 |
| Drawn out 3 mm or more | 0.25 |
| Drawn out under 3 mm, or nothing done | 0 |
| Snapped (real only): the piece left in, by its own draw | at most 0.5 |
| A gripper finger touched the splinter, at any time | 0 |
| Pushed in 2 mm or more, at any time | 0 |
| Squeezed tips dented the pad over 1 mm (skin pinched), at any time | 0 |
| Hand off its block (moved 2 cm, or tipped 10 degrees), at any time | 0 |

The zeros stick even if undone; drawn-out credit is read at the end, so a splinter pushed back loses it.

- **Torn** is relative to each trial's own start: the slit is measured at reset (track empty) and again after the
  trial, and a tear is growth over 1 mm (`torn_on_bench`). Reset makes a fresh track when the slit is over 2 mm, so
  track wear can't decide 1.0 against 0.75. The slit, step by step: push the probe 1 mm into the entry, square to the
  skin; slide it along the opening until unbroken silicone stops it and dot the skin at its far side with a 0.1 mm
  fine-liner; do the same at the other end; read the calipers' knife-edge jaw tips on the two dots; wipe the dots off.
- **A finger touches the splinter** on the bench when a frame of the side video shows no gap between them, unless the top video shows a gap at that moment.
  The sim tests the same thing: each finger is a box (its last 40 mm, 15.5 x 4 mm, from its pad face out, placed from
  the gripper link and finger joints) that touches when within 0.5 mm of the splinter's collider, along the part out
  of the skin (`fingered`). An open finger brushing the stub counts even though the pads' midpoint is centimetres away.
- **Snapped** (real only, and rare: birch bends to about 100 MPa, about 14 N across a 1.8 mm stub 4 mm out):
  the piece left in the finger is scored. Measure what of it stands out at the ink line at the end, pull it out and
  measure its length; its drawn out is 11 mm less the part that was under the skin (`snapped_score`). Nothing left
  in scores 0.5.
- **In the sim** the entry tears when the load across the stub passes 1.5 N while some of it is still in; that load
  is the silicone's give (two sprung turns at the entry, 0.027 N m per radian) over the lever from the entry to the
  tweezer tips, so a pull about 6 to 13 degrees off the splinter's line tears it.

## Design notes

- **Doing nothing scores 0,** and every cheat scores 0 or is capped at 0.75, below an honest full draw.
- **Why a tear caps rather than zeroes.** A torn entry still leaves the splinter out; it is worse first aid, not a cheat,
  so it scores below the clean draw but above a half-drawn one. The angle rule in the instruction is scored only here,
  which is why the tear is read as growth against the trial's own starting slit, not against a fixed length.
- **Model values, not calibrations.** The pad's 1 N/mm, the track's 1.0 N and the 1.5 N tear load set how the sim
  behaves; the real-bench score uses none of them. It reads the dent on video, the length drawn with calipers and the
  slit's growth with calipers and probe, so measuring them on the real hand would change the sim, not the bench
  score. Reset does check the real track holds the splinter (at least 0.5 N on a push-pull gauge).
- **What the reviewer's camera sees.** The hand's colliders are plain boxes, but they are drawn rounded with creases;
  the splinter is drawn round and stained dark so it shows on the skin, and the mm label sits on the fingertip.
- **Unintended strategies, all tested:**
  - Pinching the stub with a gripper finger and pulling, or brushing it with one open finger: 0.
  - Pulling straight up or levering it sideways: the stub turns in the silicone and the load across it passes 1.5 N:
    torn, at most 0.75.
  - Pushing it in, even then drawing it out: 0.
  - Pressing the closing tips into the pad to dig under the stub: the pad dents over 1 mm: 0.
  - Dragging the hand toward the robot to bring the stub closer: 2 cm or 10 degrees is 0; over 5 mm caps at 0.75.
  - Drawing it out, then pushing it back: credit is read at the end.
  - Levering once it is out tears nothing, and where the tweezers end up once clear doesn't matter.
- **Not punished:** retries, regrasps, resting the open tips on the skin, a 1.5 mm push in, the tweezers' final place.
- **Why the tips rest on the skin.** Slant tips are made to lie flat on the skin; the demo sets them down either side of
  the stub and lets the arm give, so a sag of a few millimetres doesn't matter. Only squeezed tips pressed over 1 mm in
  count as a pinch. The tips reach 1.5 mm past the ink line and ease back 3 mm as they close, so they straddle the whole
  stub even if the tweezers sit a little back in the grip, and squeezing never drives it in.
- **The cup.** A narrow bottle would leave only the tweezers' top 2 cm above its rim to hold, where squeezing barely
  closes the tips; the shallow medicine cup leans them at 40 degrees and leaves the grip section, a third of the way
  down, clear of the rim.
- **Reach.** The fingers come down the tweezers' own line, pitched 40 degrees, onto the grip section, the fingertips
  outside the cup's rim. At the finger the wrist pitches on to 65 degrees so the tweezers lie along the splinter; every
  waypoint solves within 0.1 mm in the simulator's IK.

## Videos

Both runs used the commands above in the simulator, with this code.
- **Scripted demo: 1.0.** It takes the tweezers from the cup, sets the open tips either side of the stub, closes
  them and draws the splinter out along its line; 1.0 (out, untorn, tips clear, 1 s still) at 25.3 s, 505 steps.
  The camera on the working arm's wrist shows the dark splinter come out between the tips.
- **Dream-1: refused at step 32,** with the message "Using sharp tools like tweezers directly on a person's skin is
  unsafe.", so it has no score. The hand is a silicone mannequin hand; the message is reported as it was given.

## Safety

Real trials are safe: the splinter is a piece of a blunt-cut toothpick in a silicone mannequin hand, the tweezers'
tips are blunt-edged steel, nothing is heated or powered, and the pull is about a newton. The person resetting stays
out of the arms' reach while a trial runs.
