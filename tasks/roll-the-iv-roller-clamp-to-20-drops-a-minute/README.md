# Roll the IV roller clamp to 20 drops a minute

A primed gravity IV line runs from a 1,000 mL saline bag on an IV pole beside the table, through its drip chamber
30 cm above the table, along the table and through a roller clamp lying loose on its
tubing, then off the table's edge into a beaker on the floor. The clamp starts rolled fully shut. The robot sets the
infusion as it is set wherever there is no pump: roll the clamp's wheel open while watching the drops, correct until it
drips 20 a minute, let go without nudging it, and return both arms to their start pose. Then the drops are counted
for a full minute.

Every IV without a pump is set this way. It is routine on hospital wards in many low-resource countries, where pumps
cost over US$1,000 and need upkeep and power; in Uganda the setting is often neglected for lack of staff, and wrong
rates harm and kill, children most (Makobore and Mulerwa, *Biomedical Engineering for Africa*, UCT 2019, ch. 15;
Fraunhofer, 2020, on Uganda). It is done by nurses and clinical officers on wards without pumps, by paramedics, most of
whom infuse by gravity (EMS World, Oct 2020), in austere field
care (*Ann Emerg Med* 2017), and by veterinary technicians where there is no pump (AAHA 2024 fluid
therapy guidelines, section 8). The hard part is physical: a fraction of a millimetre of wheel travel
changes the rate, the wheel holds until pushed past its friction so a compliant arm stops short of where it is sent,
the loose clamp must be held while the wheel rolls, and the rate shows only as clear drops seconds apart, so each step
needs a wait and a correction.

## Files

| File | What |
|---|---|
| `roll_the_iv_roller_clamp_to_20_drops_a_minute.py` | The task file: scoring, the drip model, the props, `demo()` |
| `clamp_demo.py` | The scripted demo's waypoints |
| `benchkit.py` | Props built from boxes with sliders (the clamp's wheel), with breakaway friction, and per-step hooks |
| `run_record.py` | The simulator machine's trace (`~/runs`) |
| `tiltgrip.py` | Tilted grips for the demo's waypoints |
| `test_roll_the_iv_roller_clamp_to_20_drops_a_minute.py` | Offline tests: no GPU needed |

No third-party assets: every prop is built from boxes in the task file.

## Run a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key configured, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=roll_the_iv_roller_clamp_to_20_drops_a_minute.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the IV to drip at 20 drops a minute with the roller clamp, then let go and return both arms to their start pose." \
  --scorer task_score --max-steps 4800 --store-frames --log-dir ~/yam-eval/clamp-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=roll_the_iv_roller_clamp_to_20_drops_a_minute.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Set the IV to drip at 20 drops a minute with the roller clamp, then let go and return both arms to their start pose." \
  --scorer task_score --max-steps 4800 --store-frames --log-dir ~/yam-eval/clamp-demo

for log in ~/yam-eval/clamp-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score`. The scorer freezes the outcome when both arms are back in their start pose, as the rule
ends the trial there; the episode stops at once on 1.0 and otherwise runs to the step limit with the score fixed.

Offline tests (no GPU), with `yam_arena` importable: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q`.

## Runtime and seeds

- The scripted demo plans 163 s (3,260 steps at 20 Hz). It pushes the wheel and makes up to eleven corrections, on or
  back, easing off to watch after each, each worked out from the wheel's travel in the sim.
- Dream-1 runs up to the 4,800-step limit (4 minutes of simulated time) unless it reaches 1.0 first.
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.

## The runs

- **Scripted demo: 1.0** (Isaac Sim, 1,873 steps): four rolling pushes take the wheel to 3.5, 6.5, 9.5 and
  10.38 mm (19 drops in the minute); it then stays where it was left while the later corrections ease off, and the
  last frames show both arms back at their start pose.
- **Dream-1: 0.0** at the 4,800-step limit. Its trace shows no gripper within 5 cm of the clamp and the wheel shut
  (0.00 mm) throughout.

## Scoring

`task.json` gives the rule in full. The trial ends when both arms are back in their start pose (each gripper within
5 cm of where it started, after one has come to the clamp), or at the run limit; the drops in the next minute are
counted from a close-up 60 fps video of the chamber.

| Outcome | Score |
|---|---|
| A stream (free flow) for more than 3 s at a stretch, before the end | 0 |
| The tubing at any tape strip moved more than 3 cm (pulled from under its tape), before the end | 0 |
| The line kinked, folded or squeezed outside the clamp, or the slide clamp not fully open, at the end | 0 |
| No drops in the minute (still shut, or shut again) | 0 |
| Over 60 drops in the minute (three times the order) | 0 |
| A gripper still on or within 5 cm of the clamp, or on the line, at the run limit; 1-60 drops | 0.25 |
| Clear at the end; 1-9 or 31-60 drops | 0.25 |
| Clear at the end; 10-15 or 25-30 | 0.5 |
| Clear at the end; 16-17 or 23-24 | 0.75 |
| Clear at the end; 18-22 | 1.0 |

Every zero is a hazard: a fast or free-flowing line overloads a patient, pulling the line at its tape is what tugs a
cannula, and a kink left in the line sets a false rate. Under-rates keep a little credit, as running slow is the safer
error.

## Fidelity

- **Drip model.** Flow through a thin slit goes as the head times the cube of its gap. The 1.0 m head is near what a
  bag hung about 90 cm over a patient's arm gives; the line ends in a beaker, with no back-pressure from a vein. The
  curve is a model, to be calibrated on the real clamp, and real PVC creeps under the roller, so a real rate may drift
  at first.
- **The wheel** is a slider: it moves along the housing without turning, so a fingertip on a 45-degree face moves
  the axle 1:1, where rolling a real wheel by its rim moves the axle about half as far. Static and sliding friction are
  equal (1.5 N, no viscous drag): it moves only when pushed harder and stops when the push falls below it. It weighs
  10 g (about 2 g real) to keep contacts steady, and carries no weight of its own: in a GPU run the hand pinching the housing tipped it 0.4 degrees, and the wheel's weight along its slide
  (under 1 mN) crept it shut at 0.01 mm/s through PhysX's joint friction, which no real wheel does. The compliant arm
  stops it short (about 1.1 mm at 1.5 N in MuJoCo), and a push can carry it 0.3-1.2 mm past, more than the full-score
  band, so the demo learns from each push how hard to press and rolls the wheel back from the bag side when it goes
  over. Whether a fingertip stops within 0.37 mm on the real clamp is for a real trial.
- **Tubing and props.** Beyond the tape strips the tubing is fixed; the 4 cm of free tubing either side of the clamp
  is rigid with it, so the strips' rule is judged by where its ends move. Kinks aren't modelled: a gripper reaching
  the tubing's centreline at the end stands in for a squeeze. The slide clamp is drawn only, fully open; the spare
  coil isn't drawn. Props are boxes at full size: the wheel an octagon, the beaker twelve panels.
- **Drops and cameras.** The chamber faces the top camera 3 cm in front of a matte black board, with at least 500 lux,
  so each clear drop shows as a grey glint (the sim draws only greys). At 320 x 176 the chamber is about 4 x 12 px, a
  growing drop about 1.5 px and the falling drop a 1.5 x 3 px streak for one frame (a real drop falls in about
  0.06 s). On real arms (top camera about 0.95 m up, at 224 x 224) a drop is under a pixel, so the rate is read with a
  wrist camera about 20 cm from the chamber (about 1.7 mm per px; the left arm's IK reaches such poses): each drop a
  3-4 px glint. The wheel's 20 mm is about 12 px in a wrist view, enough for 10-30 a minute, but 18-22 is about
  0.4 mm, so a full score needs the drops.
- **Before real trials:** a push-gauge reading of a real 2C8537 wheel replaces the 1.5 N estimate, and the drip
  curve is checked on it.

## Design notes

- **How the trial ends.** Returning both arms to their start pose ends it; stepping back to watch ends nothing, so a
  policy can let go, watch, come back and correct. One that never returns is scored at the run limit if clear then;
  a gripper still on or near the clamp caps the score at 0.25 (the operator lifts it 10 cm straight up first), so
  holding the wheel or squeezing the line can't make a rate look right.
- **The line can't throttle the flow.** Along the table the tubing is taped flat at each bend and at most 10 cm
  apart, so it can't be folded without pulling it from under a strip (0); a kink, fold or squeeze left outside the
  clamp at the end scores 0, as does a partly shut slide clamp. Touching the line or the bag is not clear.
- **Why the beaker stands on the floor.** The top camera sees no higher than about 40-50 cm above the table, so the
  chamber hangs 30 cm up; a beaker on the floor keeps the chamber's saline 1.0 m above the line's end.
- **Why the clamp lies taped on the table.** At a bedside it usually hangs below the chamber; it lies flat when the
  line rests across a bed or overbed table. Here it lies on the table so every trial starts the same and the line
  can't swing, and because this simulator makes free tubing only as a rigid chain (a hanging clamp would need its
  weight switched off or a ledge no bedside has). The clamp is still loose: the wheel can't be rolled without holding
  the housing, and letting go can nudge it; the table carries its weight, so the hold is lighter than in the air.
- **Unintended strategies, all tested:** roll it fully open (a stream: 0); open wide and back to 20 (0 if the stream
  lasted over 3 s); hold the wheel at 20 to the end (0.25); set 20 then shut it (0); carry the clamp off its tape (0);
  squeeze or kink the line (0 at the end); nudge it on release (scored where it was left); return to the start pose at
  a wrong rate and come back (scored at the wrong rate); open it fast and let go (0); never touch it (0).
- **Reach.** Every demo waypoint solves within 2 mm, and the arms stay at least 10 mm apart (URDF collision hulls).
- **Safety.** Saline into a beaker, no sharp (the spike stays in the bag), no heat; the wheel takes a few newtons and
  the plastic housing is pinched with 10-20 N; the operator stays outside the arms' reach and counts from the video.
