# Pull a cable out to a length mark

A grey closed feed channel (400 mm long, 14 x 12 mm inside) lies on the table in front of the left arm, running
left–right at x 30 cm with its open exit at y +10 cm facing the right arm. A black 6 mm cable (45 cm, not fixed) lies
inside it with only its end sticking 6 cm out. A yellow ruler (400 x 30 mm) is taped beside the cable line, zero at
the exit face, with black ticks every 5 cm and a red line at 30 cm (y −20 cm, continued by a red mark on the near side
of the cable). The robot must pull the cable out along the ruler until its end lies at the red 30 cm line (±1 cm),
then let go, leaving the pulled-out cable lying straight beside the ruler, and never pull the end more than 5 cm past the line. The
red line is beyond the left arm's reach (left arm y > −2 cm) and the cable end starts beyond the right arm's reach
(right arm y < +2 cm), so the left arm draws the first stretch out, hands over, and the right arm pulls the end to the
mark and sets the last centimetres by watching the ruler. This is cut-to-length measuring: precise length dispensing
with visual feedback, where over-pulling fails.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Pull the black cable out of the grey feed channel along the yellow ruler until the cable's end lies on the table at the red 30 cm line, then let go, leaving the pulled-out cable lying straight beside the ruler. Never pull the end more than 5 cm past the red line."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed-loop (reads the live cable state): the left arm grips the cable 1.5 cm behind its end, draws it out along
the cable line to the midline (reading ≈ 11 cm), lays it down, lets go and backs away. The right arm grips the end,
then repeats: move the end towards the mark by the measured remaining length (at most 12 cm per stroke), lay it on the
table, read the ruler — until the end is within 4 mm of 30 cm; then it lets go, lifts away and re-reads (re-gripping and
correcting if letting go moved the end by more than 6 mm). Success holds for 1 s and ends the episode at step 315
(~16 s); task_score 1.0. `demo/` holds its videos, `setup.png` and `result.json`.

Cheat variants (scoring checks, `LENGTH_CHEAT=<name>`, each scores exactly 0):
- `nothing`: no motion (reading 6 cm).
- `yank`: after the hand-over the right arm yanks the end to 42 cm, then brings it back and lays it exactly at
  30.0 cm along the ruler — 0 because the end passed the 35 cm line.
- `hold`: the right arm pulls the end exactly to the line (30.0 cm) and keeps holding it down there — 0 because the
  cable is not let go.
- `sideways`: the right arm pulls the end to the 30 cm reading but ~7 cm off the cable line (dragged off the ruler) —
  0 because the pulled-out cable is out of line.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

Reading = horizontal distance along the ruler from the exit face to the cable's end tip.

- **0** if the cable is still held (a gripper within 3 cm of any point of it); or the end was ever pulled past the 35 cm line during the episode (tracked every step); or any point of the
  pulled-out cable is more than 2 cm sideways from the cable line or more than 1.5 cm above the table; or less
  than 5 cm of cable is still inside the channel.
- Otherwise |reading − 30 cm| ≤ 1 cm → **1.0** (success); ≤ 3 cm → **0.6**; ≤ 5 cm → **0.3**; else **0**.
- Real trials: 0 if the channel or ruler is moved.
- `distance()`: |reading − 30 cm| (m); 1 once over-pulled.

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz); the demo needs 315. One trial takes ~3–5 min wall time on one GPU
  including the ~1–2 min Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight), so every trial
  starts identically. The over-pull memory resets at every episode reset.
