# Press a power cord into a curved cable channel

A power cord (8 mm black PVC, 50 cm, no plug) leaves a small device box at the left rear of the table and lies
straight across the table at 50 degrees towards the right arm. In front of the box is a white J-shaped open-top
cable channel (16 mm wide, 12 mm deep, 30.6 cm long: 8 cm straight away from the robot, a quarter turn of 8 cm
radius to the right, 10 cm straight to the right). The robot must press the cord into the channel along its whole
length, starting at the box end. Because the channel bends, the cord cannot simply be pulled taut over it (a taut
cord cuts across the bend and rests on the wall tops); it has to be laid in progressively from the fixed end,
with the right length of cord fed to each point. The natural solution is bimanual: one arm keeps the free end of
the cord up and out of the way while the other lays and presses the cord in, a few centimetres at a time.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Press the power cord into the white channel along its whole length, starting at the box."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Closed-loop (reads the live cord state). The right arm takes the cord 3 cm from its free end and holds it up
beyond the channel exit, keeping the loose cord off the channel. The left arm then works from the box end: it
grasps the cord at the arclength that belongs at the next point (measured from the last seated point; shifted a
little along the cord when the open fingers would land on a channel wall), carries it over the slot (2.5–3 cm
ahead, less round the bend), lowers it onto the slot and lets go, then presses it down with the closed fingertips
there and every ≤ 2 cm back to the previous point; a final touch-up presses any check point still unseated.
Success (all 8 sections seated for 1 s) ends the episode at step 963 (~48 s); task_score 1.0. `demo/` holds its
videos, `setup.png` and `result.json`.

Cheats (each must score exactly 0): `CHANNEL_CHEAT=nothing` (no motion), `CHANNEL_CHEAT=taut` (right arm pulls the
cord taut over the channel towards the exit and lays it down, never pressing it in), `CHANNEL_CHEAT=drag` (right
arm drags the free end low through the channel region round to the exit and lets go), e.g.
`CHANNEL_CHEAT=taut python cabledemo.py task.py out --no-record`. Results: all three end with task_score 0.0 (nothing: 100 steps, taut: 212, drag: 238; the drag briefly had one section in the channel mid-drag, but the cord does not stay there).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- The channel centreline (30.6 cm) is split into 8 equal sections, each checked at 8 evenly spaced points.
- A point is seated if some part of the cord is within 8 mm (half the inner width) of the centreline there, i.e.
  between the walls, and the cord's centre is at most 10 mm above the table (its top no more than 2 mm above
  the 12 mm lip). A section is seated if all 8 of its points are.
- Score = seated sections / 8. Success (1.0) = all 8 seated, held for 1 s.
- A cord resting on the wall tops (centre 16 mm up), lying beside the channel, or crossing it scores 0 for those
  sections. Doing nothing scores 0 (the starting cord passes 2 cm in front of the entrance and never enters it).
- `distance()`: channel length (m) not yet seated.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). One trial takes ~5–25 min wall time on one busy GPU including the
  ~1–2 min Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight), so every trial
  starts identically.
