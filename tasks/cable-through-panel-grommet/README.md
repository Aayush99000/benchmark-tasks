# Pass a cable through a panel grommet

A grey upright panel (280 x 100 x 6 mm) stands on the table's midline between the two arms, with a black square
grommet (26 x 26 mm bore, 10 mm above the table) low in its middle. A black 6 mm cable (40 cm) is fixed in a junction
box on the left-arm side and ends in a 60 x 16 x 12 mm plug; a 20 mm red band is taped on it 25 cm from the box.
The robot must push the plug through the grommet, take it on the far side with the other arm and pull the cable
through until the red band sits at the grommet. On the far side a 30 mm-tall duct wall runs parallel to the panel
13–14 cm behind it; the plug has to be carried over it. Neither arm can reach the other side of the panel (left arm
y > -2 cm, right arm y < +2 cm), so the plug has to be handed over through the hole: the left arm feeds, the right
arm draws the cable out.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Push the cable's plug through the black grommet in the grey upright panel, take it on the other side with the other arm, and pull the cable through until the red band on the cable sits at the grommet."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Closed-loop (reads the live cable state): the left arm picks the plug up by its cable-side end, turns it to point
through the grommet and pushes it in, then re-grips the cable 3 cm behind its last grip and pushes again until the
plug sticks ~10 cm out on the far side (4 pushes; the gripper fingers are ~7 cm wide along the cable, so they cannot
come closer than ~4 cm to the panel). The left arm lets go and backs away (the two wrists would collide above the
panel), the right arm grips the plug near its front end, lifts it over the duct wall and pulls straight away from
the panel in steps sized by the measured band distance until the band is at the grommet (success holds for 1 s and
ends the episode at step 554, ~28 s); task_score 1.0. `demo/` holds its videos, `setup.png` and `result.json`.

Cheat variants (scoring checks, each scores 0): `GROMMET_CHEAT=nothing` (no motion); `GROMMET_CHEAT=drop` (one arm
only: the left arm pushes the plug through the grommet, re-grips and pushes the cable 7 times, then lets go; the plug
stops against the duct wall, no hand-off); `GROMMET_CHEAT=around` (the plug is handed round the panel's near end
instead of through the grommet and pulled out on the far side).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

The cable's centre line (junction box → plug front) is intersected with the panel's mid-plane.

- **0** unless (a) it crosses the panel plane exactly once and that crossing is inside the grommet bore (so going
  round a panel end or over the top scores 0), (b) the whole plug is beyond the grommet's far face, and (c) the plug
  has been carried over the duct wall (centre ≥ 14.5 cm behind the panel's mid-plane), so poking or pushing the plug
  through from one side alone does not count.
- Then the band distance (along the cable, band centre to the panel, either side) gives **1.0** if ≤ 2 cm
  (success), else **0.3 + 0.5 × max(0, (15 cm − distance) / 13 cm)**.
- Real trials: 0 if the panel, grommet or junction box is moved, or the cable is pulled out of the box.
- `distance()`: band distance (m) when threaded, else 0.5 + plug-to-bore distance.

The red band is drawn by colouring one cable segment (a small spawner wrapper in `task.py`; the cable physics are
unchanged).

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). One trial takes ~4–6 min wall time on one GPU including the ~1–2 min
  Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight), so every trial
  starts identically.
