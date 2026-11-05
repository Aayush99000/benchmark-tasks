# Reseat the half-inserted plug

The rear panel of a 1U network switch mock-up (fixed, 200 mm wide, 44 mm tall) stands across the middle of the
table with four ports in its back face, 45 mm apart, open at table level. A patch cord (6 mm round cord, 25 cm,
with a 55 x 16 x 14 mm rigid plug body) is plugged into each port; the cords run straight back towards the robot
and lie loose on the table. Three plugs are fully seated (tip 25 mm inside the port); one is only half inserted
(tip 8 mm inside), so it sticks out 17 mm further than its neighbours. The robot must find the loose plug by eye,
push it fully home and let go, without pulling or unseating any of the other three plugs, even briefly (in a live
rack that would drop a working link). Pushing an already seated plug is harmless (it is already against the bottom
of its port), so pressing all four plugs home in turn is a valid solution.

Instruction: `One of the four patch cords plugged into the back of the switch is not fully seated. Find the loose plug and push it fully into its port without pulling or unseating any of the other plugs, then let go.`

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` (the simulated cables) is in this folder next to `task.py`; `cabledemo.py` runs the scripted demo.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "One of the four patch cords plugged into the back of the switch is not fully seated. Find the loose plug and push it fully into its port without pulling or unseating any of the other plugs, then let go."
```

Run one trial per process (or one per embodiment reset): the scorer keeps a short history of the trial (see
Scoring), which `task.py` clears on every embodiment reset.

Variants: the environment variable `RESEAT_LOOSE=A|B|C|D` (ports from left to right as seen by the robot) chooses
which port holds the half-inserted plug; the default (unset) is `B`, the yellow cord, left of centre. A and B are
in front of the left arm, C and D in front of the right arm.

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

The script inspects the panel (plug depths from the live object states), picks the plug that sticks out, and the
arm on that side comes down on the exposed plug body with its fingers opened only ~4 cm (so they clear the
neighbouring plugs 45 mm away), pinches it, slides it straight into the port until it bottoms out, opens and lifts
away. It reaches `task_score` 1.0 in 156 steps. Videos are in `demo/`.

Shortcut variants (`RESEAT_DEMO=...`), each scoring exactly 0: `nothing` (do nothing), `pull` (tug a seated
neighbour out 12 mm "to test it", push it back, then reseat the loose plug correctly), `wrong` (push home a seated
neighbour instead of the loose plug), `hold` (push the loose plug home but keep holding it). `all` (press all four
plugs home in turn, no inspection) is a legitimate solution and scores 1.0.

## Scoring (`score()`, partial credit; `success()` = score 1.0 with the plug at rest, held 1 s)

Depth = how far a plug's tip is inside its port (measured along the port from the panel's back face; a plug that is
not in line with its port opening counts as out).

- **Unseated neighbour (latches, score 0):** any of the three originally seated plugs is less than 22 mm deep at
  any moment of the trial.
- **Not released (score 0):** at the end a gripper is still holding or touching a plug (its TCP within 45 mm of any
  plug centre).
- Otherwise, by the loose plug's depth at the end: at least 22 mm → **1.0**; 15–22 mm → **0.5**; less than 15 mm →
  **0**.

Real-world measurement: a tape mark on each plug body 22 mm and 15 mm from its tip, watched in the top/side camera
video (a mark disappearing into the port face = that depth reached); a seated plug whose 22 mm mark appears outside
the port face at any time is unseated. Final check by eye with the grippers open and away from the plugs.

## Runtime and seeds

Run limit 1,200 steps (60 s at 20 Hz); the demo needs 156. A trial takes ~3–5 min wall-clock on one GPU
(first start of Isaac Sim adds 1–2 min). The layout is fixed (`object_jitter=0`; the panel is fixed and the cords are
never jittered), so every seed starts from the same state; use `RESEAT_LOOSE` for the four layouts.
