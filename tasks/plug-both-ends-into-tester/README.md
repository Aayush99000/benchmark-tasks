# Plug both ends of a patch cord into a cable tester

A two-piece patch-cord tester stands on the table: a blue main unit on the left and an orange remote unit on the
right, each with a test jack (a 20 x 16.5 mm opening, 26 mm deep, at table level) at the tip of a short sleeve. The two
jacks face each other across a 41 cm gap. A loose 40 cm patch cord with an identical rigid plug on each end lies
straight across the table in front of the tester. The robot must seat one plug in each jack and let go. The cord is
only about 5 cm longer than the straight line between the two seated plugs: carrying one end to its jack drags the
other end along, and a plug that has been pushed in can be pulled back out by the cord. So the two arms have to
bring both ends over together and seat the second plug without pulling the first one out. Either end may go into
either unit (a patch cord is symmetric). Each arm can only reach its own side of the table, so both arms are needed.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` (the cable simulation) and `cabledemo.py` (the scripted-demo runner) are in this folder. The cord is a
  free (unanchored) kit cable with a plug on each end; no task-local changes to the kit.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Connect the loose patch cord to the cable tester for certification: push one plug fully into the jack of the main unit (blue, left) and the other plug fully into the jack of the remote unit (orange, right), without pulling the first plug out, then let go of both plugs."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

The demo is closed loop: it reads the live plug poses. Both arms pinch their plug 10 mm from its cord end, lift
it 2.5 cm and carry the two ends forward together, so neither arm drags the other end. Each plug is lowered in
front of its jack and lined up with the jack axis using the measured plug position (the arm sags a few mm under
load). The left arm seats its plug in the main unit (about 25 mm deep) and keeps holding it while the right arm
seats the other plug in the remote unit. Then both release and lift away. task_score 1.0; success holds for 1 s and
ends the episode at step 316 (~16 s). `demo/` holds the videos, `setup.png` and `result.json`.

Other behaviours, selected with the test-only env var `BOTHENDS_DEMO=<mode>`:

| `BOTHENDS_DEMO=` | what it does | result |
|---|---|---|
| `nothing` | waits | 0 |
| `near` | carries both plugs to the jacks and puts them down without inserting: one lined up with its jack, tip 4 mm short of the opening; the other pressed against the jack face 2 cm beside the opening | 0 |
| `hold` | seats both plugs but keeps holding them | 0 |
| `one` | seats only the main-unit plug and drops the other end (intended partial credit) | 0.5 |

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if, at the end, either gripper's TCP is within 45 mm of either plug centre (still holding or touching a
  plug). In a real trial, a tester unit that has moved also scores 0. In sim the units are fixed.
- Otherwise **0.5 per jack** (main, remote) that holds a plug whose front end is at least 15 mm inside the jack
  opening. The plug counts only if it lies in the jack: centre within 8 mm of the jack axis sideways, not above the
  opening, and pointing into the jack. A plug on top of, beside or in front of a jack scores nothing for that jack.
  Either plug may fill either jack. 1.0 when both jacks hold a plug.
- `distance()`: for each jack, how far (m) the best plug still is from 15 mm deep (0.1 if no plug is lined up with
  it), summed.

## Runtime and seeds

- Run limit 1,200 steps (60 s at 20 Hz). One trial takes ~3–5 min wall time on one GPU, including the ~1–2 min Isaac
  Sim start-up.
- The layout is fixed with `object_jitter=0`. The cord is never jittered (it always starts straight).
