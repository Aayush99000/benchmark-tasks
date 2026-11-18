# Roll a knife and fork up in a cloth napkin

A wooden side table stands on the station table in front of the two arms, its top 20 cm up. On it lie a white cloth
napkin, flat, with its near edge at the side table's near edge; a table knife to the left of the napkin; and a dinner
fork to its right.

The robot has to end with the knife and the fork rolled up together inside the napkin, lying on the side table. The
napkin is true cloth: it is limp, it lies flat, and a gripper cannot get under it there. It can only be taken by an
edge that hangs free, and the knife and fork are loose inside it until the roll is done.

The task file is [roll_silverware_in_a_napkin.py](roll_silverware_in_a_napkin.py), for the `yam_arena` embodiment
([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station
scene. [labkit.py](labkit.py) must stay next to it.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/roll-silverware-in-a-napkin/roll_silverware_in_a_napkin.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Roll the knife and fork up in the white napkin." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/roll-silverware-in-a-napkin-dream
```

Scripted demo: the same command with `--policy yam_scripted`.

The run prints `task_score` at the end. `ROLL_SILVERWARE_LOG=1` prints the measurements behind the score once a
second: how many times the napkin goes round the knife, how wide it is, where the fork lies against the knife, and how
long both grippers have been clear of the roll.

## Scoring

Scored on the end state, from the napkin's mesh nodes and the knife's and fork's poses. The score is 0 unless all of
these hold:

- the knife, the fork and the napkin are on the side table;
- the fork lies along the knife: its centre within 3 cm of the knife's centre line, and within 20 degrees of it;
- the napkin goes all the way round both: napkin on every side at 75% or more of 9 stations over the middle 80% of
  each piece;
- the napkin goes at least 0.9 times round the knife.

| Outcome | Score |
|---|---|
| Nothing done; pieces laid on the open napkin; napkin dropped over the pieces | 0 |
| Fork outside the roll, or lying across the knife | 0 |
| Anything off the side table | 0 |
| Folded over: napkin 0.9 to under 1.15 times round the knife | 0.3 |
| Rolled on: 1.15 times round or more | 0.6 |
| Rolled up: 1.6 times round or more, and the napkin at most 11 cm wide across the knife, but still held, pressed on or lifted | 0.8 |
| Rolled up and let go: the roll lies on the side table (knife and fork centres at most 4 cm above its top) and every finger pad of both grippers has been at least 2 cm from the napkin, the knife and the fork for the last 2 s | 1.0 |

The trial ends with 1.0 once the roll scores 1.0 and the knife and fork have lain still for 2 s.

On a real table: a tape measure across the roll gives its width, and counting the layers at the handle end gives how
many times the napkin goes round.

## The scripted demo

Five steps, in `demo()`:

1. Pull the napkin toward the robot until a quarter of it hangs over the side table's near edge.
2. Lay the knife on the napkin along the edge, and the fork on the knife.
3. Take the hanging edge with both grippers and fold it up over the knife and fork.
4. Turn the bundle a quarter turn, so the handle ends stick out over the edge.
5. Roll it two half turns by the handle ends, taking a fresh hold for each while the other gripper holds the
   bundle down.

The demo is open loop and the napkin is simulated cloth, which does not behave identically from run to run.

## Runtime and seeds

- Demo: 104 s simulated (2,088 steps to full score): the trial ends once the roll has been let go and has lain still for 2 s. About 14 minutes of wall
  time on an L4 GPU, including Isaac Sim's start.
- Dream-1: up to 3,000 steps (2 minutes 30 seconds simulated); about 10 minutes of wall time.
- One fixed layout: `-E object_jitter=0`, so no seed is needed.

## Fidelity

- **Napkin:** a PhysX deformable surface of 1,435 nodes, not rigid pieces. One sheet 23.5 x 28 cm, 2.5 mm thick,
  68 g: the size, thickness and weight of a 47 x 56 cm dinner napkin folded in quarters. It collides with itself. It
  is limp, a little springier than cotton, and does not cling to itself.
- **Knife and fork:** rigid, real size (knife 24 cm, 85 g; fork 19.5 cm, 45 g, arched like a real fork), each built
  from several convex collision pieces.
- **Side table:** fixed; its top is smooth (friction 0.2), so a napkin slides on it readily.
- **Grip:** the gripper pads hold cloth by friction alone. Nothing is attached by rule at any point.

## Checks without the simulator

```bash
python -m pytest -q tasks/roll-silverware-in-a-napkin/test_roll_silverware_in_a_napkin.py
```

They check that the layout matches the setup text, score a made-up end state for every row of the table above
(including a finished roll that is still held, pressed on, or lifted off the table), and check the demo's five steps. They need `yam_arena` but not Isaac Sim.

## Files

- `roll_silverware_in_a_napkin.py`: the task file (layout, scoring, scripted demo).
- `labkit.py`: object kinds that `yam_arena` task files don't have (true cloth, model files), and the finger pad
  positions the scorer uses to check that the grippers have let go.
- `models/`: the knife and fork (`.obj`, `.usd`) and `make_cutlery.py`, which generates them.
- `test_roll_silverware_in_a_napkin.py`: offline checks.

## Third-party assets

None. The knife and fork are our own models, generated in code by `models/make_cutlery.py`. The scene uses the
program's YAM station replica as installed.
