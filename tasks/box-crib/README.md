# Build a box crib on a load point: three layers of crossed pairs, every corner overlapping

Six cribbing timbers lie on the table, three on each side of a red load-point mark. The robot builds a 2 x 2 box crib
on the mark, three layers high: two timbers per layer, each layer crossing the one below. The instruction says no
more than that. Rescue crews know the rule that makes a crib stable: every corner overlaps, each timber's end sticking
out past the timber it crosses by at least 4 inches. The natural-looking crib, with each pair laid at the very ends of
the pair below, meets flush at every corner and fails it. With these timbers that means keeping each pair at most
10.8 cm apart (centre to centre), and placing each one so its partner and the layer below stay put.

## The real step
Rescue crews and riggers build box cribs under loads they lift or shore, a crossed pair of timbers at a time. The
USACE Urban Search and Rescue [Shoring Operations Guide](https://firemarshal.ks.gov/DocumentCenter/View/190/US-Army-Corps-of-Engineers-Shoring-Operations-Guide---10-2021-PDF)
(Edition 5.0, Oct 2021), p. 2-48, on cribbing: "Overlap corners by at least 4 inches." It also says to keep the crib
"aligned and as square as possible" (p. 2-47) and limits a crib's height to three times its shortest width. Done by:
US&R structures specialists, heavy-rescue firefighters and riggers.

What goes wrong in real life: a crib whose timbers meet flush at the corners can roll or crush under its load. A photo
of the crib from above shows every corner.

What we simplified: half scale. A 4x4 x 20 in timber is 4.45 x 4.45 x 25.4 cm here, and the 4 in overlap becomes
50.8 mm; the timbers weigh 250 g because the arms are weak. Three layers, not the guide's full heights, and no load
is lowered onto the crib.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=box_crib.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Build a 2 x 2 box crib, three layers high, on the red load-point mark, using the six cribbing timbers." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/box-crib-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=box_crib.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Build a 2 x 2 box crib, three layers high, on the red load-point mark, using the six cribbing timbers." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/box-crib-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/box-crib-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the complete crib has stood still for 1 s;
otherwise it scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| load_point | fixed red disc, 2.4 cm across | (23, 0) |
| timber_1, timber_2, timber_3 | loose cribbing timbers, half-scale 4x4 x 20 in: 4.45 x 4.45 x 25.4 cm, 250 g, pine; lying along x | (25, 20), (25, 29), (25, 38) |
| timber_4, timber_5, timber_6 | the same | (25, -20), (25, -29), (25, -38) |

Every timber has friction 1.2 static, 1.0 dynamic. Start: both grippers open at the ready pose. One fixed layout:
`-E object_jitter=0`, so the seed doesn't change the start.

## Scoring
A timber is in a layer when it lies level (within 5 degrees), its centre within 16 cm of the load point, and its
lowest point within 4 mm of a whole number of timber widths above the table (the first layer on the table). A layer
is built right when it has exactly two timbers, parallel (within 5 degrees), square to the layer below (within 5
degrees) and each bearing on both of its timbers. Layers count from the bottom up to the first that isn't right; the
crib's centre (between the bottom pair) must be within 3 cm of the load point for any credit. A corner's overlap is
the shorter of the two crossing timbers' ends past the other's outer face.

| Score | When, at the end |
|---|---|
| 0.2 per layer | each layer built right, from the bottom (at most 0.6) |
| 1.0 | three layers, centred within 1.5 cm, every corner overlapping by at least 50.8 mm (4 in at full scale), every timber still, held for 1 s, which ends the trial (if time runs out first, this state scores 0.9) |

Three layers with the pairs at the ends of the pairs below (flush corners) score 0.6. A look at the crib from above
checks it.

## The scripted demo
Each pair is set 8 cm apart, centre to centre, so every end overhangs the timbers it crosses by 64.75 mm, 14 mm past
the rule. The arms take turns, one timber each turn, the left arm taking the timbers on its side and the right arm
those on its side; the one not placing waits over its own side. The fingers close across a timber's 4.45 cm width
(coming down 7 cm apart, so they fit between a pair), lift it, carry it 2 cm above the layer it's going into (so it
clears its partner already there), turn it square to the layer below, set it down 1 mm above its place and open.

## Runtime and seeds
- The scripted demo's plan takes 82 s of simulated time (1,644 steps at 20 Hz), within the 2,400-step limit.
- Official runs (2026-11-08): the demo 1.0, ending on success at 80 s, every timber within 6.3 mm of its aim; with every
  set-down aimed 8 mm off (towards +x and +y, then the other way), also 1.0, the timbers landing up to 17.6 mm from
  the demo's aims; Dream-1 0.0 over the full 2,400 steps.
- The scoring and reach checks run without the simulator: `python -m pytest -q test_box_crib.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
