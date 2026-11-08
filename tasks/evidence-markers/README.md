# Mark every piece of evidence without moving any of it

Four pieces of evidence lie on a grey floor mat where they fell: two spent 9 mm shell casings, a phone and a closed
folding knife, one casing tucked 1.8 cm from a chair leg. Four yellow evidence-marker tents stand at the near edge.
The robot sets one marker beside each piece of evidence, 5 mm to 5 cm from it and not touching it, ready for the
scene photographs. The instruction doesn't say what every investigator knows: evidence is photographed where it lies,
so none of it may move. A 4 g casing tips if a finger brushes it.

## The real step
"Markers (e.g, business cards, chalk, spray paint or some other marker to place by noted evidence items)." ([NIJ,
Equipment Needed for Crime Scene Investigation](https://nij.ojp.gov/topics/articles/equipment-needed-crime-scene-investigation),
2009, from its Crime Scene Investigation guides.) Evidence is then photographed in place, with and without the
markers, before anything is collected. Done by: forensic science technicians and crime scene investigators.

What goes wrong in real life: a casing nudged or a phone turned while setting a marker no longer shows where it fell,
and the photos and sketch record the wrong scene; an item without a marker is missing from the record. A photo from
above shows both.

What we simplified: the casings stand on their bases (real ones usually lie on their side); the tents are solid
yellow blocks without numbers; the phone and knife are plain boxes; the floor is a fixed mat on the table.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=evidence_markers.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put one yellow evidence marker beside each piece of evidence on the grey floor mat, ready for the scene photographs." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/evidence-markers-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=evidence_markers.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put one yellow evidence marker beside each piece of evidence on the grey floor mat, ready for the scene photographs." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/evidence-markers-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/evidence-markers-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once every piece of evidence has had its marker for 1
s; otherwise it scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left; headings anticlockwise from +x seen from above.

| Object | Description | Position |
|---|---|---|
| floor mat | fixed, grey (concrete), 40 x 56 x 0.4 cm | centred at (32, 0) |
| casing_1, casing_2 | loose spent 9 mm brass shell casings, 0.99 cm across, 1.9 cm tall, 4 g, standing on their bases | (44, 8) and (27, -5) |
| phone | loose, black, 14.7 x 7.1 x 0.8 cm, 170 g | (40, -15), heading 20 degrees |
| knife | loose closed folding knife, grey, 11 x 2.8 x 1.4 cm, 120 g | (30, 20), heading -30 degrees |
| chair leg | fixed, dark wood, 3 cm across, 12 cm tall | (27, -1.2), 1.8 cm from casing_2 |
| marker_1 to marker_4 | loose yellow plastic evidence-marker tents, 5 cm wide, 3 cm deep, 6 cm tall, 20 g | a row at x 15: y 15, 7, -7, -15; 5 cm faces toward the arms |

Every loose object has friction 1.2 static, 1.0 dynamic. Start: both grippers open at the ready pose. One fixed
layout: `-E object_jitter=0`, so the seed doesn't change the start.

## Scoring
A marker counts for a piece of evidence when it stands upright (within 10 degrees) on the mat (its bottom within 3 mm
of it), is still (under 1 cm/s and 0.5 rad/s), and the gap between its outline and the item's, seen from above, is 5
mm to 5 cm: beside it, not touching. Each marker counts only for the item nearest it, so one marker can't cover two.

| Score | When, at the end |
|---|---|
| 0.25 each | a piece of evidence with a marker beside it |
| 1.0 | all four marked, held for 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0 overall | any piece of evidence more than 3 mm from where it lay, turned more than 3 degrees, knocked over, or still moving |

Markers set out anywhere else, however neatly, earn nothing. On a real floor, a ruler from each marker to its item
and from each item to its taped outline checks it; a photo from above shows it.

## The scripted demo
Each arm takes the two markers on its side, one at a time, gripped across their 5 cm width so the fingers stay on the
sides and never come between a marker and the evidence, and sets each down 2 cm from its item on the side toward the
arms, its 5 cm face toward the item: the left arm for casing_1 and the knife, the right arm for casing_2 (on the side
away from the chair leg) and the phone. Both arms work at the same time; each marker is let go 1 mm above the mat.

## Runtime and seeds
- The scripted demo's plan takes 24 s of simulated time (480 steps at 20 Hz), within the 2,400-step limit.
- Official runs (2026-11-08): the demo 1.0, ending on success at 23 s, every marker within 1.1 mm of its aim and no
  piece of evidence moved at all (0.0 mm, 0 degrees of tilt); Dream-1 0.0 over the full 2,400 steps (it never set a
  marker down beside anything).
- The scoring and reach checks run without the simulator: `python -m pytest -q test_evidence_markers.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
