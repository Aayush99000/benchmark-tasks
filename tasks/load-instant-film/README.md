# Load a film pack into an instant camera

An instax mini 12 instant camera stands upright on the table, empty, its back towards the right arm and its lens
towards the left. A new film pack lies flat beside it. The instruction asks for the pack to be loaded and the camera
made ready to shoot. That is the camera's own loading sequence, each step a different kind of contact:
1. Press the back cover lock down: the back cover springs ajar.
2. Swing the cover open round its hinge.
3. Pick the pack up by its long sides, turn it so its yellow mark is at the camera's yellow mark (yellow end up, the
   back with the two holes facing out), and push it fully home, never touching the two holes.
4. Swing the cover shut and press it until it locks.
5. Grip the lens ring and turn it to ON.
6. Press the shutter button once: the black film cover comes out of the slot on top.

A pack in upside down or back to front, a cover that isn't locked, or a press on the pack's holes spoils the film; a
shutter press on a camera that isn't loaded right ejects nothing.

## The real step
Fujifilm's instax mini 12 User's Guide (FI020-EN-WW-01, pages 3-4, drawn steps; part names on page 6: "Back cover
lock", "Back cover", "Lens ring / Power", "Shutter button", "Film ejection slot"):
1. Slide the back cover lock down to open the back cover.
2. Insert the film pack, aligning the yellow marks.
3. Close the back cover.
4. Turn the camera on by rotating the lens ring to ON.
5. Press the shutter button: the film cover is ejected.

Fujifilm's instax printer manuals add, for the same mini film pack: "When you load the film pack, never press the two
rectangular holes on the back of the film pack."

Done by: anyone who owns an instant camera, at a party, a wedding or at home; the same step comes with every new
pack (10 shots). What goes wrong: fogged or jammed film, a wasted pack. A photo of the camera shows whether the film
cover came out.

## The camera, the pack, and what was simplified
| Size | Value used | Source |
|---|---|---|
| camera body | 104 x 66.6 x 122 mm (W x D x H) | Fujifilm's instax mini 12 specifications page (306 g without batteries, strap and film) |
| film pack | 92 x 61 x 20 mm, 45 g | size: Wikipedia's instax article (mini film pack); mass: **estimate** |
| film chamber | 67 x 98 mm, 20.5 mm deep (3 mm play round the pack), floor 11 mm up; a finger notch 10 mm deep and 72 mm tall in each side | **estimate**; the notches are a simplification (see below) |
| back cover | 3 mm plate, 30 g, hinged on its far edge, a stop at 95 degrees, springing 20 degrees ajar when unlocked | **estimate** from the guide's drawings (which warn against opening it further) |
| back cover lock | 12 x 12 x 7 mm tab, 3 mm travel, unlocks at 2 mm | **estimate** |
| lens ring | 62 mm across, 22 mm deep; ON at 30 degrees or more | **estimate** |
| shutter button | 18 mm across, 4 mm proud, 2.5 mm travel | **estimate** |
| pack's holes | two 20 x 10 mm holes, either side of the back's middle | **estimate** |

How the sim models it:
- The camera is a fixed articulation, built with `mechanism.py` (MIT, from robocurve#223; changes listed at its top):
  - fixed boxes and cylinders for the body, the rear frame round the chamber, the chamber's floor and the marks;
  - the back cover on a revolute joint with a spring whose target the task file sets every control step: locked, the
    target is 0 (held shut, 20 N m/rad); once the lock tab is 2 mm down, the target is 20 degrees (it springs ajar),
    then follows the cover wherever it is put; pushed back within 2 degrees with the tab up, it relocks;
  - the lock tab and the shutter button on sprung prismatic joints; the lens ring on a revolute joint with friction;
  - the film cover, a thin black plate inside the body on a prismatic joint, driven 45 mm up out of the slot by a
    shutter press made while the camera is ready.
- The pack is one loose rigid box at its true size; its holes and yellow mark are drawn on its back, not cut.

What we simplified:
- The camera is fixed to the table; a person holds it in one hand. Real trials: fix it with double-sided tape.
- The film cover is ejected by a joint, not a motor and rollers; nothing is exposed or printed.
- The pack is a plain box: the real pack's shape keys it, so in a real camera a wrong-way pack sits proud and the cover
  won't shut; here it fits and is scored 0.
- The chamber has 3 mm of play and a finger notch in each side, so the gripper's two fingers (offset 18-30 mm either
  side of its pinch line) can carry the pack in by its long sides; a person's fingers fit past the pack's ends.
- The sizes marked estimate above, until a camera is measured; the batteries are left out.

## Setup
Positions in cm from the midpoint between the arm bases: x forward, y to the robot's left, heights above the table.

| Object | Description | Position |
|---|---|---|
| camera | instax mini 12, mint green, upright; fixed | centre (38, 0), back towards -y (right arm), lens towards +y (left arm) |
| back cover | mint, 3 mm, hinged along its far edge (x 43.2) | covers the back; opens towards -y |
| film chamber | dark floor, yellow mark at its top far corner, a finger notch in each side | x 34.65-41.35, 1.1-10.9 cm up, behind the cover |
| back cover lock | grey tab | x 34.0-35.2, y -4.5 to -3.3, 11.2-11.9 cm up |
| lens ring | silver, black front, index mark on top | centre (38.6, y 3.3-5.9), 4.8 cm up |
| shutter button | silver | (34.6, y 3.3), 9.2 cm up |
| film pack | black; two grey holes and a yellow corner mark on its back | flat, back up, centre (24, -18), length along y, yellow end away from the arms |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`. For real trials: the camera
empty, cover locked, ring at OFF, a fresh pack placed as in the setup photo.

## Scoring
- **Seated:** the pack in the chamber with its yellow end up and its back out (each axis within 10 degrees), standing
  on the chamber's floor and centred across it (each within 3.5 mm), its back within 1.5 mm of flush with the mouth.
- **Locked:** the cover pushed shut within 2 degrees with the lock tab up.
- **ON:** the lens ring turned 30 degrees or more.
- **Hands clear:** no point along either gripper's pads, and neither fingertip's end, within 2 cm of the camera.

| Score | When |
|---|---|
| 1.0 | the film cover ejected by a shutter press made with the pack seated, the cover locked and the ring at ON; still so, hands clear, the pack still; held for 1 s, which ends the trial |
| 0.75 | seated, locked and ON, but not yet ejected (or a hand near the camera) |
| 0.5 | seated and locked |
| 0.25 | seated |
| 0 | anything else, including a pack in the chamber the wrong way round, a fingertip ever within 2 mm of either hole on the pack's back (and up to 3 mm above it), and the cover ever pulled 5 degrees open while locked |

In a real trial, the film cover in the slot and the camera's film counter (S, then 10) show the result; a ruler gives
the pack's seating.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder. `mechanism.py` must sit beside the task file, which
imports it:

```bash
INSTR="$(python3 -c 'import json; print(json.load(open("task.json"))["instruction"])')"
COMMON=(--embodiment yam_arena -E task=load_instant_film.py -E object_jitter=0 -E control_hz=20 -E cam_width=320
        -E cam_height=176 --instruction "$INSTR" --scorer task_score --max-steps 2400 --store-frames)

# Dream-1
inspect-robots run "${COMMON[@]}" --policy dream --log-dir ~/yam-eval/load-instant-film-dream

# The scripted demo, which reaches full score
inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/load-instant-film-demo

# Wrong ways (probes of the scoring rules)
FILM_PACK_TURNED=1 inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/lif-turned   # yellow end down
FILM_DEMO=noring inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/lif-noring     # ring left OFF

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/load-instant-film-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. With `FILM_DEBUG=<file>` set, each control step adds a line to that file: the
pack's place and turn, whether it is in the chamber and seated, the cover's angle, the lock's travel, the ring's angle,
the shutter's travel, the film cover's height, the lock state, presses, ejection, the holes rule, and where the pad
tips are.

## The scripted demo
It plays a fixed plan from the fixed layout with the arm's own inverse kinematics, and reads nothing from the
simulator.

1. The right hand, pointing down, presses the lock tab 3 mm; the cover springs 20 degrees ajar.
2. Turned so its two closed fingertips lie along the cover, it comes down into the gap behind it and pushes it round
   its hinge, in 5-degree steps, to its stop at 95 degrees.
3. It pinches the pack across its width, holding 16 mm of its 20 mm sides, lifts it and turns it a quarter turn about
   the line between its fingers, to point at the chamber: yellow end up, back out.
4. It carries the pack straight in, its fingers in the side notches, 2 mm past flush, opens to 72 mm (inside the
   notches) and backs out. The pack stands home on the chamber's floor.
5. It goes round the open cover's free edge, low, and swings the cover shut from outside to about 45 degrees, then
   turns to point at it and pushes it straight shut until it locks, and parks.
6. The left hand, pointing at the lens, grips the ring and turns it (to its 80-degree stop), lets go, then presses
   the shutter button 2.2 mm and parks. The film cover comes out of the top.

## Runtime and seeds
- **Official runs (2026-11-23, this version):** the scripted demo scored 1.0, ending on success at step 991 (50 s):
  pack seated the right way (back 0.0 mm from flush, centred), cover locked, ring at its 80-degree stop, one shutter
  press, film cover 44.7 mm out, hands clear.
  - Seven earlier versions failed, and each failure changed the plan or the prop: the lock press sent below the table (a
    missing table height), the pack meeting the half-open cover (the cover now has its stop), the pack slipping when held
    by 6-8 mm and turned about two axes (now held 16 mm and turned about one), the gripper's fingers stopping at the
    chamber's mouth (the side notches), a pack 2.6 degrees off upright stopping at 1 mm play (now 3 mm), a separate push
    landing on the holes (now pushed home while held), and the cover reopened while being shut (now pushed the last part
    straight on).
  - Probes (the same demo, one thing wrong; stopped once both arms had parked, at step 1,236 and 1,232):
    `FILM_PACK_TURNED=1` loaded the pack yellow end down, then locked the cover, turned the ring and pressed the shutter:
    grade 0, nothing ejected. `FILM_DEMO=noring` left the ring OFF: grade 0.5 (seated and locked), the shutter press
    ejected nothing.
  - Dream-1 (2026-11-23, the task's instruction, 320 x 176 cameras, 2,400 steps): 0. It moved both arms for the full
    run without touching the lock, the cover or the pack.
- **Offline checks:** the scoring, lock logic and reach checks run without the simulator:
  `python -m pytest -q test_load_instant_film.py` (needs `yam_arena` importable).
- **Seeds:** one fixed layout (`-E object_jitter=0`), so the seed doesn't change the start.
