# Place ECG leads on the correct electrodes

A patient manikin lies on its back on the table with its **head away from the robot**, so the robot stands at the
patient's feet. Four identical, unlabelled white electrode cups sit on its torso at the limb-lead positions: right
arm (RA), left arm (LA), right leg (RL) and left leg (LL). Three ECG lead wires (white, black, red) leave a patient
monitor on the robot's left and lie straight across the table, their clips all on the robot's right. The robot must
drop each clip into the cup of its electrode under the AHA 3-lead code: **white = RA, black = LA, red = LL**; RL stays
empty. The instruction names the convention but not the cups: the point is the reasoning step that, for a patient
lying face up with the head away, **the patient's right side is on the robot's left (+y)**. So RA is the far cup on
the robot's left, LA the far cup on the robot's right, LL the near cup on the robot's right (RL: near, robot's left).
Applying "right" in the robot's own frame (the mirror error) puts every clip in a wrong cup. Since all clips start on
the robot's right, the white clip has to cross to the left arm: the demo hands it over by setting it down on the
torso midline.

Geometry check: for a person lying supine, anterior = up (+z) and head = +x (away from the robot); a person's right
= anterior x head-direction = z x x = +y, i.e. the robot's left.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1400 \
    --instruction "Attach the 3-lead ECG cable to the patient manikin, which lies on its back with its head away from you. Following the AHA colour code, drop each lead's clip into the white electrode cup at the correct body position for that lead; one cup stays empty. Then let go of the clips."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1400
```

Closed loop (reads the live clip poses): the right arm picks the white clip and sets it on the torso midline; the left
arm picks it there and drops it into RA while the right arm drops black into LA and red into LL; both arms back off.
task_score 1.0 at ~540 steps (~27 s). `demo/` holds its videos, `setup.png` and `result.json`.

Shortcut variants (`ECG_DEMO=<mode> python cabledemo.py task.py out`), all score exactly 0:
- `nothing`: no motion.
- `mirror`: patient's right taken as the robot's right (white LA, black RA, red RL; in sim the red clip ends
  perched on the black lead beside RL — white and black in wrong cups already zero the score).
- `swap`: white and black exchanged (white LA, black RA, red LL correct): one wrong clip zeroes the score.
- `hold`: correct placement, but the left arm keeps holding the white clip in its cup.

## Scoring (`score()`, at the end; `success()` must hold 1 s)

- A clip lies in a cup when its centre is inside the cup's 40 mm inner square (top view) and at most 12 mm above the
  cup floor.
- 0 if a gripper TCP is within 3 cm of any clip, or any clip lies in a cup other than its own (also the unused RL cup).
- Otherwise 1/3 per clip in its own cup (white RA, black LA, red LL).
- `distance()`: clips not in their own cup, +3 if any is in a wrong cup, +1 if a gripper still holds a clip.

## Runtime and seeds

- Run limit 1,400 steps (70 s at 20 Hz). One trial takes ~3-5 min wall time including Isaac Sim start-up.
- Layout fixed with `object_jitter=0`; the leads are never jittered (they always start straight). The manikin is not
  rotated between trials.
