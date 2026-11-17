# Pay by card: enter the PIN on a shop's PIN pad with the other hand cupped over the keys

A shop's countertop PIN pad lies on the table. The instruction gives a PIN, 7 3 1 9, to enter and confirm with the
green key, and asks for the keypad to be covered with the other hand so the person across the counter can't see which
keys are pressed. So one hand types on small keys while the other hand stands just beyond each key it presses, on the
onlooker's side, as a screen: the two hands work a few centimetres apart over the same keys, and the screen has to
move with the typing hand from key to key.

## The real step
Banks tell customers to hide the keys while they type their PIN. Bank of Ireland, [Card and ATM safety](https://www.bankofireland.com/security-zone/card-and-atm-safety):
"Always cover your card PIN when entering it on the keypad in a shop or supermarket". Commonwealth Bank and the UK's
Action Fraud give the same advice. Done by: nearly everyone who pays by card.

What goes wrong in real life: a PIN seen over the shoulder, or by a camera, is how a stolen card gets used.

What we simplified: the terminal is fixed to the table and its screen is not simulated. The onlooker is a fixed eye
point, and a key counts as seen if any corner of its top has a clear straight line to that point past both grippers'
collision shapes.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=shielded_pin.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Enter the PIN 7 3 1 9 on the card terminal and press the green key to confirm. Cover the keypad with your other hand while you type, so the person across the counter can't see which keys you press." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/shielded-pin-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=shielded_pin.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Enter the PIN 7 3 1 9 on the card terminal and press the green key to confirm. Cover the keypad with your other hand while you type, so the person across the counter can't see which keys you press." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/shielded-pin-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/shielded-pin-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the finished state has held for 1 s; otherwise it
scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| PIN pad | charcoal plastic case the size of an Ingenico Lane/3000, 18 x 8.3 cm, 2.4 cm tall, fixed; its screen at the far end | centre (29, 0) |
| keypad | 15 keys, 18 x 11 mm, 3 columns 22 mm apart and 5 rows 16 mm apart: 1 2 3 / 4 5 6 / 7 8 9 / * 0 # / red, yellow, green, the 1 2 3 row farthest from the customer and 1 on the customer's left; digits printed on the keys | rows at x 27.8, 26.2, 24.6, 23.0, 21.4; columns at y 2.2, 0, -2.2 |
| keys | each sprung, 2 mm proud of the case, 1.2 mm of travel; it registers when pushed 0.8 mm (about 2.4 N) and is ready again once back above 0.4 mm | key tops 2.6 cm above the table |
| onlooker | the person across the counter: an eye point 60 cm beyond the keypad's middle and 45 cm above the keys | (84.6, 0), 47.6 cm above the table |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
From the keypad's press log and the grippers' collision shapes.

| Score | When, at the end |
|---|---|
| 1.0 | the presses are exactly 7, 3, 1, 9, green, no two keys down at once, at each press every corner of the pressed key hidden from the onlooker's eye behind a gripper, and both grippers more than 3 cm from the terminal, held 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.5 | the same presses, but one or more of them was seen |
| 0.25 | the four digits, in order, not confirmed |
| 0 | anything else: a wrong key, an extra press, two keys at once, a digit missing |

A press is hidden when the straight line from the onlooker's eye to each of the four corners of the key's top (at
the depth it registers) passes through one of the grippers' convex collision hulls (palm and fingers, either arm), at
the moment it registers. The typing hand alone can't hide a key: it sits on the key, and from across the counter the
corners on its far side are in plain view.

## The keypad model
`PinPad` in `shielded_pin.py` runs before every physics substep (240 Hz). It reads each key's depth below its rest; a
key passing 0.8 mm is logged as a press (once, until it comes back above 0.4 mm), with whether another key was down at
the time and whether the key was hidden. The keys are rigid bodies on sprung PhysX prismatic joints in the terminal
(`fittingkit.Slider` with a drive: 3,000 N/m, so about 2.4 N at the click). The case leaves 1.5 mm round every key
through its whole travel: in the sim the keys collide with the fixed case despite their joints.

## The scripted demo
Open loop, both arms, on one shared timeline:
1. The right hand closes its fingers and turns them a quarter turn, so their thin side (4 mm either side of the
   middle at the tips) is along the keypad's rows; it leans 35 degrees back towards the arms.
2. The left hand closes its fingers, turns the same way and leans 15 degrees towards the onlooker; it stands with its
   fingertips 24 mm beyond the key to be pressed and 5 mm above the key tops. Its other side (9 mm either side of the
   middle at the tips, wider above) covers the key's 18 mm width from the onlooker.
3. For each key, the screen moves to it first (riding 1 cm higher), then the right hand comes over the key, presses
   it: it stops 6 mm above the key, settles 0.3 s, then pushes slowly for 0.6 s towards 7 mm below the key's top (the
   key's stop holds it at 1.2 mm; the arm gives a little, so it aims deep), holds 0.3 s, and lifts 10 mm back and
   25 mm up. It aims 5 mm beyond the key's middle, since in the sim the leaning hand lands short.
4. After the green key both hands move clear.

The poses were searched offline against the YAM's own gripper collision hulls (URDF forward kinematics,
`plan_check.py`): both hands flare to 10 cm wide 8 cm above their tips, so they lean apart to stay clear. Offline, every
press is hidden by the screening hand alone, the hands are never closer than 2.7 mm (at the end of each approach), the pressing fingertips touch
only their own key, both arms reach every pose within 0.1 mm, and no step exceeds 60% of the per-step joint limit.

## Part kit
`fittingkit.py` is copied from the accepted robocurve/benchmark-tasks task `tasks/compression-joint` (commit de817b9),
under the MIT licence its authors state in that task's README, with one addition noted in its docstring: `Slider`
takes an optional spring drive (`stiffness`, `damping`, `target`), used here for the keys. `plan_check.py` is the
offline check of the demo's poses described above.

## Runtime and seeds
- Official runs (2026-11-17, the Robocurve Brev launchable, 1,200 steps, `object_jitter=0`): the scripted demo scores
  1.0: the keypad logs 7, 3, 1, 9 and green at 5.2, 8.9, 12.6, 16.3 and 20.0 s, each hidden from the onlooker and
  with no other key down, and the trial ends at 22.5 s (450 steps) once the hands have been clear for 1 s. An earlier
  run of the same demo logged the same presses at the same times. Dream-1 scores 0.0 over the full 1,200 steps: its
  right hand comes over the keypad with its gripper open and hovers by the bottom rows, but presses no key, and its
  left hand never moves in to screen.
- The scripted demo's plan is 23.7 s of simulated time.
- The press log, the line-of-sight test, the scoring, the layout, reach and step-guard checks, and the demo against
  the real gripper hulls run without the simulator: `python -m pytest -q test_shielded_pin.py` (about 2 minutes).
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
