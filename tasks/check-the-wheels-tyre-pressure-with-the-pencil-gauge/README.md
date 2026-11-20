# Check a car wheel's tyre pressure with a pencil gauge at its angled valve, then put the gauge back

A 14 inch steel car wheel with its tyre lies flat on a workshop bench. Its rubber valve stem leans in toward the hub,
as a snap-in stem in a steel rim's well does. A straight-headed pencil tyre gauge lies in a cradle beside it. The
robot is asked to check the tyre's pressure with the gauge and put the gauge back. Doing that means taking the gauge,
turning it onto the stem's 30-degree lean, pressing its chuck on square and home so the chuck's pin opens the valve
core while its gasket seals on the stem's tip, taking it off, and laying it back in the cradle. The instruction names
only the goal. The result is read the way a mechanic reads it: off the gauge's stick.

## Credit

This task takes up the idea of PR 1896, "Read a wheel's tyre pressure with a pencil gauge, then put the gauge back in
its holder", submitted as TK-KUQS9V and revised as TK-UE73FA. Both versions were not accepted, and the hold on the
idea lapsed with no further revision. The first review asked for a working Schrader valve and a gauge whose chuck
depresses the valve core and seals at the valve, with success from an actual pressure reading rather than an
artificial collar. The second review asked for the valve's angle to be true to a real wheel, pointing toward the
wheel's centre. This is a new implementation written from scratch; no files from that PR are used.

How each point is met here:
- **A working Schrader valve, core depressed by the chuck.** The stem's brass barrel has an open 5.2 mm bore with a
  sprung core pin in it (a slider: 1.5 N to start it moving, 0.6 mm to open, 3 mm of travel), its tip 0.8 mm below
  the barrel's tip. The gauge's chuck has an 8.8 mm socket that fits over the 7.7 mm barrel, a rubber gasket at the
  socket's floor and a 2 mm pin standing 2.0 mm proud of it. Only when the gasket is home on the barrel's tip is the
  core pushed in 1.2 mm, past its opening point. All of this is contact between rigid shapes; nothing is switched by
  a script.
- **Sealing at the valve, not on a collar.** There is no collar. "Sealed" is the gasket face on the barrel's tip
  (within 0.6 mm along the stem, 1.2 mm off its axis, 4 degrees). Open but not seated is a leak.
- **Success from an actual reading.** The gauge has a reading stick (a slider) that only air moves out. The score
  reads the stick's position against its psi marks at the end, with the gauge let go: no flag says "a seal happened".
  A short press, a crooked press or a press that does not open the core gives a low reading or none.
- **The valve's angle.** The stem leans 30 degrees from the wheel's axis toward the hub, out of the inside of the
  rim's barrel, as a TR413 snap-in stem does in a steel wheel's well. The gauge must be turned onto that line: a
  straight-down press meets the barrel 30 degrees askew and cannot seat.
- **Real setting, true size.** A real car wheel and tyre size (587 mm, 165 mm tall) lying on a bench, as in a tyre
  bay. The stem's tip is 136 mm up, and the hand's grip on the gauge while pressing is 171 mm up, under 0.20 m.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable (the program's YAM embodiment for Isaac
Lab-Arena):

```bash
I="Check the tyre pressure of the wheel with the pencil gauge, then put the gauge back in its cradle."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir logs/dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir logs/demo
```

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz).
- **Seeds:** none needed; run with `-E object_jitter=0`.
- **Layout lots:** trial k uses lot k mod 2 (`lots.py`): lot A as in the screenshot, lot B with the cradle and gauge
  3.0 cm nearer the robot and 7.0 cm to the left. `--epochs 2` runs both and averages them; `TASK_LOT=A` or `B` pins
  one. A replay of lot A's motions closes on empty air in lot B. A memorised replay of both lots is not ruled out.
- Files: `task.py`; `parts.py` (multi-shape parts with sliders), `tilt.py` (a tilted grip for the scripted demo),
  `lots.py` (layout lots), all importing only NumPy and `yam_arena`; `near_misses.py` (plain-Python checks of
  `score()` and `success()`, and of the air rule, no simulator needed).

## The mechanism

- **Wheel** (fixed): a 24-sided tyre ring (183 to 293.5 mm radius, 165 mm tall), a steel rim barrel (inside face
  168 mm from the axis, 12 to 152 mm up) with flanges and a centre disc. The valve stem leaves the barrel's inside
  face on the side nearest the robot, 110 mm up, along an axis leaning 30 degrees from vertical toward the hub: a
  rubber body (11.6 mm, 12-sided) then a brass barrel (7.7 mm outside, 5.2 mm bore, 12-sided), tip 30 mm from the
  base. The core is a 1.6 mm square pin on a slider along the stem, 8 g, 250 N/m with its rest point 6 mm out (1.5 N
  at rest, 1.8 N at 1.2 mm), so the push to open it stands for the core spring plus the tyre's air on its seal.
- **Gauge** (loose, 51 g total: 45 g body and 6 g stick, articulated): socket (12-sided, 8.8 mm, 10 mm deep), gasket face, depressor pin, head,
  barrel, a flat-sided grip 12 mm across, and the stick (6 mm, 6 g, a slider along the gauge, 66 mm of travel). The
  stick's drive holds it where the air last left it; an `on_step` callback moves that point.
- **The air rule** (`air_step`), each control step: if the core is pushed in 0.6 mm or more and the gasket is seated
  square on the barrel's tip, the stick is pushed out 4 mm per step (80 mm/s) toward 33 mm, the tyre's 32 psi. If the
  core is open but the gasket is not seated, air escapes round it and the stick goes no further than 3 mm (12 psi).
  The stick never goes back in by itself, as on a real pencil gauge; a later good press takes it on up. A press held
  under about 0.4 s therefore reads low. This is how the model stands in for air flow; it is not a measured leak
  curve.
- **Cradle** (loose, 0.35 kg): a base plate and two U-shaped saddles 75 mm apart, 22 mm between the walls for the
  14 mm barrel.

## Scoring as implemented

`score()` and `success()` in `task.py`.
- **Reading:** 10 psi plus the stick's travel at 1.5 mm per psi. Right: 30 to 34 psi (the tyre is at 32).
- **Let go:** an `on_step` callback measures every step the distance from both arms' collision hulls (forearm
  `link3`, `link4`, wrist `link5`, gripper and both fingers; every hull face sampled at most 3 mm apart, so a flat
  face resting on the gauge counts, not only hull corners) to every shape of the gauge; all must exceed 5 mm. Until it
  has measured, or if it cannot, the state is unknown and nothing scores (fail closed).
- **0.5:** the reading is right, the air rule has driven the stick to at least the 30 psi mark during this trial,
  and the gauge is let go. A stick pulled out by contact without that air-driven history scores 0. The air-history
  gate resets only at the simulator's scene reset; the real-trial equivalent is the video of the stick emerging
  while the chuck is seated on the valve.
- **1.0:** also in the cradle (middle within 12 mm along the saddles and 4.5 mm across, axis within 2.5 mm of its
  resting height, within 6 degrees of the saddles' line, either way round) and still for 20 control steps (1 s:
  its middle within 0.5 mm and its axis within 0.5 degrees of where they were at the start of that second, counted
  every control step by an `on_step` callback). Stillness is judged from the pose alone, as a person would from the
  video, not from velocity, because a gauge resting in contact jitters in velocity without moving.
- **Stillness ignores spin about the gauge's own axis.** The barrel is round and lies on flat saddle floors, so it
  can rock a degree or so about its own axis without going anywhere; in one trace it did so for 7 s after being let
  go. That rocking is not counted as moving. Any other turn of its axis over 0.5 degrees, or any shift of its middle
  over 0.5 mm, restarts the 1 s count. Success is the 1.0 state, held 1 s.
- **0:** no reading or a wrong one (a leak's 12 psi included), the gauge held, or dropped more than 2 cm below the
  table top.
- No credit for touching, lifting or carrying the gauge, or for pressing it on crooked.
- **Real trial:** read the stick against its marks, and check against a reference gauge on the same valve; the video
  must show the stick coming out while the chuck is on the valve (a stick pulled out by hand scores 0). A top photo
  for the cradle.

## Scripted demo

`demo()` uses fixed waypoints from the published start layout of the trial's lot only (no object state is read).

- **Right arm only.** Over the gauge's grip in the trial's lot, down, pinch the grip's flats 3 mm past their faces,
  lift; rise to 270 mm while leaning the finger tips 60 degrees toward the hub, which turns the level gauge about
  the fingers' line onto the stem's 30-degree lean; cross high over the tyre (the leaning gauge's head hangs about
  60 mm below the hand); come down onto the stem's line 40 mm out, settle, press 25 mm past seated (the arm is soft,
  about 190 N/m, so the chuck goes home with a few newtons), hold 1.5 s, back off along the line, rise, cross back,
  level the gauge over the cradle, lower it into the saddles, let go, move clear and wait 10 s.
- The approach and press targets are offset by the 3.5, -1.5, -2.0 mm (x, y, z) the loaded arm settled away from
  its target there in an earlier trace (`SETTLE` in `task.py`); without it the chuck's mouth landed on the barrel's
  tip 4 mm off its axis and never went over it. The chuck's 45-degree flared mouth takes up what is left.
- `tilt.py` drives the tilted grip by wrapping `yam_arena.demo.grip_rotation` in the task's own process; it affects
  only the scripted demo, never Dream-1 or scoring. Why a flat-sided grip: in the first trace the round barrel turned
  in the pads about their closing line as the hand leaned, and the gauge fell.
- `DEMO_TOUCHES = ("wheel",)`: the offline checker treats the fingers as closed; at the valve the chuck, not the
  fingers, closes round the stem.

## Checked on the GPU (Isaac Sim 6.1)

- **Final demo after the air-history gate repair** (`--epochs 2`, task_score 1.0: lot A 1.0, lot B 1.0, both trials ended on success): in each lot the
  hand took the gauge, turned it onto the stem's lean, pressed the chuck home, the stick ran out to 32 psi, and the
  gauge went back into its saddles and was let go.
- **What the press did** (a trace of both lots): the gasket came to rest 0.2 to 0.3 mm from the barrel's tip and
  0.4 mm off its axis, the core was pushed in 1.0 mm, and the stick read 31.7 psi during the 1.5 s hold and settled
  at 32.0 psi.
- **Dream-1** (`--epochs 2`): task_score 0.0 (lot A 0.0, lot B 0.0); both trials completed at the step limit. No evaluation errors were recorded.
- **Stillness rule, found on the GPU:** in lot A the gauge came to rest rolled about 21 degrees about its own axis
  and kept rocking about a degree about it, its middle steady to 0.1 mm. An earlier rule that counted every turn
  kept restarting its 1 s count, and lot A scored 0.5 in three demo runs; with spin about its own axis ignored, it
  scores 1.0.
- **Earlier traces:** a round barrel turned in the pads about their closing line as the hand leaned, and the gauge
  fell (hence the flat-sided grip); the leaning gauge's head struck the tyre while crossing it (hence the high
  crossing); the chuck's mouth landed on the barrel's tip 4 mm off its axis and never went over it (hence `SETTLE`
  and the flared mouth).

## Licences

No third-party assets: every object is built from boxes and cylinders in `task.py`.
