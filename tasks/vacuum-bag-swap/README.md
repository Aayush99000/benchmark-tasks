# vacuum-bag-swap

**Instruction:** "Put a new bag in the vacuum."

A bagged canister vacuum stands free on its wheels on the bench, its front toward the arms, its hose
plugged in and lying along its right side to the handle, wand and floor nozzle behind it, and its
dust-compartment lid swung open on its rear hinge. The full bag hangs in the compartment from its
cardboard collar, which stands in the collar holder on the front wall's inside, over the inlet. To the
front right an opened carton of new bags holds one standing up, tab first; to the front left is an
empty wastebasket. The vacuum rolls if pushed, so one hand steadies it while the other works.


## Why this job

Changing the full dust bag in a bagged canister vacuum is one of the most familiar household chores,
and routine work for housekeeping staff. The full bag comes up by its collar's tab and goes straight in
the bin; the new collar has to go down the holder's channels square until it stops, so its gasket seals
round the inlet, and the lid is shut and latched, all while one hand keeps the vacuum, which rolls on its
wheels, from being pushed away.

## Real parts

| object | real part | size |
|---|---|---|
| vacuum | bagged canister vacuum, ABS shell, free on two 150 mm wheels and two castors | 470 x 290 x 255 mm, 5.5 kg; compartment 175 x 250 mm inside, rim 205 mm up |
| hose, wand, nozzle | 38 mm corrugated hose, brushed-aluminium telescopic wand, 240 mm floor nozzle (drawn) | hose along the right side; wand across the bench behind |
| lid | red ABS domed shell on a damped rear hinge, latch, inspection grille | 200 x 290 x 82 mm, 0.30 kg |
| collar holder | two short upright plastic channels with a chamfered lead-in, and a stop | 12 mm gap, 30 mm tall; stop 70 mm up; inlet 38 mm, 130 mm up |
| old bag | printed cardboard collar with rubber gasket ring, full quilted fleece bag | collar 120 x 125 x 2 mm, grip tab 40 x 75 mm; bag 127 x 200 x 117 mm; 0.18 kg |
| new bag | same collar, fleece folded behind it | pack 96 x 125 x 14 mm; 0.03 kg |
| carton | printed paperboard, opened, at (0.30, -0.30) | 140 x 75 x 75 mm |
| wastebasket | polypropylene, rolled rim, at (0.30, 0.43) | 260 mm across, 230 mm tall |

## Real-world build

No maker publishes the dimensions of its bag collars and holders, so the holder and the collars are
made from the files in `fabrication/` (written by `fabrication/make_fabrication.py` from the very
dimensions the simulation uses, `vac.py`):

- **Vacuum:** any bagged canister vacuum whose bag compartment is at least 175 x 250 mm inside and 175 mm
  deep, for example a Miele Complete C3 or Classic C1 (GN bags). Take out its own bag holder.
- **Holder:** print `holder.stl` (PLA or PETG, 0.2 mm layers, plate face down: a 1.5 mm plate with a 50 mm
  opening, two 30 mm channels with a 12 mm gap behind lips covering 4 mm of each collar edge, chamfered
  lead-ins, and the stop ledge 60 mm below the opening's centre). Tape it to the compartment's front wall
  with 1 mm double-sided foam tape, its opening round the inlet. Glue `bevel.stl` along the wall's inner
  top edge over the holder.
- **Collars:** cut collar and tab (120 x 125 mm + 40 x 75 mm, 38 mm hole) from 2 mm greyboard with
  `collar.svg`; glue on the gasket ring (52/38 mm, 3 mm EPDM foam), its hole over the card's.
- **Full bag (about 5 minutes, clean and contained):** cut a 300 x 220 mm sheet of 60-80 g/m2 spunbond
  polypropylene fleece, fold and glue (or staple) it into a 200 mm wide pouch, and glue its mouth round the
  back of a collar's hole. Put in a quart zip-top freezer bag holding 150 g of wheat bran (about 0.6 L),
  sealed. Fold and tape the pouch into a 127 x 200 x 117 mm block behind the collar and check the whole on
  a kitchen scale: 180 +- 10 g. The bran never leaves the sealed inner bag, so the full bag is reused from
  trial to trial (refold and re-seat it, about 2 minutes).
- **New bag:** a collar with an empty fleece pouch folded flat behind it (96 x 125 x 14 mm, about 30 g),
  standing in an opened 140 x 75 x 75 mm carton, pack toward the carton's back.

## Scoring

Run limit: 2,000 control steps (100 s at 20 Hz). Positions that concern the vacuum are measured in
its own frame, so a vacuum that rolled is judged as it now stands.

- **0.25** the old bag is out of the vacuum and its collar lies in the wastebasket (inside its rim's
  radius less 10 mm, below the rim), let go (no hand within 60 mm of its tab), and it stays there.
- **0.5** and the new collar is started in the holder: its bottom edge inside the 12 mm channel (within 1
  mm), centred across within 8 mm, square to the front wall within 10 degrees, below the channels' tops.
- **0.75** and the new collar is seated: its bottom edge within 3 mm of the stop, above or below.
- **1.0** and the lid is shut on the rim and latched, every finger pad (measured from the tool centre
  and the jaw opening) at least 80 mm from the latch button and from the new collar's tab, and the
  vacuum, lid and new bag at rest (under 10 mm/s and 0.1 rad/s), held 1 s.
- **Caps:** the vacuum pushed more than 30 mm over the bench at any time caps the score at 0.5 (it
  was not steadied; a canister shifts a little on its wheels while handled, 30 mm is a shove); either bag dropped on the bench (not in the bin, the carton or the vacuum) caps it
  at 0.25.

Every outcome scores: a bag never moved scores 0; a bag binned with the new one never started 0.25; and
so on up the ladder. All the measures are distances and angles a tape measure and a protractor (or a
photo) can check on a real vacuum.

## Simulation notes

- **Vacuum:** one free rigid body (5.5 kg). Its wheels and castors are welded to it and meet the bench
  with friction 0.15, standing in for a canister's rolling resistance: a hand pulling on the lid or pushing on
  the collar moves the vacuum unless the other hand holds it.
- **Collar holder:** pure geometry: two channels 12 mm wide behind lips, a chamfered lead-in, a stop.
  Nothing pulls a collar into place.
- **Lid and latch:** the lid swings on a damped hinge on the body (0.008 N m s per degree), shut on the
  rim at its hinge's stop. Its centre of mass is set 130 mm forward of the hinge and 10 mm up (its latch
  and front moulding): open past upright it rests on its stop; swung forward past about 85 degrees it falls
  shut by itself. The latch is a disclosed rule: once the lid is within 2 degrees of shut and a
  hand's tool centre is within 60 mm of the latch button, the hinge is held by a very large damping
  (no spring, no target); the lid is never snapped to a pose.
- **Bags:** each is one rigid body: the full bag does not sag, swing or shed dust; the new bag's pack
  stays folded. The hose, wand and floor nozzle are drawn (no collision). The motor and suction are not
  simulated.

## Scripted demo

The right hand holds the vacuum's right side wall from the front while the left pinches the old
collar's grip tab from above, draws the bag straight up out of the holder, carries it over and drops it
in the wastebasket. The left hand then holds the vacuum by its hose socket on the front, clear of the
lid, until the end. The right hand takes the new bag out of its carton by its tab, turns it square to the
holder, runs it down the channels to the stop and lets go; then grips the lid's side skirt, swings it
forward to about 70 degrees open and lets it fall shut, and presses its front down until it latches.
Both hands let go and stand clear.

## Reproduction

`prepare` exports the runtime into `runtime/`. From this task directory, with the
`yam-inspect-isaac:6.1.0` image (or `python3 runtime/build_image.py --image yam-inspect-isaac:6.1.0`):

```bash
mkdir -p /tmp/rc-task-home/tmp ../vacuum-bag-swap-outputs
docker run --rm --gpus device=0 --network host --shm-size 8g \
  --user "$(id -u):$(id -g)" -v "$PWD":/task:ro -v "$PWD/../vacuum-bag-swap-outputs":/outputs:rw \
  -v /tmp/rc-task-home:/tmp/task-home:rw -e HOME=/tmp/task-home -e TMPDIR=/tmp/task-home/tmp \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e OMNI_KIT_ACCEPT_EULA=YES -e ACCEPT_EULA=Y \
  --workdir /outputs --entrypoint /isaac-sim/python.sh yam-inspect-isaac:6.1.0 \
  /task/run.py --policy scripted --episodes 1 --livestream 0 --viz none
python runtime/rescore.py --task task.py ../vacuum-bag-swap-outputs/TASK/RUN/episode-01/trajectory.jsonl \
  --expect ../vacuum-bag-swap-outputs/TASK/RUN/episode-01/result.json
```

The outputs are mounted outside the task folder, so the run's source snapshot never copies them.
Seed 0, no randomisation; the scene settles for 30 ticks after reset before control starts. Control is
20 Hz and physics 240 Hz. A trial is at most 2,000 steps (100 simulated seconds); the demo ends on success at step 1,047; about 10-20 minutes of wall time on a shared GPU. Each
episode writes four 20 fps camera videos, setup/final PNGs, the measured state trajectory and
result.json. The scorer reads only measured object poses, robot TCP/gripper state and the lid's latch
state.

## Files

`task.py` (scene, latch rule, demo), `vac.py` (geometry), `scoring.py` (scorer), `simkit.py` (compound
rigid parts and joints), `fabrication/` (holder and bevel STLs, collar template SVG, and the script that
writes them from `vac.py`). No third-party assets.
