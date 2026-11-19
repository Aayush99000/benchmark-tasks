# Brake pad change: push two pads flush into their retainers each side of the disc, then swing the caliper down over them

A front brake corner of a small hatchback stands as it would on a stand with the wheel removed, still under its fender, its caliper pivoted up on its upper guide pin. The robot reaches in through the wheel arch, past the inner arch liner, the strut and the control arm, and pushes two new pads into their sprung retainers either side of the disc, one at a time, until each seats flush, then swings the caliper down over them until it shuts.

**Ability tested:** a sequence of fits with tight clearances. Each pad is carried under the raised caliper and its ears pushed 12 mm into the two retainers on its side of the disc until its backing plate seats flush: the outer pad first, then the inner pad with its shim and wear indicator; both go in from the arch side. The 2 kg caliper is then swung down about its upper pin, so that its piston and its outer fingers pass either side of the 21 mm disc and the pads with 2 mm to spare, and its bridge closes over both pads' edges with 1 mm: a pad standing more than 1 mm proud stops the caliper short. In the demo the left hand fits both pads; the right hand starts the caliper down from the top and the left hand finishes it (the hand-over is at 126 degrees); the score does not check which hand does what.

**Why it matters:** replacing brakes is a core task of automotive service technicians and mechanics (O*NET 49-3023: "Repair, reline, replace, and adjust brakes"), and the pad change is the most common brake job. The order is the maker's: pivot the caliper up out of the way, install the pads with the wear indicators on the inside, pivot the caliper down into position.

**The wheel well and suspension (revision).** Robocurve's review asked for the surroundings that constrain access to an installed brake. They are now solid collision geometry in the scene, sized from a Honda Jazz (GD) front corner: the fender panel with its wheel arch (radius 35.5 cm about the hub, an estimate), the inner arch liner (a sheet arc of radius 38 cm, an estimate), the MacPherson strut (54 mm damper, 125 mm spring) on its clamp, the lower control arm from the knuckle's ball joint to two bushings, the tie rod and the bulkhead. The wheel is off, as in a real pad change; the car is on a stand. Simplifications, also in `fidelity`: the fender panel is cut away on the car's end side beyond 110 degrees of the arch and the liner stops at 150 degrees, so that the left arm can enter; no skirt under the arch (the table is the road); the hub is not turned to lock.

**The real geometry.** The corner has a round vented disc at its real thickness, a caliper body with its piston and fingers on its guide pin, the bracket with sprung pad retainers, and pads with ears and backing plates, all at the scale of a Honda Jazz front brake. The dimensions and clearances are taken from that car's parts; the retainers' spring drag is not modelled and the push into them is not measured (see the calibration table below).

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Fit the new brake pads, then swing the caliper down over them."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                                    # full score, with a video
python demokit.py task.py OUT --no-record --mode noflush,onepad,noshut,none   # the failure modes below
```

The demo is a scripted reference. It reads the poses of the pads and of the caliper, which the cameras see; a policy gets only the cameras. The left arm takes each pad from the parts box by its edge, 25 mm below its centre (a hand tilted 70 degrees, the jaws across its 15 mm, 40 N pinch at most), carries it through the wheel arch under the raised caliper, squares it up to the retainers, pushes it straight in 12 mm, lets go and seats it with closed fingertips on its rim. The right arm then pushes the caliper's bridge round its pin from the top from 146 degrees, and the left arm finishes it from 126 degrees down to shut; both hands back away.

## Building it for a real trial

- **Wheel well and suspension:** a plywood fender panel (1.24 m x 0.8 m, 12 mm) with the wheel arch cut at radius 35.5 cm about the hub (measure the real car's arch and cut to that), an inner arch liner rolled from 2 mm sheet to radius 38 cm, and the strut and lower control arm that come with the salvaged corner, mounted on the same frame as the knuckle. Leave the car's end of the well open. The wheel is off.
- **Brake corner:** a front corner of a Honda Jazz (2002 to 2008, 1.3) or the same class, from a breaker's yard, with its knuckle, strut and lower arm, bolted to a frame (a stand) with the disc's centre about 25 cm above the bench: a vented disc 262 mm across and 21 mm thick, the caliper bracket bolted to the knuckle, four pad retainers fitted and greased.
- **Caliper:** the single-piston floating caliper on its upper guide pin, its piston pushed back, its lower guide bolt out, pivoted up out of the way (150 degrees from shut, hung from the strut with a wire, as a technician does). Weigh it: about 2 kg is our estimate. If it does not stay put, add friction at the pin, for example an O-ring under its rubber sleeve, until the luggage-scale test in the table below reads about 1.3 N m. The brake hose may stay connected; it hangs slack.
- **Pads:** a new pair for that car (131.4 x 54.7 x 15.3 mm with ears, 0.2 kg each), the inner one with its shim and wear indicator, standing on one ear, linings towards the disc's side they belong to, in a slotted wooden parts box (70 x 100 mm, 12 cm tall) beside the stand.

Calibrate before trials:

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| Caliper mass | 2.0 kg | **Not measured.** Weigh it |
| The guide pin's hold | 1.3 N m of dry friction, above the caliper's 1.18 N m worst weight moment, so it stays where it is left | **Not measured.** With a luggage scale at the caliper's lower end, read the pull at which it starts to swing, times the distance from the pin |
| The pads' push into the retainers | greased channels, friction 0.3, 1.5 mm wider each side than the real clip slot (a position-driven hand cannot square a pad as a technician's fingers do); no spring drag or snap | **Not measured.** Push a pad into its retainers with a push gauge and read the drag and the snap at flush |

## Safety for a real trial

Protect the gripper's fingers from the pads' edges and from grease: cover the fingertips with rubber or nitrile and wipe grease off the retainers' contact faces. The corner is heavy and the caliper swings on its pin: clamp the stand to the bench, keep hands out of the caliper's swing while the robot works, and keep an emergency stop in reach. Brake dust: do not blow it; wipe the bracket with brake cleaner before the trial.

## Scoring

See `scoring` in `task.json`, and `score`, `success` and `pad_now` in `task.py`. In a real trial:

- **Pads flush:** each pad in its own retainers, the inner pad (shim and wear indicator) on the piston's side and the outer on the fingers' side, both linings facing the disc, and pushed the full 12 mm in; a 1 mm feeler gauge must not enter between any ear and its retainer's end. A pad on the wrong side or turned round does not count.
- **Caliper shut:** swung down onto its bracket, its lower bolt hole lined up with the bracket's thread (within 1 degree in the simulator).
- **Hands clear:** no gripper within 5 cm of the pads or the caliper's bridge, all of this for 2 s: 1.0.
- **0.3:** both pads flush, without the 1.0 result, or both flush only because the closing caliper pushed one home. **0.1:** one pad flush. **0:** otherwise. A pad dropped off the bench caps the score at 0.1.

In the simulator, a pad counts as flush when its centre is within 1 mm of its seated position along the retainers, within 5 mm across and 3 mm up or down, and turned less than 4 degrees from its seated pose. The lower guide bolt is left out of this version: the task ends with the caliper shut over the pads. A pad standing proud stops the caliper short, or the caliper presses it home; either way the score stays at 0.3 or below. Each failure mode is run in its own simulator session (a second mode in the same session starts from the first one's leftovers).

## Results

| Run | What it does | Score |
|---|---|---|
| `full` (the film) | Both pads pushed in 11 of 12 mm and seated flush (the outer turned 3.4 degrees, the inner 1.3); the caliper swung shut to -0.3 degrees; held 2 s | 1.0, at step 1835 (91.8 s of the 120 s limit) |
| `noflush` | The inner pad pushed only 6 mm and left; the closing caliper presses it home, which does not count: the pads must be flush before the caliper comes down | 0.3 |
| `onepad` | The inner pad left out: the caliper closes on the lone outer pad | 0.1 |
| `noshut` | Both pads flush, the caliper left up | 0.3 |
| `none` | The arms wait | 0.0 |
| Dream-1 | The right hand reaches into the parts box and touches the new pads there, but never carries one to the retainers; it then draws back, the caliper still up | 0 |

No seed is needed: nothing in the scene is random. Runtime per trial, on one GPU: the demo's 1,835 steps took about 12 minutes while recording three cameras (2.4 steps per second); a run to the 2,400-step limit takes 10 to 18 minutes.

## Assets

All meshes are made by the task's own code from plain shapes. No third-party assets, no textures.

## Licence

MIT
