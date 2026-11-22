# Fit a comb busbar to a row of breakers and tighten each terminal to 2 N m with a torque screwdriver

A row of DIN-rail miniature circuit breakers is commonly fed by a pin comb busbar. The maker's steps (Contactum consumer unit instructions): "Loosen the
MCB terminal screws. Fit the busbar and tighten the terminal screws." Schneider Electric gives 2 N m as the terminal tightening torque for its iC60
1-25 A breakers. Each breaker's line terminal is a box clamp, shut until its screw is backed out, and a rigid comb fits only when every clamp in the row
is open.

The difficulty is in the hands:

- **Open the whole row.** Each terminal screw is backed out about 3.75 turns with the driver, in wrist strokes: the bit kept on the head through each
  turn, lifted off, the wrist turned back, the bit set in again. A clamp left shut stops the whole comb.
- **Insert into all at once.** The comb's teeth enter all the open terminals together, in one straight push down from above; skewed, a tooth catches.
- **Close the row to torque.** Each screw is driven back down onto its tooth and turned on, against a resistance that climbs to 2 N m, until the preset
  driver clicks. The arm supplies that torque through the driver's T-handle; a stroke that gives up short of the click leaves the terminal under-tightened.

**The arms.** In the demo the right arm works the driver and the left arm lays the comb. The scoring asks for the result only: any division of the work
is allowed.

## Where this task comes from

This idea was first submitted by another team (board TK-PSBF2Q). Its review asked for two changes: require the maker's 2 N m terminal torque with a
torque-capable screwdriver instead of finger-tight screws and a pull test, and say how the comb's unused fourth tooth is insulated. That team's revision
hold lapsed without a revision. This task makes those two changes, and builds on the original's MIT-licensed code (`kit/mechkit.py` and `kit/demokit.py`
unchanged except for one added function, `twist`; the task file and the scorer check adapted). With thanks to the original authors.

What changed from the original:

| | Original | This task |
|---|---|---|
| Tool | a plain #2 Pozidriv screwdriver, round handle | a T-handle torque-limiting screwdriver preset to 2.0 N m, PZ2 bit |
| Screw resistance | none: the screw followed the tool's turn | a counter-torque on the driver about its axis while it turns: 0.15 N m running, rising to 2.0 N m over 30 degrees past the tooth |
| "Tightened" | the screw driven onto its tooth (about 10 N on it) | the driver turned 15 degrees further against its slipping 2.0 N m clutch: its click |
| Spare tooth | bare, the hand's grip | covered before the trial by the range's yellow insulating tooth cap (A9XPT920); still the hand's grip |
| Partial credit | 0.1 per screw down | 0.1 per terminal at 2 N m, 0.04 per screw only driven onto its tooth |

Why a T-handle: a hand gripping a round handle turns it only by friction close to the axis, and a parallel gripper on a 24-32 mm handle holds
well under 2 N m that way. The demo grips the T-bar by its two ends, 37 mm either side of the axis, as a fist does: with about 50 N a finger at the
rubberised handle's 1.2/1.0 friction, that holds about 3.7 N m. In the first simulated runs, a grip pinching the bar near the axis slipped round
before 2 N m. The YAM's wrist joints are modelled with a 7 N m effort limit (DM4310 joint motors), so 2 N m is within reach.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`).
- `kit/mechkit.py` and `kit/demokit.py`, in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without modifying
  them.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=fit_comb_busbar.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --policy dream -P rows_per_request=20 --scorer task_score --max-steps 10000 \
    --instruction "Fit the comb busbar to the three breakers and tighten each of their top terminals to 2 N m with the torque screwdriver, then put the screwdriver down and move both hands away."
```

The scene is the same for every seed.

## The demo

From this folder:

```bash
python kit/demokit.py fit_comb_busbar.py OUT                                                # the full-score run, with video
python kit/demokit.py fit_comb_busbar.py OUT_checks --mode closed,behind,loose,hover,snug,leave --fast  # the failure checks
python test_fit_comb_busbar.py                                                           # the scorer and torque model, no simulator
```

The demo is a scripted reference, not a policy. It reads the driver's, the comb's and the terminals' poses from the simulator; all of them are in view of
the cameras. It never reads the scorer.

1. The right hand takes the driver from its stand by the two ends of its T-bar, as a fist grips it, draws it back clear of the breakers
   and squares it to the row.
2. For each breaker it sets the bit on the terminal screw's head and backs the screw out in wrist strokes, keeping the bit's tip on the head through each
   turn, until the clamp is open. It parks the driver.
3. The left hand takes the comb by its bar over the capped fourth tooth, carries it over the row and lowers its three teeth into the open terminals, then
   lets go: the comb rests on the breakers' tops.
4. The right hand drives each screw back down in strokes, onto its tooth and on until the driver clicks, lays the driver back in its stand and backs away.

Demo results (2026-11-22, Isaac Lab-Arena 0.3.1 on an L40S):

- **Full run, GPU physics:** task score 1.0, success at step 6,257 of the 10,000-step limit (313 s). All three terminals click; the right arm's
  worst IK miss is 8.2 mm, on the reach to the left screw.
- **Failure checks** (the same script, changed at one step):

| Mode | What changes | Score |
|---|---|---|
| `closed` | the screws are never backed out: the comb's teeth stand on the shut clamps | 0.0 |
| `behind` | the teeth aimed into the space behind the shut jaws, then pushed onto them | 0.0 |
| `loose` | the comb fitted, the screws never driven back in | 0.34 |
| `hover` | the bit held 4 mm in front of each head while the wrist turns | 0.0 |
| `snug` | each screw stopped as soon as it bears on its tooth, short of the click | 0.46 |
| `leave` | all three clicked, but the bit set back in the left head and the hand left on the driver | 0.64 |

`closed`, `behind`, `loose` and `hover` ran on CPU physics before the last change to the tightening strokes and the back-turn rule, neither of which
those modes reach; `snug` and `leave` ran on CPU physics with the final task file.

## Scoring

`--scorer task_score` prints the total. The task file sets `SUCCESS_HOLD_S = 0` and counts its own hold: `score()` is 1.0 only once the 1.0 state has
held 40 control steps (2 s) in a row, with the pull test inside it, and `success()` is true from then on.

- **0:** once the comb or the driver has fallen off the table, for the rest of the trial.
- **0.03 for each terminal opened:** its screw backed out far enough for a tooth (3.7 turns), counted once it has been (0.09 for the row).
- **0.25 more while the comb is home:** all three teeth in their clamps and its bar down on the breakers' tops (within 1.5 mm), square to the row in plan
  (within 3 degrees).
- **For each terminal, 0.1 more once tightened to 2 N m** (driven onto its tooth and turned on until the driver clicks), **or 0.04 if only driven onto
  its tooth**: 0.64 at most with all three tightened.
- **1.0:** all three tightened to 2 N m; every fingertip of both hands at least 5 cm from the whole comb and 4 cm from the whole driver (its axis and both
  ends of its T-bar); the driver's tip at least 2 cm from the screw heads; and a 10 N pull up along the teeth moving the comb less than 1 mm; all held 2 s.
  A pull that moves the comb more leaves the trial at its partial score.
- **Backing a tightened screw off** more than 3 degrees undoes its click (its torque is then more than 10% under the preset): it has to be tightened
  again. A turn back takes off only the angle it covers, so the few degrees a wrist springs back at the end of a stroke don't undo a click.

Real trial: read from the film and checked by hand. A terminal is open when a 3 mm gauge pin drops into its clamp. The bar is down when a 1.5 mm feeler
gauge does not pass under it, and square when a try square laid along the row's fronts meets it within 3 degrees. A terminal is tightened to 2 N m when
the driver's click is on the film and a torque screwdriver set to 1.8 N m then does not turn the screw; a screw that turns at 1.8 N m but is on its tooth
counts as only driven onto it. The pull is made with a spring balance hooked under the comb's bar, the comb's rise read on a dial gauge. The hands'
distances are read from the film against a tape on the bench, and the 2 s hold by the film's timer.

## Real trials: the build

- **Board.** A painted steel mounting plate on a wooden board, standing upright at x 38 cm, with a 35 mm DIN rail across it 15 cm up.
- **Breakers.** Three single-pole miniature circuit breakers of one range with box-clamp line terminals in their tops and M5 terminal screws in their
  fronts (for example Schneider Electric Acti9 iC60N 1P, 16 A: 91 x 18 x 78.5 mm, 0.125 kg, 18 mm pitch; terminal torque 2 N m), clipped side by side,
  fronts towards the arms. Back each screw down onto its empty clamp, finger-tight, before a trial.
- **Comb busbar.** The same range's comb busbar (for example Acti9 A9XPH112, 18 mm pitch), cut to four teeth, with one yellow tooth cap from an Acti9
  A9XPT920 set pushed onto the fourth tooth. Measure the tooth and cap sizes (modelled 3 x 6 x 13 mm and 9 x 6 x 15 mm). Lay it on two thin posts at
  (21.5, 16) cm, its bar 10 cm up.
- **Torque screwdriver.** A T-handle torque-limiting screwdriver set to 2.0 N m with a 1/4-inch hex drive and a PZ2 bit: for example a CDI TorqControl
  TLA28NM (adjustable 2-8 N m in 0.1 N m steps, 1/4-inch hex magnetic bit holder) with a 25 mm PZ2 insert bit. Measure its T-bar, barrel and mass
  (modelled 75 x 20 x 26 mm, 24 mm, 240 g). In a V-stand whose rests are at x 24.7 to 28.7 cm, y -15.5 cm, its axis 17 cm up.
- **Torque check.** A second torque screwdriver that can be set to 1.8 N m, for the hand check after a trial.
- **Safety.** De-energised breakers only, never connected to supply.

## The model behind it

The breakers are one fixed articulation. Each terminal's screw turns on a hinge and its clamp's jaw slides on a slider. The thread is code: the driver's
turn about its own axis turns a screw only while its bit's tip is on that screw's head (within 0.6 mm of its face and 2.5 mm of its centre, the shaft
within 10 degrees of square), either way; the jaw follows at 0.8 mm a turn and stops on whatever it closes on. A screw driven home onto its tooth clamps
it: its jaw is driven 2 mm past contact on a 20 N/mm drive, capped at 60 N (the jaw presses the tooth with about 10 N).

While a screw turns, its resistance is put on the driver as a torque about the driver's own axis, against the turn, at every physics step until the next
control step: 0.15 N m while the screw runs free `[estimate]`, rising linearly over 30 degrees past the jaw's contact with the tooth to the driver's
2.0 N m preset, then held at 2.0 N m (the clutch slipping). Turning 15 degrees on against the slipping clutch is the click. A driver that is not turning
feels no torque. The comb and the driver are free rigid bodies; the driver's centre of mass is in its handle. The screw heads have no collider of their
own: where the bit's tip is on a head is read in code.

## Attribution

The task statements quoted in `task.json` are from the O*NET database (O*NET OnLine, task statements for 47-2111.00,
https://www.onetonline.org/link/summary/47-2111.00) by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the
CC BY 4.0 licence. O*NET is a trademark of USDOL/ETA. Sources quoted:

- the installation steps: Contactum consumer unit instructions (https://www.tlc-direct.co.uk/Technical/DataSheets/Contactum/Cu_Instructions.pdf);
- the terminal torque: Schneider Electric, "How to tighten to the correct torque for Acti9 products" (https://www.se.com/uk/en/faqs/FAQ000132733/) and its
  Acti9 tightening-torque table (iC60, 1-25 A: 2 N m);
- the breaker's figures: Schneider Electric product data sheet, Acti9 iC60N 1P (A9F04116W, https://docs.rs-online.com/11bf/A700000015874965.pdf);
- the comb busbar: Schneider Electric product data sheet, Acti9 comb busbar (A9XPH112W, https://docs.rs-online.com/da9d/A700000015872170.pdf);
- the tooth cap: Schneider Electric, Acti9 set of 20 tooth caps (A9XPT920, https://www.se.com/il/en/product/A9XPT920/set-of-20-tooth-caps-for-acti-9);
- the torque screwdriver: CDI TorqControl TLA28NM listing (https://www.zoro.com/cdi-cdi-torque-limiting-t-handle-adj-2-8nm-tla28nm/i/G7640696/).

Contactum, Schneider Electric, Acti9 and CDI are trademarks of their owners, named for identification only.

## Assets

Every mesh is generated by `fit_comb_busbar.py` and `kit/mechkit.py`. There are no textures and no third-party assets.

## Licence

MIT. Portions are adapted from the MIT-licensed original submission of this idea (TK-PSBF2Q).
