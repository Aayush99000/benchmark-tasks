# Change an Apple Watch band: hold the release button while the old half slides out, then slide the new half in

One everyday act with two hands: a face-down Apple Watch Series 11 (46 mm) has its old band half fitted. One hand holds the band release button down on the watch's back, while the other draws the old half's bar out along its slot. The new half is then slid into the same slot until the catch clicks.

## Sources

- Steps and the check ("hold down the band release button, then slide the band across"; "slide the new band in until you feel and hear a click"): Apple, Change your Apple Watch band, https://support.apple.com/en-asia/108908 ; Apple Watch User Guide, https://support.apple.com/guide/watch/apd3cc13d48f/watchos
- Case size 46 x 39 x 9.7 mm and the 37.8 g listing: Apple Watch Series 11 tech specs, https://www.apple.com/apple-watch-series-11/specs/
- Band fits 44 to 49 mm cases: Apple, Sport Band listing.
- Slot, bar, notch, catch and button sizes, forces and frictions: not published by Apple, so they are estimates from maker photos (listed in the setup text). The slot clearance is widened from about 0.2 mm to 1.0 mm a side so that the rigid-body simulator holds it.
- No third-party assets or CAD: every part is modelled from boxes in `task.py`. Apple's Accessory Design Guidelines carry band-slot drawings for band makers; they were not used here.

Apple and Apple Watch are trademarks of Apple Inc. This task is not affiliated with or endorsed by Apple.

## Scoring

Judged once both grippers are 3 cm from the watch and bands and everything has been still for 1 s. Old half out of the slot: 0.25. New half in the slot within 0.5 mm of centred with the catch dropped into the bar's notch and the button up, printed side up, old half out, watch within 3 cm of its start and level: 1.0. In the simulator these are read from the bar poses in the watch's frame, the button's and the catch's joints, and the fingertips.

## Results

Scripted demo: 1.0 on GPU physics in 3 of 3 runs (866 steps, 43 s; run limit 2,400). Dream-1: 0 in 2,400 steps.

## Run it

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Change the band's top half for the new one: hold the band release button for the top slot while you slide the old half out, then slide the new half in, printed side up, until it clicks."
```

From this folder:

```bash
python demokit.py task.py OUT --mode full
python demokit.py task.py OUT --no-record --mode none,no_button,half_in
```

## Failure table

Each lazy route is a `--mode` of the demo and must score no more than its cap.

| Route | Mode | Cap | Result |
|---|---|---|---|
| Do nothing | `none` | 0 | 0 |
| Draw the old half without the button held | `no_button` | 0 (the catch holds the bar) | 0: the old half stayed in |
| New half pushed in 6 mm short of the catch | `half_in` | 0.25 | 0.25: old half out, new bar not in the notch |

Nothing is random (object_jitter=0).

## Licence

MIT
