# Put both AirPods Pro back in their case, each stem-first into its own well, and push the lid shut so its magnet holds it

An Apple AirPods Pro 2 MagSafe Charging Case stands open on the bench, lid at 115 degrees on its far side, and both earbuds lie on their sides beside it. One gripper holds the case low at one end. The other picks each bud, turns it stem down, steers the stem down its own well's bore until the bud is within the well magnet's 7 mm reach, and lets go; the magnet pulls it home. Then it pushes the lid over from behind its far edge with a fingertip until the lid's magnet holds it shut.

## Run

From this folder:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,left_only,swap,one_bud
```

The harness's instruction (Dream-1 run): `--instruction "Put both earbuds back in their charging case and close the lid."`

## Scoring, and the routes that score less

Run on RTX 4070 Ti SUPER GPUs (RTX 5090 for the run record below); scores are the harness's task_score.

| Mode | What it does | Score |
|---|---|---|
| full | right hand holds the case; left seats the right bud, then the left bud, and pushes the lid shut | 1.0 (1,163 steps, 58.15 s) |
| none | both arms wait | 0.0 |
| left_only | the left hand does everything, the case unheld | 0.0 (both buds seat, but the push on the lid tips the case over, 90 degrees) |
| swap | each bud steered into the other bud's well | 0.0 (each side rib meets the other well's wall; both buds stay 6.3 mm proud and the lid rests 46 degrees open on them) |
| one_bud | the right bud seated, the left bud left on the bench, the lid pushed shut | 0.25 |
| proud | the second bud let go 12 mm up | 1.0 (it falls into its well under gravity and the magnet; this is a valid route, not a cheat) |
| remove_after | both buds seated, the case let go and the hands clear for 2.5 s (the score reads 0.5), then the hand steadies the case again, pulls the right bud out and sets it on the bench | 0.25 (the bud on the bench does not count; 1,531 steps) |
| case_moved | both buds seated, the case let go and the hands clear for 2.5 s (the score reads 0.5), then the right hand pushes the case 6 cm along the bench | 0.0 |

The score is the state at the end, never a best so far. Each bud seated in its own well, within 0.5 mm of home, 1 mm of its axis in each direction, 5 degrees of upright and 10 degrees of its turn: 0.25. Lid shut within 1 degree over both seated buds: 0.5 more. The case must end upright within 5 degrees and within 3 cm of where it stood, else 0. Judged once every fingertip of both hands is 3 cm clear of the case and all has been still for the last 1 s; until then the score is 0, and a bud taken out, a case moved or a hand back at the case changes it at once. Only failures stick: a case tipped past 20 degrees at any moment, or a bud off the bench, scores 0 (the case may be lifted, as long as it never tips past 20 degrees). The same rule runs in the real trial and in score(): look at the case only after the hands are away and it has been still for a second.

## Earlier feedback

- "The scorer retains its last settled partial score when the case or earbuds subsequently move or the hands are no longer clear." score() now recomputes partial credit from the current state at every step (`Trial.step` in task.py): no best-so-far is kept, nothing is judged until both grippers are 3 cm clear and everything has been still for 1 s, and only the two failures (a tip past 20 degrees, a bud off the bench) stick.
- "Add a test in which a seated earbud is subsequently removed before the run ends, and confirm that the final score reflects the actual outcome." Modes `remove_after` (both seated and the score at 0.5, then the right bud pulled out onto the bench: final 0.25) and `case_moved` (both seated, then the case pushed 6 cm: final 0), run on GPU; see the table above.

## Real trial

The real trial uses a real AirPods Pro 2 with its MagSafe Charging Case; the sim's well, key and magnet figures are estimates, and the real case's own wells and magnets decide the outcome. Stand the case on the bench with its lid fully open (115 degrees), the buds on their marks, lying on their sides, front up. Seated: turn the open case upside down and both buds stay in. Shut: turn the closed case upside down and the lid stays shut. Moved or tipped: the case stands on its base inside a tape outline drawn 3 cm outside its footprint. Judged from the trial video once both grippers are 3 cm clear and the case has been still for 1 s.

Kit: one AirPods Pro 2 with MagSafe Charging Case (Apple's US launch price $249; any AirPods Pro 2 case with the same wells works), masking tape and a pen for the outline and the bud marks. Reset: put the buds back on their marks and open the lid fully, about 20 seconds (estimate); nothing is consumed. Recharge the case between sessions if its light shows amber.

Forces and safety: the buds weigh 5.3 g and the case 50.8 g, and the lid's push is about 1 N (estimate), far below the YAM's roughly 40 N grip per finger and 20 N push. Nothing in the scene is sharp, hot or fragile enough to harm the rig; set a low grip force so the buds' gloss plastic is not marked, and stick-on soft pad covers on the fingertips are a cheap extra.

## Modelling

- Case at Apple's 45.2 x 60.6 x 21.7 mm and 50.8 g; buds at Apple's 30.9 mm length, 21.8 mm width and 5.3 g; bud head 17.3 mm deep: Apple's 24.0 mm includes the silicone ear tip, which squashes in the real case and is not modelled.
- The case is one free mechanism: its body, and the lid on a hinge along the far top edge. Each well is a bore shaped to the stem (5.7 x 7.6 mm, 23.3 mm long, 0.75 to 0.8 mm of play a side) with a chamfered mouth (2.5 mm a side, 4 mm deep) and two key slots over its top 10 mm: one for the rib on the stem's outer side (2.2 mm proud of the stem), one for the bump on its front. The buds are mirror images, so each fits only its own well, stem first, front toward the robot.
- Each well's magnet is code: a 0.4 N pull on a bud toward its seat, only while the bud's seat is within 7 mm of home, with the equal and opposite pull on the case. The lid's magnet is a drive in its own hinge, 1 N at the lid's front edge (21.7 mm from the hinge) over its last 5 degrees, so the case's body and lid pull on each other.
- A seated head stands 7.6 mm proud of the rim and the shut lid clears it by 1.4 mm, so a bud left proud holds the lid open.
- The demo reads object poses and the lid's hinge angle.

## Sizes

| Part | In the scene | Published | Source |
|---|---|---|---|
| case depth x width x height (lid shut) | 21.7 x 60.6 x 45.2 mm | 21.7 x 60.6 x 45.2 mm | Apple tech specs |
| case mass | 50.8 g | 50.8 g | Apple tech specs |
| bud length | 30.9 mm | 30.9 mm | Apple tech specs |
| bud head width | 21.8 mm | 21.8 mm | Apple tech specs |
| bud head depth | 17.3 mm | 24.0 mm with the silicone ear tip | Apple tech specs; the ear tip squashes in the real case and is not modelled |
| bud stem width x length | 5.7 x 23.3 mm | 5.7 x 23.3 mm | Apple dimensional drawing |
| bud mass | 5.3 g | 5.3 g | Apple tech specs |
| lid full opening | 115 degrees | 115 degrees | Apple dimensional drawing |
| well play, chamfer, key slots, magnet reach and pull, lid magnet | 0.75 to 0.8 mm a side, 2.5 mm x 4 mm, 2.3 mm half width, 7 mm and 0.4 N, 1 N | none published | estimates from maker photos |

`python task.py --sizes` prints the scene's sizes next to the published ones (no simulator starts):

```
part                                            scene  published
case depth (x)                                   21.7       21.7
case width (y)                                   60.6       60.6
case height, lid shut                            45.2       45.2
case mass, g                                     50.8       50.8
bud length (head + stem)                         30.9       30.9
bud head width (y)                               21.8       21.8
bud head depth (x), ear tip not modelled         17.3       24.0
bud stem width (x)                                5.7        5.7
bud stem length                                  23.3       23.3
bud mass, g                                       5.3        5.3
lid open, degrees                               115.0      115.0
```

## Run record

The command above, run three times on RTX 4070 Ti SUPER GPUs: 1.0, 1.0, 1.0, each in 1,163 steps (58.15 s), slowest 30 steps 23 to 27 s. Four runs with the free parts shifted by up to 5 mm at reset (`--jitter 0.005`, seeds 11 to 14): 1.0, 1.0, 1.0, 1.0 (1,163 to 1,174 steps). On an RTX 5090: 1.0 in 1,163 steps (58.15 s), slowest 30 steps 30.6 s.

## Sources

- Apple, AirPods Pro (2nd generation) tech specs, https://support.apple.com/en-gb/111851: each earbud 30.9 x 21.8 x 24.0 mm, 5.3 g; MagSafe Charging Case 45.2 x 60.6 x 21.7 mm, 50.8 g. Checked on 18 Nov 2026; the page is for the 2022 model with the Lightning MagSafe case, whose sizes and masses the task uses.
- Apple, Accessory Design Guidelines dimensional drawings, "MagSafe Charging Case (USB-C) for AirPods Pro (2nd generation)" and "AirPods Pro (2nd generation)", https://developer.apple.com/accessories/: the lid's full opening (115 degrees) and the stem's section (5.7 mm) and length (23.3 mm). Used for sizes only; the drawings are not included.
- Apple, "Charge AirPods (all generations) or AirPods Pro (all generations)", https://support.apple.com/guide/airpods/devde25a4bbe/web: "Put your AirPods in their case. Close the lid." and "AirPods shut down and charge whenever they're in the case with the lid closed." Checked on 19 Nov 2026.
- Apple Newsroom, "Apple announces the next generation of AirPods Pro", 7 Sep 2022, https://www.apple.com/newsroom/2022/09/apple-announces-the-next-generation-of-airpods-pro/: US price $249.
- Edison Research, The Infinite Dial 2025 (US survey of people aged 12 and over, January 2025): 59 percent own Bluetooth wireless headphones. Reported in "Streaming and Social Media Habits Surge, Says Infinite Dial", Radio World, 26 Mar 2025, https://radioworld.com/news-and-business/programming-and-sales/streaming-and-social-media-habits-surge-says-infinite-dial; report: https://ssrs.com/wp-content/uploads/The-Infinite-Dial-2025.pdf. The figure covers all Bluetooth headphones, earbuds and over-ear alike.
- Well, bore, key, magnet, hinge and friction figures are estimates from maker photos, as Apple publishes none.
- All meshes are built in `task.py` from boxes and prisms; no third-party meshes are used.

Apple, AirPods, AirPods Pro and MagSafe are trademarks of Apple Inc., named for identification only.

## Licence

MIT
