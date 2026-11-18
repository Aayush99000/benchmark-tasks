# Load a new lancet into a lancing device: cap off, lancet seated, tab twisted off, cap back on, cocked

A lancing device is the spring-loaded pen that pricks a fingertip for a blood-glucose test. It lies on the table with
its end cap on; a kit tray beside it holds one new twist-top lancet. The robot has to get the device ready for a test:
take the end cap off, push the lancet into the holder until it is gripped, twist the lancet's protective tab off to
bare the needle, put the cap back on until it snaps, cock the device by pulling its arming barrel back until it
clicks, and lay it down.

**Instruction:** "Load the new lancet into the lancing device: take the device's end cap off, push the lancet into the
holder, twist off the lancet's protective tab, put the end cap back on, then cock the device by pulling its arming
barrel back until it clicks. Lay the device down on the table."

The steps are the ones in a pen lancing device's directions for use: cap off, lancet pushed into the holder until it
is held, its round protective cover twisted off, cap back on over the needle, and the barrel at the far end pulled
back until the device clicks. What makes them hard for a robot is in the parts, not in the list:

1. **Nothing here can be worked on with one hand.** The device weighs under 30 g and its cap snaps on with 2 N. A
   pull on the cap of a device lying loose only drags it across the table. Pulling the barrel back against its spring
   does the same. One hand has to hold the device while the other works on it, and the instruction does not say so.
2. **The tab has to be twisted, not pulled.** Its moulded neck yields to a light finger torque and lets go after an
   eighth of a turn, but along its length it stands 8 N, three times the holder's 2.5 N grip on the lancet. A straight
   pull does not bare the needle: a light pinch slides off the tab (the control run below), and a pull firm enough to
   matter would bring the lancet back out of the holder first.
3. **Three small fits in a row.** A 4 mm lancet into a 4.8 mm socket, a 6.4 mm tab taken in the fingertips, and a cap
   that has to pass over a bare needle and onto a 9 mm nose, all on a device held in the air by the other hand.
4. **A bare needle in the middle of the job.** From the moment the tab is off until the cap is back on, the needle is
   exposed. A gripper that touches it has spoiled the lancet (and, with a person holding the pen, pricked them): the
   trial scores 0.

## Why this task is different

**A whole device procedure with real fits and snaps, not one insertion.** The score ladder follows the device's own
states: lancet gripped, needle bared, cap snapped on, spring cocked. Each depends on the one before, and two of the
snaps are found only by pushing far enough.

**Two ways of holding and two ways of turning.** One hand holds a 17 mm pen in the air against pulls and pushes along
its length. The other has to work from two directions: leaning in from the side for the cap and the lancet, and
level, end-on, to twist the tab with its wrist.

**The wrong force gets a different result, not a slower one.** Pull the tab and it stays on. Pull the cap with nothing
holding the device and the device comes too. Push the cap on short of its snap and it is not on: it scores as off.

**Nearest tasks on the board, and how this differs.**

- "Load an IV set into an Alaris pump module" (accepted) and "Take the cap off a child-resistant medicine bottle"
  (accepted) are also device procedures. The pump is fixed and the bottle is one push-and-turn. Here the device is a
  loose 29 g pen that has to be held throughout, and the procedure has four separate fits: a pull-off snap, a push-in
  grip, a twist-off neck and a sprung slide.
- "Replace a cartridge in a ratchet-rod dispenser" (accepted) and the pending task that loads an anaesthetic
  cartridge into a dental syringe are the same family, a consumable into a hand-held tool, with different mechanisms:
  nothing is twisted off, nothing is cocked, and there is no cap to refit over a bare needle.
- The pending insulin-pen task (a safety shot, then a dialled dose) works a pen's dial and button; it does not load
  or open the pen.
- No task on the board has a lancet, a lancing device or a fingerstick (searched titles, descriptions and reviews on
  the day of submission).

## Why it matters: help with blood-glucose testing

The task is submitted in the **Other** domain. Its case is assistive care for people with diabetes.

- **Scale.** 589 million adults were living with diabetes in 2024
  ([IDF Diabetes Atlas, 11th edition](https://idf.org/news-and-resources/news/idf-diabetes-atlas-11th-edition/)). In
  the United States the figure is 38.4 million people, and 16.5 million of them are aged 65 or over
  ([CDC, National Diabetes Statistics Report](https://www.cdc.gov/diabetes/php/data-research/index.html)). Between a
  quarter and a third of people living in long-term care have diabetes (American Diabetes Association position
  statement on long-term care, 2016, as reported by
  [U.S. Pharmacist](https://uspharmacist.com/article/diabetes-in-longtermcare-and-skilled-nursing-facilities-the-ada-position-statement)).
- **Every fingerstick check starts with this step, and it cannot be done in advance with someone else's device.** A
  pen lancing device belongs to one person. CDC's guidance for blood glucose monitoring reads "Never use fingerstick
  devices for more than one person" and "Never reuse lancets"
  ([CDC, infection control for blood glucose monitoring](https://www.cdc.gov/injection-safety/hcp/infection-control/index.html)).
  Among that page's references is a report of hepatitis B virus transmitted among people having their blood glucose
  monitored in long-term-care facilities (MMWR 2005;54:220-223). So whoever helps has to load a fresh lancet into
  the person's own device, at the person's side, each time.
- **Who needs the help.** Loading a lancet is small work with a needle in it: a 4 mm part into a 4.8 mm socket, a
  6 mm tab to twist off, a cap to refit over a bare point. People with diabetes and memory loss often cannot monitor
  their own glucose reliably by fingerstick, and their carers take the work on (reviewed in
  [Donat Ergin et al., 2025](https://ueaeprints.uea.ac.uk/103015/1/donat-ergin-et-al-2025-continuous-glucose-monitoring-in-older-people-with-diabetes-mellitus-and-cognitive-impairment-a.pdf)).
  Poor sight and numb fingers, both known complications of long-standing diabetes, and a tremor make the same step
  hard. No figure was found for how many people need help with this one step, and none is claimed here.
- **Who gives it now.** Family carers and home care aides. Nearly four million people work as home health or personal
  care aides in the United States (Bureau of Labor Statistics, as reported by
  [NBC News](https://www.nbcnews.com/business/economy/eldercare-workers-job-growth-pressure-rcna258820), February
  2026). Clinics and care homes that test many people use single-use lancets instead; the pen device in this task is
  the one a person uses at home.
- **What a robot that can do it changes.** With the device loaded and cocked for them, a person who can still press
  it to a fingertip keeps doing their own test, at the times their treatment needs, without waiting for a carer's
  visit.
- **The same skills elsewhere.** Pen injectors, insulin pen needles, twist-off ampoule and dropper tips, and
  spring-loaded tools that are cocked by pulling a sleeve all combine a held body, a small push fit and a twist-off.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: the layout (two device positions), the instruction, the device's own behaviour (`advance()`), what happened during the trial, scoring |
| `lancetkit.py` | Builds the device body, arming barrel, end cap, lancet, tab and dish as loose rigid bodies, and the four joints between them (cap snap, holder grip, twist-off neck, sprung slide) |
| `demo.py` | The scripted demo, and its control runs |
| `test_scoring.py` | Offline tests of the fits, the device's behaviour, the scoring rule and the layout (no simulator) |
| `task.json` | The submission's details |

## Requirements

The yam_arena embodiment and Isaac Lab-Arena, as set up by the program's GPU instance image (an NVIDIA RTX GPU is
needed). `test_scoring.py` needs only Python with numpy, pytest and the `yam_arena` package.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load the new lancet into the lancing device: take the device's end cap off, push the lancet into the holder, twist off the lancet's protective tab, put the end cap back on, then cock the device by pulling its arming barrel back until it clicks. Lay the device down on the table." \
  --scorer task_score --max-steps 2400
```

Run limit: 2,400 steps (2 minutes at 20 Hz). The demo reaches full score at step 1,729 (86 s) on both layouts.

The device lies in one of two places. Layout A is the default; set the environment variable `LANCET_LOT=B` for the
other (1.5 cm nearer the robot and 1.5 cm further left). The tray and the dish do not move.

## Scripted demo

```bash
python demo.py task.py out_dir                  # the demo (LANCET_LOT=B for the second layout)
python demo.py task.py out_dir --mode nohold    # control: pull at the cap with the device lying loose
python demo.py task.py out_dir --mode pull      # control: pull the seated lancet's tab straight back
python demo.py task.py out_dir --mode nocap     # control: leave the cap off and cock the device
python demo.py task.py out_dir --mode still     # control: the arms do not move
```

1. The left hand comes straight down on the device's body, lifts the device 11 cm off the table and holds it level,
   its nose toward the right arm. It holds it through steps 2 to 5.
2. The right hand, leaning 30 degrees out to its own side, takes the end cap between its fingers, draws it straight
   off the nose and lays it in the kit tray's cradle.
3. It takes the new lancet by its protective tab, brings it level to the nose and pushes it into the holder until the
   holder grips it; then it lets go.
4. It goes round to a level pose pointing at the nose, takes the tab in its fingertips and turns its wrist 100
   degrees, which takes the tab off; it drops the tab in the dish.
5. Leaning again, it takes the cap from the cradle, brings it over the bare needle and the nose and pushes it on until
   it snaps.
6. The left hand lays the device on the table and lets go. The right hand holds the body near the cap, the left takes
   the arming barrel and pulls it back until the device is cocked, and both let go.

The demo uses only the layout constants in `task.py`, the sizes in `lancetkit.py` and each hand's own measured position
(proprioception): to correct an arm's sag before a grasp, and to find the device's nose from where the other hand is
actually holding it. It never reads a part's state.

## Scoring

Tools: the trial video (a close view of the device), a ruler, a protractor or level.

- **Lancet home:** the lancet's insertion end is within 1 mm of the socket's floor, inside the socket and within 10
  degrees of in line, at which the holder grips it. It counts as out again if it is drawn 3 mm back. Real trial:
  the lancet stays in the holder when the device is tipped nose down, and 15.5 mm of it stands out of the nose.
- **Tab off:** the protective tab's neck has let go: the tab has been twisted 45 degrees or more about the lancet's
  axis, or forced more than 4 mm or 20 degrees out of its place on the lancet. Real trial: the tab is a separate
  piece.
- **Cap on:** the end cap's mouth is within 1 mm of the body's shoulder, round the nose and within 10 degrees of in
  line, at which it snaps on. It counts as off again if it is drawn 3 mm back. Real trial: no gap wider than 1 mm
  between cap and body, and the cap stays on when the device is held by the cap.
- **Cocked:** the arming barrel has been pulled back 6.5 mm or more of its 8 mm travel (the device's click). Real
  trial: the device's own click, or its trigger button's cocked indicator.
- **Needle touched:** while the tab is off (taken 10 mm clear of where it joined the lancet) and the end cap is not
  snapped on the device, any part of a gripper comes within 2 mm of the needle. Real trial: from the close video, with
  the 4 mm lancet body in view as the scale.
- **Laid down and let go:** at the end the device lies on the table within 5 degrees of level with its underside
  within 2 mm of the surface; the device, cap, barrel and lancet are at rest (under 2 cm/s); and no part of either
  gripper is within 3 cm of the device.

| Outcome | Score |
|---|---|
| The lancet is not home; or the needle was touched; or the device went off the table | **0** |
| Otherwise, the tab is still on | **0.25** |
| Otherwise, the cap is not on | **0.5** |
| Otherwise, cocked, laid down and let go | **1.0** |
| Otherwise (capped, but not cocked or not laid down and let go) | **0.75** |

Every outcome falls into exactly one row (the first that applies). The order the steps were done in does not matter;
only the device's final state and the needle. In code: `advance()`, `_watch()`, `score_from()` and `score()` in
`task.py`; the fits are `seat_fit()`, `cap_fit()`, `twist()` and `barrel_pull()` in `lancetkit.py`.

## Verification

All runs use the submitted scoring code.

| Run | Score | Notes |
|---|---|---|
| Scripted demo, layout A | 1.0 | Cap off at step 300; lancet home at step 685 (8.1 mm into the 9 mm socket); tab off at step 920 and dropped in the dish; cap snapped on at step 1,365 (0.7 mm from fully home); cocked at step 1,640 (barrel pulled 7.5 mm). While the needle was bare no part of a gripper came within 7.9 mm of it. Full score at step 1,729 (86 s) |
| Scripted demo, layout B | 1.0 | The same steps: lancet 8.5 mm in, cap 0.9 mm from fully home, barrel pulled 7.4 mm, nearest approach to the bare needle 5.2 mm. Full score at step 1,729 |
| No holding hand (control) | 0.0 | The right hand took the cap of the device lying loose and pulled: the device was dragged 34 mm across the table with its cap still on |
| Tab pulled, not twisted (control) | 0.25 | With the lancet home and the device held, the right hand pulled the tab straight back 30 mm: its fingers slid off, the tab stayed on and the lancet stayed home. That is the score for a seated lancet alone; pulling earned nothing |
| Cap left off (control) | 0.5 | Lancet home, tab off, then the device laid down and cocked (barrel pulled 7.4 mm, step 1,320) with its needle bare: no credit for cocking without the cap |
| Arms still (control) | 0.0 | Nothing moves |
| Dream-1, 2,400 steps, layout A | 0.0 | The left gripper picked the device up by its body and held it in the air; the right gripper came up to the cap end and its fingers reached round the cap, but the cap was not drawn off. The device was put back on the table with its cap on, and the right gripper ended over the kit tray. The lancet was not taken |
| `pytest test_scoring.py` | 20 passed | The fits (lancet home, cap on, barrel pull, twist), the device's behaviour step by step (grip made once, tab off by twist or by force, cap snap, cocking latch, the bare-needle rule), the score ladder and its zero cases, the two-hand and twist-not-pull numbers, and both layouts |

The videos show the overhead camera, the right wrist camera and the left wrist camera side by side.

What we learned building the demo, which shows what the task asks of a policy:

| Development run | What happened |
|---|---|
| Lancet carried by its tab, neck with no spring-back | The tab crept 20 degrees round on its neck from the handling alone, and the neck let go before the lancet reached the device. The neck now springs back from small twists and yields at a set torque (see Fidelity) |
| Lancet pushed at a socket with a square mouth | Its end stopped 1.3 mm in, and the lancet fell to the table when the fingers let go. The mouth is now rounded and slick (see Fidelity) |
| Cap pushed back on over a square-edged nose | The cap stopped 10 mm short at the nose's front edge while the push slid the device 4 to 6 mm through the holding fingers. The edge is now rounded and the bore widened (see Fidelity) |
| Barrel pulled its 8 mm of travel | Both arms gave way under the spring and the barrel moved only 5.5 mm, short of the click. The pull is now commanded 9 mm further and made over 2 s |

## Runtime and seeds

About 6 minutes per recorded demo run and about 7.5 minutes per Dream-1 run on one L40S, including under a minute of
start-up. `object_jitter=0`; each layout is fixed, and there is no randomness in the task.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table (x forward, away from the robot; y to the
left).

- **Lancing device:** a pen-shaped lancing device, 13.4 cm long with its cap on, lying along y with its nose toward the
  right arm; the tip of its nose at x 34, y 5 (layout A) or x 32.5, y 6.5 (layout B). In all it weighs 29 g.
  - Body: blue plastic, 17 mm across, 7.8 cm long, 22 g, with an 8 mm flat that it lies on and a yellow trigger button
    on top. At its front a dark grey nose 9 mm across and 12 mm long holds the lancet: a round socket 4.8 mm across
    and 9 mm deep, its mouth and the nose's front edge rounded.
  - End cap: pale grey plastic, 17 mm across and 3.4 cm long, 3 g. It fits over the nose and snaps against the body's
    shoulder; the snap lets go at a 2 N pull. It starts on.
  - Arming barrel: dark grey plastic, 17 mm across and 2.2 cm long, 4 g, on the body's rear end. It slides back 8 mm against a
    return spring (200 N/m, 1.6 N at full travel); the device is cocked once it has been pulled back 6.5 mm.
- **Lancet:** one twist-top lancet, 3.25 cm long with its tab: a white round plastic body 4 mm across and 2.45 cm long
  with a rounded insertion end, a needle 3 mm long at the other end, and over the needle a white protective tab 6.4 mm
  across and 8 mm long on a moulded neck. 0.6 g. It lies along y on the kit tray's ridge, its insertion end toward
  the device at x 21.8, y -14.8, its axis 3.2 cm above the table.
- **Kit tray:** a grey plastic block 7 x 11 x 2 cm, fixed to the table, its middle at x 24, y -16. On it a ridge 8 mm
  wide, 1.9 cm long and 1 cm tall with two 1.5 mm kerbs holds the lancet, its tab overhanging the ridge's end; two
  rails 6 cm long, 2.5 mm tall and 1.1 cm apart form a cradle where the cap can be laid.
- **Dish:** an open white plastic dish, 7 cm across inside and 2 cm tall, 30 g, loose, base centred at x 44, y -1,
  for the tab.
- **Robot:** both arms start in the raised ready pose with their grippers open.

## Fidelity

- **Loose rigid bodies and four joints.** Body, barrel, cap, lancet and tab are free rigid bodies. The barrel rides a
  slide with stops and a return spring. The cap's snap and the holder's grip are joints made when the part reaches
  home (within 1 mm) and broken by a pull of 2 N and 2.5 N. The tab's neck is a spring about the lancet's axis that
  yields at 0.004 N m; the task lets it go once the tab has been twisted 45 degrees or forced 4 mm or 20 degrees out
  of place (a joint between parts this light gives a little under load, so the limits sit outside that give), and it
  breaks at an 8 N pull.
- **The forces are estimates, not measurements.** The cap's snap, the holder's grip, the neck's torque and the
  barrel's spring are set by judgement and to what this robot can apply (about 3 N); a real device should be measured
  before a real trial, and its barrel may need more than 1.6 N.
- **Simplified shapes.** The lancet's body is round with a rounded end (the common one has a square end that has to
  be turned to line up). The cap pulls straight off (many twist a little first). The cap's bore is 2.2 mm larger than
  the nose, the nose's edges are rounded and the nose is slick (friction 0.2), where a real cap is a close fit with a
  chamfer. The needle is drawn 0.8 mm thick (a 30G needle is 0.3 mm) and has no collider: touching it is judged by
  distance. The depth dial is drawn only, and the device is not fired.
- **Sizes.** The lancet's 32.5 mm length and 6.4 mm tab follow a listed 30G twist lancet (32.6 x 6.4 mm); the device's
  sizes are estimates typical of pen lancing devices, not taken from one model.

## Real-world build

- **Parts:** a pen lancing device with a pull-off (or quarter-turn) end cap and a pull-back arming barrel, and
  twist-top lancets, as sold together for home glucose testing; a small tray with a groove for the lancet and one for
  the cap; a dish. Tape marks for the device's place.
- **Each trial:** a new lancet in the tray, the device capped, uncocked and empty on its marks.
- **Scoring:** film the device close up. Check that the lancet stays in when the device is tipped, that the tab is
  off, that the cap is fully on, the cocked indicator, and where the grippers were while the needle was bare.
- **Safety:** lancets are sterile, single-use and needle-pointed. Use a new one for each trial, do not fire the
  device, and put used lancets in a sharps container. No person holds the device during a trial.
- **Protecting the grippers:** in a failed trial a bare needle can meet a soft gripper pad and puncture it. Either
  blunt the needle of each trial lancet beforehand (file its point flat; the twist-off tab, the body and the fits
  are unchanged) or fit thin puncture-resistant covers over the pads. Either way, a gripper touching the needle is
  still scored from the close-up video, as above.

## Third-party assets

None. Every part is built from primitive shapes in `lancetkit.py`.
