# Open a hanging locker padlock by its dial

**Instruction:** "Open the padlock on the locker with the combination 24-8-31, and leave it hanging open on the locker's
hasp."

A dial combination padlock hangs free on the hasp of a steel locker door. The maker's directions for this lock type:
turn the dial right three times and stop at the first number; turn left one full turn, passing the first number, and
stop at the second; turn right and stop at the third; pull. The numbers are printed on the black dial and read
against the red index on the body above it. Because the lock hangs free, the hand on the dial's 18 mm knob also drags
and twists the body, and a gripper's wrist turns at most 240 degrees, so the dial is turned in several
grips with re-grasps and two reversals of direction. Each number has to stop on its mark at the index, within half a
mark (4.5 degrees): the index must read that number and not a neighbour.

## Running a trial

```
inspect-robots run --embodiment yam_arena -E task=open_locker_combination_padlock.py \
    --instruction "Open the padlock on the locker with the combination 24-8-31, and leave it hanging open on the locker's hasp." \
    --policy yam_scripted --scorer task_score --max-steps 5000
```

- Scene: `station`. Control at 20 Hz. Run limit: 5,000 steps (250 s). The scripted demo (`demo()`) uses the right
  arm only: it pinches the dial's knurled knob with the gripper pointing straight at the dial and turns it with the wrist roll,
  re-gripping when the wrist runs out of travel (4 + 1 grips right, 2 + 1 left, 1 right), then pulls the body down by
  the knob. Each grip turns the dial a few degrees less than the wrist rolls, and by a varying amount, so the last grip
  for each number is closed-loop on the angles read from the simulator (as a scripted demo may), never reversing: a
  fast approach in 0.1 s steps that stops about 4 degrees short, a pause, then a slow trim in 0.25 s steps of at most
  0.5 degrees onto the number; while a wheel is being carried it steers on that wheel's own gate. Before letting go of
  the knob for numbers 1 and 2 the wrist eases back 8 degrees so the opening pads cannot flick the dial past the number. It reaches success
  at about step 2,000 (100 s). It prints the dial and gate angles once per control step (`PADLOG` lines). A trial takes
  10 to 30 minutes on a shared GPU.
- No object jitter; the lock always starts at dial 0 with the pack cleared to the left (the reset: spin left three or more
  turns, stop at 0), so the seed changes nothing.
- `--scorer task_score` gives 1.0 on success and otherwise the score at the end (`SCORE_OVER = "end"`).
- Offline checks: `pytest test_open_locker_combination_padlock.py` (generated scorer tests from start, goal, partial
  and cheat layouts, and the demo's reach).

## Scoring

The scorer reads the lock's joint angles (dial, wheels, shackle) and pose; the release check reads the robot's joint
state to find each gripper's centre point between the pads.

| Rule | Value |
|---|---|
| Gate set | That wheel's (or the cam's) gate within 4.5 degrees (half a mark) of the fence: the dial was stopped on the number, not between it and a neighbour. The fence physically reaches release depth only when all three are within 4.0 degrees, so a stop one mark off on any number keeps the lock shut. |
| Set positions | wheel 1 at 185.2 deg (24 dialled right, both pins pushing: -9 x 24 + 2 x 20.6); wheel 2 at 267.4 deg (8 dialled left: -9 x 8 - 20.6); cam at 81 deg (31 dialled right). Angles about +x, mod 360. |
| Open | Shackle at least 8 mm up from closed, relative to the body (short leg out of the body). |
| On the hasp | The top of the shackle's arch within 12 mm of the hole centre in x, y and z. |
| Released, at rest | Each gripper's centre point at least 30 mm from the lock's bounding box; over the last 1 s the body centre stayed within a 5 mm box and its tilt within 5 deg (a hanging lock keeps a small sway on the hasp). |
| 1.0 | Open, on the hasp and released for 1 s, at rest over that second. |
| 0.75 | All three gates set but not opened, or opened but still held, moving, or lifted off the hasp without falling. |
| 0.5 / 0.25 | Wheels 1 and 2 set / wheel 1 set. |
| Sticky zero | Lock body ever more than 35 mm below where it hung (off the hasp); shackle coming open while any gate is more than one mark off (forced; judged at the moment it first opens, as the open lock's dial turns freely). |
| Success | 1.0 held for 1 s (`SUCCESS_HOLD_S`). |

Doing nothing, turning right straight to 24 without the full turns, spinning the dial one way only, and pulling with a
wrong combination all score 0 (the last moves the shackle under 2 mm).

**Measuring it in a real trial:** place a camera about 150 mm from the lock, 40 degrees to the side of the gripper's
line, so the dial numbers and the index stay legible past the gripper. From the video, note each stop's number and the
direction turned into it; the maker's rule then says which wheels are set (wheel 1 by the right-hand turns past two
full turns into 24; wheel 2 by the left turn passing 24 once into 8; the cam by the final right turn into 31). A stop
counts only if the index sits on the number to within half a mark (read it against the marks either side). For 0.75, the judge pulls the body down by hand at
the end: it opens only if all three gates are set. The last second of video shows whether the lock is at rest; the end
photo shows the shackle open on the hasp and the grippers at least 30 mm away. Measure the opening pull once with a
spring scale hooked under the body (expected about 3 N beyond the lock's weight).

## Scene and dimensions

World frame: x forward from the line through the arm bases, y left, z up from the station table top (z = 0.7475 in
the simulator). Lock frame: origin on the dial axis at the body's mid-thickness, axes along the world axes, dial facing
-x. Lock shape and print follow the maker's product photographs and published dimensions
(https://www.masterlock.com/products/product/1500D: 48 mm body, 7 mm shackle, 19 mm shackle length, 21 mm inside
width); the overall height, 74 mm, matches retail listings (2.9 in), and the 19 mm body thickness one retailer's 3/4 in. Sizes read off the photographs (scaled by
the 48 mm body) are marked "photo".

| Item | Value |
|---|---|
| Door section (fixed) | 1 mm painted steel face, 300 x 250 mm (y -0.210..+0.090, z 0..0.250), face at x 0.2545; 25 mm side returns; 90 mm steel foot behind |
| Hasp tab | 3 mm steel in the plane y = -0.060, x 0.2295..0.2545 (25 mm out), z 0.185..0.207; 10 mm hole, axis along y, centre x 0.2395, bottom z 0.191 |
| Lock body | round, 48 mm diameter, 19 mm thick (retail listing), stainless cover on a steel body; centre (0.2395, -0.060, 0.148), 3 mm above the dial axis (photo); 112 g |
| Dial | black disc 37 mm diameter (photo), 1 mm proud of the body face (x 0.229..0.230), centred at (0.2395, -0.060, 0.145); knurled steel knob 18 mm diameter (photo) standing 6 mm on the disc with a 1 mm black cap (knob height an estimate: one retailer lists 33 mm overall depth, 27 mm here) and a white zero mark (front at x 0.222); with its drive cam 12 g |
| Dial print | white, as on the maker's dial: a tick at each of the 40 numbers (0.7 mm wide, from 16.3 to 18.1 mm radius, every fifth from 15.0 mm), numerals 0, 5 ... 35 centred at 12.6 mm radius, 3.6 mm tall with 0.75 mm strokes (the real ones measure about 3 mm in the photo; drawn about 20% taller so they stay legible in the 320 x 176 wrist camera), standing on the radius with their tops toward the rim; numbers rise clockwise seen from the front; red triangular index on the body at 12 o'clock. The print is 0.15 mm thick and has no collision |
| Shackle | 7 mm hardened steel: two legs and a semicircular arch of 14 mm centre-line radius (11 straight segments), 19 mm inside clearance above the body's top, 21 mm inside width (leg centres +/-14 mm); short leg ends 4.5 mm inside the body's curve; travel 12 mm; 30 g |
| Wheel pack | cam disc x -8..-6 mm, wheel 2 x -2..0, wheel 1 x 4..6 (lock frame); rims 12 mm radius, hubs 9 mm; gates 14.7 deg wide with radial sides, 3 mm deep; pins 2 x 3 mm at radius 5.5..8.5 mm (pick-up after 318.8 deg of free play); wheels 3 g each |
| Bolt and fence | slides along y, 3.5 mm travel; fence 1.2 mm thick at the rims' +y side, 0.8 mm outside the rims when idle; it must go 2.6 mm in (to 10.2 mm radius) before the nose clears the 45-degree catch at the short leg's end, which it can only do with every gate within 4.0 deg; 3 g |
| Springs | shackle 50 N/m, 1.5 N preload (up); bolt 300 N/m, 4.9 N preload (toward the catch) |
| Friction | dial drag 0.008 N m static (0.006 dynamic); each wheel 0.002 N m; steel on steel 0.3; knurled knob 1.0 (the knurl is a friction value, not modelled ridges); catch and bolt 0.1 |
| Total lock mass | 163 g |

Forces: hanging, the catch carries the body's weight (about 1.35 N) plus the shackle spring (1.5 N), under the 6 N
that would cam the bolt in. With the gates set, a pull of about 3 N more cams the bolt into the gates and the shackle
springs up 12 mm (the body drops 12 mm on the hasp).

## Real-world parts

- A 1-7/8 in (48 mm) dial combination padlock, Master Lock No. 1500D (published: 48 mm body, 7 mm shackle, 19 mm
  shackle length, 21 mm inside width; about 165-175 g in retail listings). The simulated lock opens at 24-8-31. A
  real 1500D's combination is fixed at the factory, so a real trial uses the purchased lock's own combination: change
  the numbers in the instruction (and in the scoring of stops) to that combination. Prefer a lock whose second number
  is 10 to 30 marks from the first, as here (the left turn is then between 1.25 and 1.75 turns).
- Reset before every real trial: close the shackle, spin the dial left at least three full turns and stop at 0, hang
  the lock on the hasp with the long leg on the -y side, and let it come to rest.
- A real No. 1500 may open with a stop a little outside half a mark (see below), so a real trial judges the stops
  from the dial video against the half-mark rule, not from whether the lock opens.
- A single-tier steel locker door lock area, or a 1 mm steel plate with a welded 3 mm hasp tab as above, clamped
  upright to the table.

## How precise the stops must be

The maker's directions for this lock type say to stop *on* each number: "Turn the dial 3 times to the right and stop on
the first number of the sequence ... stop on the second number ... stop on the third number"
(Master Lock padlock FAQ, https://www.masterlock.eu/service-and-support/faqs/padlocks). The maker publishes no
tolerance figure. Owners report that worn or loose real locks forgive more: a dial "off by half a mark ... will almost
certainly still work" (locksmithing discussion forum, 2006, https://www.polytechforum.com/locksmithing/suggested-update-to-faq-wrt-master-lock-hack-1544573-.htm),
and one owner wrote of a No. 1500: "Each stop was about 1 number off of the actual combo but still opened" (Lock Picking 101 forum, 2011,
https://www.lockpicking101.com/viewtopic.php?p=380391). The task follows the maker's rule, read as strictly as a
person can read a dial: each number counts only within half a mark (4.5 degrees) of the index, so the index reads
that number and no neighbour. This is a choice made from the maker's directions, not a measured property of a
particular lock; the simulated gates admit the fence within 4.0 degrees, inside that half mark, so a stop one whole
mark off on any number keeps the simulated lock shut.

## Assets and licences

Every object is built from primitives and procedural convex meshes defined in the task file, with our own kit
modules (`yam_parts.py`, `yam_joints.py`, `yam_mesh.py`). No third-party meshes, textures, scans or fonts.
