# Pinch-zoom a street map on a loose tablet until the whole red walking route fits on the screen

**Instruction:** "Zoom the map out until the whole red route fits on the screen, filling at least half its width, then lift your fingers off."

A 10-inch tablet (Amazon Fire HD 10, 2023) lies loose and screen up on the desk. Its screen shows a street map with a red
walking route, zoomed in so far that only the middle of the route is visible. The robot has to pinch the map out with
two fingertips on the glass until the whole route is on the screen and fills at least half of it, without dragging the
map away, tapping a place pin open or knocking the tablet, and then take its hand away.

The screen is live in the simulation. Every control step the task reads where the robot's four fingertips are, decides
which of them touch the glass, runs the map viewer's own touch rules on those touches and redraws the map, the route,
the pins and any open place card on the screen (one drawn-only mesh per map layer under the tablet's body, clipped to
the screen's edges). The cameras see the map zoom and pan as the fingers move.

## Running a trial

```
inspect-robots run --embodiment yam_arena -E task=pinch_zoom_map_on_tablet_to_fit_route.py \
    --instruction "Zoom the map out until the whole red route fits on the screen, filling at least half its width, then lift your fingers off." \
    --policy yam_scripted --scorer task_score --max-steps 800
```

- Scene: `station`. Control at 20 Hz. Run limit 800 steps (40 s). The scripted demo takes 14.9 s (298 steps) and
  succeeds at step 222. A demo trial takes about 1-2 minutes on a shared GPU.
- No object jitter; one fixed layout; the seed changes nothing.
- `--scorer task_score`: 1.0 on success, otherwise the score at the end (`SCORE_OVER = "end"`).
- Offline checks: `pytest test_pinch_zoom_map_on_tablet_to_fit_route.py` (scorer on goal, partial and cheat layouts).
- Every step with a touch, and every 10th step, the console prints a `PZLOG` line (zoom, map centre, route box on the
  screen, touches, fingertip heights above the glass, card, tablet pose) and every touch event as a `[viewer]` line.

## The map viewer (simulated and real)

The real trial runs `map.html` from this folder in the tablet's browser. It loads **Leaflet 1.9.4**
(https://leafletjs.com, BSD-2-Clause, https://github.com/Leaflet/Leaflet/blob/v1.9.4/LICENSE) from unpkg, pinned by
Subresource Integrity, and draws our own synthetic street plan (no map tiles, no network map data; the data is in the
page and is generated from the same constants as the simulated screen). Map units are millimetres on the screen at
the start view (`L.CRS.Simple`, documented as "a simple CRS that maps longitude and latitude into x and y directly").

Map options (quotes from the Leaflet 1.9.4 reference, https://leafletjs.com/reference-1.9.4.html):

| Option | Value | Why |
|---|---|---|
| `touchZoom` | `true` | "Whether the map can be zoomed by touch-dragging with two fingers." Not `'center'`, so it zooms about the fingers. |
| `zoomSnap` | `0` | "A value of 0 means the zoom level will not be snapped after fitBounds or a pinch-zoom." |
| `inertia` | `false` | no momentum after a pan |
| `zoomAnimation`, `fadeAnimation`, `markerZoomAnimation` | `false` | no animated zoom |
| `bounceAtZoomLimits` | `false` | "Set it to false if you don't want the map to zoom beyond min/max zoom and then bounce back when pinch-zooming." |
| `minZoom`, `maxZoom` | start zoom -3 and +2 | the map shows 1/8x to 4x of the start scale |
| `doubleClickZoom` | `true` (default) | "zoomed in by double clicking"; `zoomDelta` 1 = 2x |
| `dragging` | `true` (default) | one-finger pan |
| `closePopupOnClick` | `true` (default) | a tap on the map closes a place card |
| zoom control, keyboard, scroll wheel, box zoom, attribution | off | nothing else on the screen |
| popups | `autoPan: false`, `closeButton: false` | opening a card does not move the map |

Leaflet has no map rotation (the 1.9.4 reference has no rotation option), so rotation is off by construction.
The page sets `touch-action: none` and `user-scalable=no` so the browser itself never zooms the page.

**Start view and reset.** The page computes the start zoom from the screen width so that one map unit is one
millimetre on the screen (`zoom = log2(innerWidth / 217.6)`), centres the map at (0, 0), and closes any card. Reloading
the page is the whole reset. Before a trial, tap its overlay once: the page goes full screen (Fullscreen API) and
rebuilds the start view at full-screen size. Check `window.devicePixelRatio` and `innerWidth` once on the tablet; the
start view does not depend on them, but the touch slop in millimetres does (below).

**What the simulation does with touches** (Leaflet source: https://unpkg.com/leaflet@1.9.4/dist/leaflet-src.js,
`Draggable`, `Map.TouchZoom`, `DomEvent.DoubleTap`, `Map.DoubleClickZoom`):

1. One touch alone: the browser holds its moves back until it has gone the touch slop from where it landed (Android
   `TOUCH_SLOP` = 8 dp, taken as 8 CSS px = 1.36 mm); after that the map follows the finger's whole offset (a pan).
2. A second touch landing ends any pan (`Draggable._onDown` finishes the drag when `touches.length !== 1`) and starts a
   pinch: the zoom is the start zoom times (current spacing / start spacing), and the map point that was under the
   midpoint of the two touches at the start stays under their current midpoint (`TouchZoom._onTouchMove`). The pinch
   ends when either finger lifts; the finger left on the glass does nothing until it lifts too.
3. A touch that never left the slop and never overlapped another touch is a tap (a click). A tap on a pin opens its
   card; a tap elsewhere on the map closes it; a tap on the open card does nothing.
4. Two taps within 0.3 s and 17 mm (Android `DOUBLE_TAP_TIMEOUT` 300 ms, `DOUBLE_TAP_SLOP` 100 dp; Leaflet's own
   double-tap simulation uses 200 ms) zoom in 2x about the second tap.
5. A touch landing within 3 mm of a screen edge counts as an edge swipe (scored zero: on Fire OS it can bring up the
   system bars and leave the page).

**Checked against the real library.** We drove `map.html` in Chromium with emulated touches (Chrome DevTools
`Input.dispatchTouchEvent`, 1280 x 800 viewport) and compared Leaflet's resulting view with the task's rules:

| Gesture | Leaflet in Chromium | Task's rules |
|---|---|---|
| the demo's pinch: 90 -> 25 mm about (-8, 0) mm | zoom 0.2778x, centre (20.80, 0.00) | 0.2778x, (20.8, 0.0) |
| pan 20 mm right, 10 mm up | centre (-20, -10) | (-20, -10) |
| uneven pinch with a moving midpoint, 80 -> 30 mm | 0.375x, centre (40.00, -43.33) | 0.375x, (40.0, -43.3) |
| tap on the cafe pin | card opens | card opens |
| tap on the map with a card open | card closes | card closes |
| double tap 20 mm below centre | 2x, centre (0, -9.95) | 2x, (0, -10) |
| touch on a pin drifting 1.5 or 2.5 mm, then up | card opens, no pan | 1.5 mm: tap; 2.5 mm: pan |
| touch on a pin drifting 4 or 8 mm | pans 4 / 8 mm, no card | pans, no card |
| touch held 0.7 s on a pin | card opens | card opens (no long-press rule) |

Desktop Chromium's touch slop came out between 2.5 and 4 mm at this size (15-23 CSS px); Android's documented slop is
8 dp. The task uses the Android figure, which is the stricter of the two. Untested on the Fire tablet itself.

## Where the pins are

The pins are 7 mm discs; a tap counts on a pin when it lands within its 7 x 7 mm square. At the start view the cafe pin
is at (-60, +37) mm from the screen centre (right and up positive), the museum at (+70, -40) mm and the pharmacy off
the screen at (+160, +110) mm. The demo's fingertips land at (-53, 0) and (+37, 0) mm and slide along the screen's middle
line, so the pins are 37 mm and 52 mm from where they land; only a fingertip that lands alone, near a pin, and lifts
without sliding opens a card. Pins move with the map as it zooms (they keep their screen size).

## Real-trial touch hardware (required)

A capacitive screen registers a conductor that disturbs its electrode field and gives the charge a path to a large
capacitance (a finger does this through the body, about 100-200 pF in Microchip's human body model). Bare robot
fingertips will not register reliably. Each of the two pads' end faces (about 15 x 4 mm) must carry a soft conductive
tip connected to ground:

- conductive fabric tape wrapped over each end face (for example Adafruit 3960, conductive nylon fabric tape,
  8 mm x 10 m, about US$4: https://www.adafruit.com/product/3960), or a 6 mm conductive fibre or rubber stylus
  replacement nib glued to each face (about US$7-17 for a pack);
- a thin wire from each tip to the robot's grounded metal frame or to mains earth (for example an alligator-clip lead,
  Adafruit 4100, about US$3 for six: https://www.adafruit.com/product/4100).

Grounding a robot-held passive stylus is the fix reported by Evil Mad Scientist for their drawing robot
(https://www.evilmadscientist.com/?p=30745: a passive stylus may not register on its own; a grounded wire touching the
stylus makes it work, with the caveat that "your particular screen and stylus may not behave the same way").
Commercial touch-test robots use 7 mm conductive tips (Microsoft's touch test jigs use "7mm slugs":
https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/display-touch). Not yet verified on this
robot: before the trial, check with a touch-test app that each tip alone registers a touch, a drag and a release, and
that the two tips 25 mm apart register as two touches (projected-capacitive panels resolve two touches about
12-15 mm apart centre to centre; the Windows touch requirement is 8 mm edge to edge).

## Does the other hand need to steady the tablet?

No, and the task does not ask for it. In every simulated run the tablet moved less than 0.1 mm and turned less than
0.01 degrees while one hand pinched: the two fingertips slide toward each other at the same speed, so their friction on
the glass cancels, and a light press (about 2 N) adds to the tablet's own grip on the desk (0.4 x 4.3 N). A control run
pressing 15 mm past contact (about 8 N, the most this arm gives) pinched the same way, scored 1.0 and did not move the
tablet either. Two hands are allowed but not needed.

## How hard to press

The arm is compliant, so the press is set by how far past the glass the fingertips are commanded. At 4 mm past (the
demo) and at 15 mm past both fingertips stayed on the glass (end faces 0.04-0.11 mm above it in the simulator) through
the whole slide. Commanded exactly to the glass, they hovered 0.3-0.7 mm above it while sliding (the arm sags and the
fingers ride up), the screen registered no touch at all and the trial scored 0. The fingertips have to be pressed onto
the glass and kept there while the fingers close.

## Scoring

The scorer reads the viewer's state (zoom, map centre, open card, pinches, edge swipes), the tablet's pose, and the
arms' joint readings (to find the fingertips).

| Rule | Value |
|---|---|
| Touch | a fingertip's end face on the glass (within 0.2 mm; pressed faces rest 0.04-0.11 mm above it in the simulator), over the picture (217.6 x 136.0 mm) |
| FITS | the route's bounding box wholly on the screen at least 5 mm inside every edge, and at least 108.8 mm (half the screen) wide: a zoom of 0.18x-0.35x about the right place |
| PINCH | two touches on the glass together whose spacing fell to 75 % or less of its start |
| SETTLED, for 1 s | no touch; every fingertip face 20 mm or more above the glass or 10 mm or more outside the tablet's outline; the tablet moving under 0.5 mm per 50 ms step |
| 1.0 | settled, FITS, no card open, tablet within 20 mm of its start, turned under 5 degrees, flat within 3 degrees (success: held 1 s more) |
| 0.75 | settled and FITS, but a card open or the tablet moved or turned |
| 0.5 | settled, zoomed out to 0.5x or less, not FITS (zoomed out too far is here: at the 1/8x limit the route is 75 mm wide) |
| 0.25 | a PINCH happened at any time |
| 0 | anything else, including doing nothing, a fingertip still on the glass or hovering |
| Sticky zeros | the tablet turned over (tilt over 45 degrees) or off the desk (20 mm below its top); a touch landing within 3 mm of a screen edge; in a real trial also the screen cracked |

The map only zooms out by pinching (no zoom buttons; a double tap zooms in), so every score of 0.5 or more needs a
pinch. Over-zooming (out past 0.18x) leaves the route too small and scores 0.5 until the robot spreads its fingers or
double-taps to zoom back in.

**Measuring it in a real trial:**
1. Record the tablet's screen (Android screen recording) and an overhead video, started together.
2. Mark the tablet's outline on the desk with tape before the trial.
3. At the end, with the hand clear and everything still for 1 s, take the last screen frame: measure the route's
   bounding box in pixels (1 px = 0.1133 mm). FITS when it is at least 960 px wide and at least 44 px inside every edge.
4. Check no place card is open, and the tablet against its outline with a ruler (20 mm) and a protractor (5 degrees).
5. From the screen recording: a pinch is any stretch with two touch points whose spacing shrank by a quarter (turn on
   Android's "Show taps" developer option); an edge swipe is a touch starting in the outer 3 mm (26 px) of the screen.

## The scripted demo

`demo()` is open loop (the start layout and time). The right hand opens its fingertips 90 mm apart (centre to centre),
yaw 0 so they close along the screen's long axis, comes down over a point 8 mm left of the screen centre, sets both
fingertips on the glass and presses about 4 mm past contact (roughly 2 N on this compliant arm), closes them to 25 mm
apart in 2.4 s while they slide on the glass, lifts straight off and moves away. The zoom ends at 0.278x with the route
centred. The left hand only moves out of the cameras' way. In the final GPU run the left fingertip touched first and the
right one 0.65 s later (the gripper is not perfectly level: the faces were 0.6 mm apart in height as the first one
landed); the first one stayed within the touch slop meanwhile, so the viewer saw a pinch, not a pan. Both stayed on the
glass through the whole slide.

## Layout (metres, world frame; arm bases at (0, +/-0.305, 0.750); table top 0.7475)

| Object | Type | Material | Size | Mass | Pose |
|---|---|---|---|---|---|
| Tablet: Amazon Fire HD 10 (2023, 13th generation) | free rigid body: box with 10 mm rounded corners | plastic back, glass front (one material, friction 0.4 / 0.35) | 246 x 164.8 x 8.6 mm | 0.434 kg | centre (0.200, -0.060, 0.7475 underside), long side along y |
| Screen picture | drawn-only meshes 0.1-0.65 mm above the glass | lit display | 217.6 x 136.0 mm (1920 x 1200 at 224 ppi) | none | centred on the tablet's front |
| Route | drawn polyline, 3 mm wide at every zoom | red | box 600 x 300 mm at the start view | - | box centre 20 mm right of the screen centre |
| Pins (3), route ends (2) | drawn 7 mm discs with a white dot | blue / green / dark | 7 mm | - | see "Where the pins are" |
| Place card | drawn rectangle with a blue title bar | white | 44 x 24 mm, its bottom 5 mm above the pin | - | above its pin, shown only when open |

Map plan (map units = mm at the start view): streets every 80 units across and 75 units up, 9 units wide; four main
roads 16 wide and one diagonal avenue; a river 70 wide; four parks. Route corners: (-280, -150), (-120, -150),
(-120, -75), (40, -75), (40, 75), (200, 75), (200, 150), (320, 150).

Sources for the tablet: Amazon developer device specifications (screen 10.1 in, 1920 x 1200, 224 ppi, 5-point
multi-touch, Fire OS 8 on Android 11):
https://developer.amazon.com/docs/device-specs/ft-device-specifications-firehd-models.html; size and weight
(246 x 165 x 8.6 mm, 434 g): https://www.gsmarena.com/the_new_amazon_fire_hd_10_tablet_is_now_up_for_sale_starting_at_140-news-60315.php.
Android touch constants: https://android.googlesource.com/platform/frameworks/base/+/refs/tags/android-11.0.0_r1/core/java/android/view/ViewConfiguration.java.

Estimates: the friction values, the corner radius, the CSS pixel size (0.17 mm: devicePixelRatio 1.5, from Amazon's
"hdpi" class for the earlier Fire HD 10 with the same panel; check on the device) and the touch slop on the Fire tablet's browser.

## Real-world parts

- Amazon Fire HD 10 tablet (2023, 13th generation), no case, about US$140; any 10-inch 1920 x 1200 Android tablet
  works if the start view is rebuilt from its screen width (the page does this).
- A browser with Fullscreen API support (Fire OS's Silk browser; Chrome for Android also works), screen timeout set to
  "never" during the trial.
- Conductive fingertip covers and ground leads as above (about US$10 in all).
- Masking tape for the tablet's outline.

## Reset

Reload `map.html` and tap its overlay (start view, card closed); lay the tablet back on its tape outline; ready pose,
grippers open.

## Licences

The task uses only primitive shapes and the program's own kit (`yam_parts.py`, copied unchanged). `map.html` loads
Leaflet 1.9.4 (BSD-2-Clause) from a CDN; the map data is our own and released with the task.
