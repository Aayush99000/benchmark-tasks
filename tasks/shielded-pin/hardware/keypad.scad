// The PIN pad for real trials: a case the size of an Ingenico Lane/3000 (180 x 83 x 24 mm) with a 3 x 5 keypad of
// Kailh Choc V1 Brown switches under printed 18 x 11 mm keycaps, at the same positions as in shielded_pin.py.
// Print in PLA or PETG, 0.2 mm layers. Render one part at a time: part = "case", "plate" or "cap".
part = "case";

L = 180; W = 83; H = 24;            // case, mm (x along the length, keypad at x = 0 end)
WALL = 2.5;
PLATE_T = 1.3;                      // Choc V1 plate thickness (1.2-1.3 mm)
CUT = 13.8;                         // Choc V1 plate cutout, square
ROW0 = 78; ROW_PITCH = 16;          // rows from the keypad end: 78, 62, 46, 30, 14 mm (shielded_pin.ROW_X)
COL_PITCH = 22;                     // columns at y = +22, 0, -22 mm from the centre line
CAP_W = 18; CAP_D = 11;             // keycap top, across (y) and along (x)
KEY_PROUD = 3.5;                    // a cap's top above the case top at rest
CLEAR = 1.5;                        // gap between a cap and the case opening

module key_positions() {
  for (r = [0:4]) for (c = [-1:1]) translate([ROW0 - ROW_PITCH * r, COL_PITCH * c, 0]) children();
}

// The shell: open at the bottom, an opening round each keycap in the top, a screen recess at the far end.
module case_shell() {
  difference() {
    translate([0, -W / 2, 0]) cube([L, W, H]);
    translate([WALL, -W / 2 + WALL, -1]) cube([L - 2 * WALL, W - 2 * WALL, H - WALL + 1]);
    key_positions() translate([-(CAP_D + 2 * CLEAR) / 2, -(CAP_W + 2 * CLEAR) / 2, H - WALL - 1])
      cube([CAP_D + 2 * CLEAR, CAP_W + 2 * CLEAR, WALL + 2]);
    translate([110, -30, H - 0.8]) cube([60, 60, 1]);   // screen recess (a dark sticker goes here)
  }
}

// The switch plate: glued under the top, at the depth that puts the cap tops KEY_PROUD above the case.
// PLATE_DROP is a starting value [not checked against Kailh's drawing]: after a test print, adjust it so the cap tops
// stand 3.5 mm above the case top at rest (KEY_PROUD), so that a cap never bottoms on the case.
PLATE_DROP = 6.0;
module plate() {
  difference() {
    translate([2, -(W - 2 * WALL - 1) / 2, 0]) cube([92, W - 2 * WALL - 1, PLATE_T]);
    key_positions() translate([-CUT / 2, -CUT / 2, -1]) cube([CUT, CUT, PLATE_T + 2]);
  }
}

// A keycap: 18 x 11 mm top, 6 mm tall, two Choc V1 stem sockets underneath (5.7 mm apart, 1.2 x 3.0 mm prongs).
// The digit and four 2 mm corner dots are recessed 0.4 mm into the top and filled with white paint: the dots are
// what the rater looks for in the onlooker's camera.
module cap(label = "5") {
  difference() {
    translate([-CAP_D / 2, -CAP_W / 2, 0]) cube([CAP_D, CAP_W, 6]);
    for (s = [-1, 1]) translate([-1.6, s * 5.7 / 2 - 0.65, -0.01]) cube([3.2, 1.3, 3.0]);   // stem sockets
    for (sx = [-1, 1]) for (sy = [-1, 1])
      translate([sx * (CAP_D / 2 - 1.5) - 1, sy * (CAP_W / 2 - 1.5) - 1, 5.6]) cube([2, 2, 1]);
    translate([0, 0, 5.6]) linear_extrude(1) rotate(-90) text(label, size = 5, halign = "center", valign = "center");
  }
}

if (part == "case") case_shell();
if (part == "plate") plate();
if (part == "cap") cap();
