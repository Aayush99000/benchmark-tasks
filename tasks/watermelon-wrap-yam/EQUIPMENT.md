# Watermelon wrapping: equipment and geometry

This is a dimensioned reproduction proposal. The simulated geometry below is implemented; the proposed physical materials and assembly have **not** been purchased, built or tested. No branded dispenser is claimed to match these dimensions. The specific physical assembly below uses a stock guarded slide-cutter rail on a dimensioned housing; do not substitute a push-down cutter. Material choices below are build assumptions, not recovered simulator material measurements.

All dimensions and positions are in millimetres in the standard station world frame. The work surface is z = 750. Object positions are centres unless an edge or face is named. X increases from the dispenser towards the fruit; Y runs along the dispenser. The fruit cut-face centre is (350, 0, 840).

| Item | Quantity | Implemented geometry and initial location | Proposed physical specification; status |
|---|---:|---|---|
| YAM arms and parallel grippers | 2 | Unchanged calibrated station mounts and cameras from the pinned YAM adapter; see `station_setup.py` and README | Existing YAM station; task-specific hardware trials unperformed |
| Work mat | 1 | 800 × 1000; centre (350, 0); top at z=750; thickness = 750 minus the adapter's table height (2.5 in the referenced station) | Flat washable silicone mat at the measured required thickness; assumed |
| Watermelon half | 1 | Hemisphere diameter 180, height 90, mass 0.8 kg; flat face at z=840, rind down | Real mini-watermelon half cut to the specified geometry; physical reset target diameter180±5, height90±3 mm and mass0.80±0.10 kg; record actual values and use the same nominal scoring mask; trial produce is not consumed |
| Fruit rests | 4 | Each 12 × 12 × 38; centres at every combination of x=285 or 415, y=-20 or +20, z=769 | Washable HDPE blocks, secured to the mat/base so the two Y-direction access lanes remain open; M4 countersunk bolts into the fixed base below the mat; expose no bolt heads on the support tops |
| Dispenser housing | 1 | 130 × 460 × 75; centre (115, 0, 787.5) | Fabricated enclosure with deburred stainless feed lip, as specified below; unbuilt, not a retail-appliance claim |
| Film roll | 1 | Visual cylinder diameter 70, axial width 400; axis Y; centre (115, 0, 810) | Proposed food-contact cling film, nominal 400 mm width and 10 micrometre LDPE thickness; the model does not encode a polymer or physical thickness, so this choice requires validation |
| Film sheet at reset | 1 | Continuous roll-fed sheet, 400 wide and 75 exposed; supply edge x=180, z=850; leading edge x=255; Y extent -200 to +200 | Same intact film; no pre-tucks, prepared creases or pre-cut detached sheet |
| Feed lip | 1 | 84 × 412 × 4; centre (218, 0, 847), top z=849 | Smooth deburred stainless sheet on rigid housing; model supports film at z=850 |
| Dispenser clamps | 2 | Each 40 × 24 × 40; centres (130, -218, 770), (130, +218, 770) | Table clamps or bolts fixing the housing; M6 through bolts or bench clamps outside the film path |
| Guarded rail | 1 | 25 × 450 × 8; centre (180, 0, 855) | Covered commercial cutter rail with no exposed accessible blade; a stock 450 mm replacement cling-film slide-cutter rail, mounted with the blade enclosed, as specified below |
| Cutter carriage | 1 | 24 × 24 × 28; mass 25 g; centre initially (180, 215, 873); native prismatic travel to y=-215 | Graspable carriage of the same dimensions on the guarded rail; force, friction and cutting load not calibrated |
| Passive roll brake | 1 proposed | Simulated payout occurs under sheet extension; there is no native spindle or layered roll mechanics | Adjustable passive drag on a spindle; 6 mm stainless axle with felt end washers and M6 locknuts; adjust tangential pull at the feed lip to0.2–0.5 N using a spring scale; this physical setting is a stated reproduction target, not a calibrated simulation parameter |
| Evaluation cameras and rulers | Existing cameras plus diagnostic views as needed | Top and two wrist cameras remain policy observations; overview and underside views are evaluation only | Fixed top view, two low oblique/side views, millimetre rulers and a 5 mm image overlay; mirrors may provide underside visibility without moving the completed wrap |

The rigid fruit has native friction coefficients 0.65 static and 0.5 dynamic; the cutter uses 1.0 and 0.8. These are simulation parameters, not measured coefficients of the materials proposed above. Rigid fruit damping and the film's local tack are similarly uncalibrated.

The four rests are real collision objects in the simulation; they do not constrain the fruit pose. There are no added handles on the fruit and no object teleportation. The film is reduced co-simulation, with material points spaced 12.5 mm along the sheet and 400/24 = 16.667 mm across it. The apparent film roll is visual, while the supply boundary implements payout. Those distinctions must remain in the fidelity note.

## One specified assembly

Fabricate the 130×460×75 mm hollow dispenser housing from 6 mm HDPE plate, with two end cheeks and a removable lid. Fit a 6 mm stainless axle through the roll core using two acetal core plugs. Set its centreline at world(115,0,810), axis Y. Use food-contact400 mm-wide, nominal10 micrometre LDPE cling film; exposed roll diameter70±5 mm. Use felt washers and M6 locknuts outside both cheeks as the passive drag adjustment. Secure the housing at the two clamp locations; target location tolerance±1 mm.

Mount an off-the-shelf **guarded 450 mm sliding cling-film cutter rail** with its cut line at x180,z850, parallel Y. It must permit the full430 mm carriage-centre travel, from +215 to-215, and keep its blade enclosed. Fit a24×24×28 mm HDPE grip sleeve to the stock carriage, keeping the cutting slot and guards clear; carriage plus sleeve target25±5 g. Fix the rail to the HDPE lid with M4 bolts. The specified25×450×8 rail body and the sleeve are the external envelopes used in simulation. Select a replacement rail that fits those envelopes or document a scene revision; a 450 mm press-down Wrapmaster head is not an equivalent mechanism.

Bolt the deburred84×412×4 mm stainless feed lip to the housing so its top is z849 and the sheet exits at z850. Feed the intact sheet over the lip, below the guarded cutter path, towards +X. Its leading edge starts at x255:75 mm exposed. Do not detach it from the roll, fold it under the fruit, or add a grip tab.

The four12×12×38 mm HDPE rests are individually fixed from below; their centres are listed above. They leave both Y-side approach lanes clear. The half rests freely on them, cut-face up. Neither clamps nor glue may hold the fruit or its film. The rests are deliberately part of the task fixture, not hidden pose constraints.

Use the standard station table1100×1300×60 at(360,0,717.5), and arm bases at left(0,305,750), right(0,-305,750), both facing +X. Keep standard parallel grippers and the pinned station reset joint angles. The top camera centre is(-166.4949,-9,1691.8205), quaternion WXYZ=(0.68301,0.18301,-0.18301,-0.68301), normalized. Top intrinsics640×480: fx392.1956,fy391.7224,cx318.3894,cy237.8763 with 60-row top/bottom crop. Exact wrist transforms, standard camera stand/floor/lamp and calibration are in [station.py at the pinned revision](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/station.py). No fruit/cutter fixture changes the arm mounts.

## Reset and measurement setup

1. Fit fresh intact film and draw exactly75±2 mm. Place cutter at +215±1 mm. Set all fixtures to the specified centres±1 mm and axes±1°. Empty open grippers; no preformed tucks or creases.
2. Place the fruit cut-face centre at(350,0,840)±2 mm, cut-face level±2°. It is a natural fruit in the physical trial; use the size/mass tolerances above. Simulation uses the nominal rigid hemisphere and seed 0, no object jitter.
3. Place1 mm rulers in top/two low oblique views, outside the manipulation space. A fixed mirror may expose the underside; do not move the fruit to inspect during scoring. Use the2 mm patch grid and5 mm coverage grid in SCORING.md, resolution<=0.5 mm/pixel. Extra views are evaluation-only.
4. Start from a settled station; record the complete180-second maximum episode. After each trial reset with fresh film, return cutter to +Y and fruit to the same four rests; do not reuse a partially wrapped sheet.
5. Retain the guarded commercial cutter assembly without exposed blade access; the robot grasps only the sleeve. Actual pull force, cutter travel and fruit dimensions are recorded per rig. These are reproduction parameters; material calibration is not claimed.

A reproducible design does not establish hardware validation. The same essential skill—draw continuous film, tuck around the rounded underside, cut and retain—must remain in any physical replica. The film's one-way coupling and local tack are approximation limits, already disclosed; they are not evidence that real forces match.

## Asset provenance

Task fruit, seed marks, film, dispenser, rests and cutter are original procedural geometry in `assets.py` and `film.py`. The package does not redistribute a robot mesh collection. YAM, the station and Isaac components are operator-installed dependencies; retain their upstream licences and versions when packaging an execution environment. The compatibility overlay does not grant any new redistribution rights.
