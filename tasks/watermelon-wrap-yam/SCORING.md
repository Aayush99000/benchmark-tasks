# Watermelon wrapping — scoring contract 2.0

The primary score is the highest cumulative level satisfied **at the end of the episode**, not the best earlier frame. Use 3,600 controls at 20 Hz (180 s). `scoring.py` and `tuck_measurement.py` implement these rules. Version 2 replaces counts of mesh nodes with a connected physical patch area; historical run014 remains a version 1 result.

| Score | Cumulative outcome |
|---:|---|
| 0 | No qualifying draw, or a latched failure below. |
| 0.2 | Both arms simultaneously grasped opposite material side edges at least once; total material length from leading edge to supply/cut line is 300–650 mm inclusive. |
| 0.5 | The draw condition and at least 95% coverage of the scoring disk on the cut face. |
| 0.75 | The above plus one connected underside film patch on each side, each >=200 mm² in side projection and >=15 mm wide. |
| 1.0 | The above plus full separation from the roll, no active film grasp, every robot collision bound >=20 mm from all film points, and retention of all conditions for >=1 continuous second. |

A wrap that loses coverage or a tuck after release loses that credit. One tuck alone earns no credit above 0.5. Cutter end position without a measured full cutting sweep earns no cut credit. Payout outside the permitted interval earns 0. Score is a hierarchy, not a sum; a higher level never bypasses a lower requirement.

## Measurements and simple-tool procedure

The fruit frame originates at the cut-face centre; X/Y lie in the face, Z points upward. The half has radius 90 mm. All scoring masks follow the fruit if it moves.

| Measurement | Numeric definition | Physical measurement |
|---|---|---|
| Draw | Material payout including the initial 75 mm: 300–650 mm | Film edge length marks or ruler between original leading edge and cut line; hand travel is not material length. Video establishes simultaneous opposite-edge hold. |
| Coverage | 5 mm Cartesian sample grid inside radius 87 mm; a sample counts if film covers it at height -2 to +12 mm from the face; count >=95% | Calibrated top photo with the same grid; side photo confirms the height band. The 3 mm outer rind border is intentionally excluded. |
| Tuck region | Film within 7 mm of spherical rind (83<radius<97), below z=-45, inward of the rim (horizontal radius<70), and on +Y with y>18 or -Y with y<-18 | Opposite low oblique side views plus top view and a ruler/reference hemisphere outline. The film must actually follow the underside; a distant hanging flap does not qualify. |
| Patch area | In each X/Z side projection, use a fixed 2 mm grid: x from -70 to +70, z from -98 to -44; cell centres start at (-69,-97). Count a cell once if its centre is covered by qualifying film. The largest four-neighbour-connected group must contain >=50 cells (200 mm²) and span >=15 mm including cell widths. | Apply the supplied grid to the side photo. Count union cells, not overlapping layers. Trace the largest connected patch; isolated specks and multiple disconnected small patches cannot be added together. |
| Cut | Native carriage follows x=180 ±8 and z=873 ±10 mm, starts at +Y, and crosses all 24 film-width intervals to -Y | Video shows the guarded slider traversing the whole sheet and a continuously separated edge. No manual tear or extra cutting tool is permitted. |
| Release/hold | Zero active film grasps, robot/film clearance >=20 mm, all full conditions for >=1 s | Timestamped top/side footage; use the final continuous second and a ruler for clearance. The robot may hold the cutter clear of the film; it may not support the wrap. |

The 2 mm grid is a stated measurement discretisation, not a count of simulation vertices. Boundary cells are tested by their centre. Overlapping triangles count once and the same area cannot earn credit on both sides. Reconstructed film points are used to decide the 3D region; no synthetic material fiducial grid needs to be printed on the film. With transparent film, backlighting and low-angle illumination should expose its outer boundary. Keep diagnostic cameras/mirrors outside the robot work envelope; they are evaluation equipment only.

## Invalid episodes and all other outcomes

The score latches to 0 for: cutting before both tucks qualify; a visible tear; fruit centre moving horizontally >150 mm from reset; fruit cut-face centre below z=785 mm; fruit-up dot world-up <0.7 (tilt >45.573°); or nonfinite simulation state. The numerical film model additionally latches damage when any edge/rest ratio >3 or its overstrain accumulator exceeds 0.20 s while ratio >1.6, with decay at 0.1 times elapsed time. These strain thresholds are numerical validity guards, not calibrated tearing properties of real LDPE.

All other incomplete states use the table: no draw=0; valid draw without coverage=0.2; cover without both tucks=0.5; both tucks without cut/release/hold=0.75. Loss of final geometry reduces the score; recovery does not undo a latched failure. At timeout, score the final state. Occlusion or measurement uncertainty crossing a threshold means the higher level is unverified; report the highest established level and the missing observation.

## Reset and evidence sheet

Before each trial photograph fresh uncut film, the 75 mm exposed leader, fruit centred on the four supports, cutter at y=+215 mm and empty open grippers. No pre-tuck, crease, detached sheet or glued film. Record source revision, seed, action limit, payout, coverage count/total, both patch cell counts/widths, first qualifying tuck time, cut time, minimum clearance, continuous released hold, final score and any latched failure. Use <=0.5 mm/pixel evaluation views and 1 mm rulers. Save the unedited full sequence.

## Checked evidence

The fresh v2 run `evidence/v2_demo_001` achieved **1.0 at 1,335 of 3,600 actions**. Independent execution/source audit passed. Coverage was99.8946%; connected underside patches were1,040/904 mm² and54/62 mm wide; payout425 mm; robot clearance155.8 mm; continuous released hold4.05 s. Opposite-edge grasp was observed at165, both tucks at665, and cut completion at1030.

The bundle includes the complete66.6-second video, all267 score samples, source snapshots, setup views and30 selected geometry states, including the final10 samples spanning2.25 seconds. The reviewer independently recomputed their coverage and tuck metrics. Full native geometry remains in the original GPU run; the exported geometry is explicitly sampled. Historical v1 evidence remains labelled separately. The authenticated v2 Dream-1 test completed at0.0/1.0 after3,600 actions; complete EvalLog and all three camera videos are included under `evidence/v2_model_002`.
