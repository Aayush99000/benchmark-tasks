# Fabrication package: twin-roll dispenser prototype P0

This package supplies dimensioned manufacturing geometry and assembly instructions for the task's original custom dispenser. It is not an OEM product or a claim that a physical prototype has already been built. The existing native demonstrations remain simulation evidence. Machining, assembly and the bench measurements below are still required before real trials.

Start with [ASSEMBLY.md](ASSEMBLY.md), [BOM.csv](BOM.csv) and the six [dimension sheets](drawings/). The shop hand-off consists of:

| File | Use |
| --- | --- |
| `drawings/01-cabinet.svg` | Closed datums, supports and full installation dimensions |
| `drawings/02-latch.svg` | Exact POM cam/square load zone, real guide pins/slots and spring |
| `drawings/03-selector.svg` | Plate, tracks, keeper strips, openings and 240 mm travel |
| `drawings/04-hinge-damper.svg` | Pin hinge, edge reliefs, mechanical stops and passive fluid damper |
| `drawings/05-assembly.svg` | Assembly sequence and mounting details |
| `drawings/06-closing-tool.svg` | Six aluminium stock pieces and the explicit edge-joint weld |
| `cut-list.csv` | All original box-stock pieces, exact size, orientation, location and material |
| `parts-mm.json` | Machine-readable dimensions, frames and exported CAD bounds |
| `stl/*.stl` | Closed, consistently oriented, millimetre solids for machining/printing |
| `assembly.scad` | Inspectable CAD assembly; choose a named `part` for export |
| `closed_assembly.glb` | Assembly inspection; this visual is not a manufacturing acceptance test |
| `generate.py`, `drawings.py` | Rebuild the manufacturing files from the adjacent submitted manifest |
| `verification.json` | Binary STL readback, dimensions, volumes and finite motion checks |

**Units and datums:** STL, SCAD, drawings and dimension tables use **millimetres**. The visual GLB uses **metres**, as required by glTF; it is scaled by0.001 from those CAD coordinates and uses the same datum. The closed assembly origin is world `(368,-245,787.5)` mm. Thus `world_mm = CAD_mm + [368,-245,787.5]`. X points towards the cabinet rear; Y runs from its right edge to its left edge; Z points up. Do not auto-centre imported parts when checking the assembly. A printer may move an individual part onto its bed, but that does not change its assembly coordinates. The submitted manifest also gives the separate 90° open starting pose.

`source_*_contact_envelope.stl` preserves the submitted contact geometry, with sub-micron cleanup of coincident radial-box seams. It is an assembly reference, not necessarily one fused manufactured part: the source cabinet includes a shell and two separately fixed axles; the source support pair includes four pieces; the original closing tool contains an edge-only neck/bridge connection. The corresponding fabricated CAD and instructions provide real connections. `cabinet_with_joint_hardware.stl` is the ABS shell with shoulder clearances, keepers and stops; **install the two separately exported `support_*_with_insert_bore.stl` axles** using the M6 fixings. The original steel receiver working members remain steel, with a machined steel guide/seat body and removable steel upper. Install `receiver_steel_body.stl` and `receiver_upper_removable.stl` as separate parts, following the ordered assembly. The combined receiver STL is an inspection reference. The selector, bolt, pins, hinge shaft and damper rotor remain separate moving parts. Never print the assembly as one fused lump.

The fabrication files add physical hardware omitted by the ideal simulator joints: hinge knuckles and a shaft, local non-working hinge-edge reliefs, support fixing holes, a two-pin bolt guide, compression spring reaction seat, selector side keepers and mechanical stops. The POM bolt's inclined working profile and its contained 6×6 mm load zone are retained. Mounting bores are on its non-working top/rear faces. A small specified weld makes the closing tool's edge-only source join physically continuous. See the explicit differences in [ASSEMBLY.md](ASSEMBLY.md).

The cover is routed 4 mm ABS sheet with bonded ABS hinge/latch features. Machine the POM-C bolt and steel receiver/guide body; a generic resin/FDM substitute does not retain their specified materials. Prefer one-piece CNC aluminium for the closing tool, retaining the explicit small connecting web; the optional welded-stock route gives two required riser notches. The large fixed shell may be built from cut ABS stock and bonded smaller printed brackets. The kit does not assume a small printer can produce a 490 mm cabinet in one piece. Ream journals and finish guide fits after printing. Do not resize critical contact profiles to compensate globally for print shrinkage.

Rebuild, without starting a simulator:

```sh
python -m pip install --target /tmp/dispenser-cad-deps manifold3d==3.5.5 trimesh numpy
PYTHONPATH=/tmp/dispenser-cad-deps python fabrication/generate.py
python fabrication/drawings.py --pdf
```

The `--pdf` option requires local Google Chrome or Chromium and prints all six SVG sheets as A3 landscape pages without file-path headers or footers. Omit that option to regenerate SVG only. The script creates its temporary HTML/browser profile outside this package and writes the final PDF to `fabrication/drawings/dimensions.pdf`.

Manifold booleans produce closed solids; each exported binary STL is re-imported and checked for watertightness, consistent winding, positive volume and bounds error below 0.001 mm. The additional door hardware is checked at 181 angles over 0–180°, the bolt guide at 73 positions over 18 mm and the selector keepers at 241 positions over 240 mm. These are finite geometry checks, not continuous contact, load, durability or physical-success certifications.

The simulation's damping/friction values are engineering targets. The real passive damper and lubricated guides have explicit hardware and measurement procedures; their actual coefficients, added moving masses and tissue properties have not been measured. Record the assembled values instead of claiming that an ideal revolute/prismatic joint or an STL guarantees those values. No existing simulation, scorer, action file or recorded result is changed by this manufacturing documentation.
