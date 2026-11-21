"""Unheated turntable fixture, roller ring, transparent tray and mug. Dimensions in metres."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from yam_arena import station
from station_model import add_arms, numbers
import coupling

TABLE = station.TABLE_TOP
HUB = np.array([0.565, 0.0])
FRONT = HUB[0] - 0.161  # closed door / yellow withdrawal line
FLOOR = TABLE + 0.115
HINGE = np.array([FRONT, 0.172])
LATCH_RADIUS = 0.344
HANDLE_RADIUS = 0.332
OPEN = math.radians(-170)
RING_START = np.array([0.202, 0.136])
TRAY_START = np.array([0.226, -0.085])
MUG_START = np.array([0.365, -0.322])
RING_R = 0.079
TRAY_R = 0.127
RING_PICK_Z = TABLE + 0.087
TRAY_PICK_Z = TABLE + 0.093
MUG_R = 0.031
MOTOR_SPEED = math.tau / 10
BUTTON = np.array([HUB[0] - 0.174, -0.244, FLOOR + 0.039])
OFFSET = np.array([-0.063, -0.016])


def geom(parent, name, shape, pos, size, mat, **kw):
    attrs = dict(name=name, type=shape, pos=numbers(pos), size=numbers(size), material=mat)
    return ET.SubElement(parent, "geom", **attrs, **kw)


def decor(parent, name, shape, pos, size, mat, **kw):
    return geom(parent, name, shape, pos, size, mat, contype="0", conaffinity="0", mass="0", **kw)


def circ(parent, name, radius, z, tube, mat, segments=32, visual=False):
    make = decor if visual else geom
    for i in range(segments):
        a, b = np.array([i, i + 1]) * math.tau / segments
        ends = (
            radius * math.cos(a),
            radius * math.sin(a),
            z,
            radius * math.cos(b),
            radius * math.sin(b),
            z,
        )
        make(parent, f"{name}_{i}", "capsule", (0, 0, 0), (tube,), mat, fromto=numbers(ends))


def fit_fascia(assets, case):
    # A planar UV mesh keeps the original printable keypad artwork at its physical size.
    ET.SubElement(assets, "texture", name="keys", type="2d", file=str(Path(__file__).with_name("fascia.png")))
    ET.SubElement(assets, "material", name="keypad", texture="keys", specular=".22", shininess=".28")
    ET.SubElement(assets, "mesh", name="key_face", vertex="0 .040 -.111  0 -.040 -.111  0 -.040 .111  0 .040 .111  .0005 0 0",
                  texcoord="0 1  1 1  1 0  0 0  .5 .5", face="0 1 2  0 2 3  1 0 4  2 1 4  3 2 4  0 3 4")
    ET.SubElement(case, "geom", name="keypad_print", type="mesh", mesh="key_face", pos="-.168 -.220 .112",
                  material="keypad", contype="0", conaffinity="0", mass="0")


def cabinet(w, drives, assets):
    steel = ET.SubElement(w, "body", name="cabinet", pos=numbers((*HUB, FLOOR)))
    # 440 W x 355 D (including closed handle) x 258 H, on a separate motor plinth.
    geom(steel, "oven_floor", "box", (0, -.040, -.006), (.150, .220, .006), "enamel")
    geom(steel, "base_pan", "box", (0, -.040, -.018), (.150, .220, .006), "case")
    geom(steel, "counter_riser", "box", (0, -.040, -.0745), (.143, .205, .0405), "plinth")
    for dx in (-.121, .121):
        for dy in (-.220, .143):
            geom(steel, f"foot_{dx}_{dy}", "cylinder", (dx, dy, -.029), (.013, .005), "rubber")
    for name, radius in (("inner_track", RING_R-.013), ("outer_track", RING_R+.013)):
        circ(steel, name, radius, 0, .0025, "enamel", 48)
    # Light cavity liner and black skin with three glazed inspection apertures.
    geom(steel, "roof_window", "box", (0, 0, .222), (.146, .148, .002), "clear")
    geom(steel, "rear_window", "box", (.148, 0, .053), (.002, .156, .053), "clear")
    geom(steel, "left_window", "box", (0, .178, .053), (.148, .002, .053), "clear")
    for name, pos, size, material in (
        ("roof_front", (-.1485, -.040, .222), (.0025, .220, .002), "case"),
        ("roof_rear", (.148, -.040, .222), (.002, .220, .002), "case"),
        ("roof_left", (0, .164, .222), (.146, .016, .002), "case"),
        ("roof_right", (0, -.204, .222), (.146, .056, .002), "case"),
        ("rear_wall", (.147, -.040, .163), (.003, .220, .057), "enamel"),
        ("rear_skin", (.149, -.040, .163), (.001, .220, .057), "case"),
        ("left_wall", (0, .176, .163), (.150, .004, .057), "enamel"),
        ("left_skin", (0, .179, .163), (.150, .001, .057), "case"),
        ("rear_left_jamb", (.147, .168, .053), (.003, .012, .053), "case"),
        ("left_front_jamb", (-.146, .178, .053), (.004, .002, .053), "case"),
        ("control_housing", (-.0085, -.220, .110), (.1585, .040, .110), "case"),
        ("oven_right_wall", (0, -.177, .110), (.150, .003, .110), "enamel"),
        ("upper_lintel", (-.148, 0, .212), (.002, .178, .008), "case"),
        ("threshold", (-.156, .001, -.021), (.006, .178, .003), "case"),
    ):
        geom(steel, name, "box", pos, size, material)
    decor(steel, "withdrawal_line", "box", (-.161, .001, -.0176), (.001, .178, .0003), "amber")
    for i, y in enumerate((.172, -.194)):
        decor(steel, f"sill_tab_{i}", "box", (-.156, y, .2244), (.007, .004, .0004), "clear")
        decor(steel, f"sill_tick_{i}", "box", (-.161, y, .225), (.001, .004, .0005), "amber")
    fit_fascia(assets, steel)
    for i in range(9):
        decor(steel, f"vent_{i}", "box", (.052+i*.008, -.218, .2243), (.0016, .024, .0003), "vent")
    # Flat paint under the clear tray and reticles at the inspection roof.
    for radius, ink in ((67, "ring_limit"), (91, "ring_limit"), (35, "load_ink"), (78, "load_ink")):
        z = .0028 if radius in (67, 91) else .0005
        circ(steel, f"template_{radius}", radius/1000, z, .00065, ink, 60, visual=True)
    for radius, ink in ((.006, "navy"), (.012, "ring_limit")):
        circ(steel, f"center_target_{radius}", radius, .2243, .00055, ink, 36, visual=True)
    for i in range(4):
        a = i * math.pi / 2
        c, s = math.cos(a), math.sin(a)
        decor(steel, f"quadrant_{i}", "box", (.139*c, .139*s, .0006),
              (.005, .002, .0005), "navy", euler=numbers((0, 0, a)))
    for side in ("back", "left"):
        # Scales sit to one side of the visible disks.
        back = side == "back"
        xy = (.142, -.141) if back else (-.139, .151)
        size = (.0004, .012, .052) if back else (.012, .0004, .052)
        decor(steel, side+"_ruler", "box", (*xy, .052), size, "enamel")
        xy = (xy[0]+.001, xy[1]) if back else (xy[0], xy[1]+.001)
        def bar(name, z, width, height, ink):
            size = (.0005, width/2, height/2) if back else (width/2, .0005, height/2)
            decor(steel, f"{side}_{name}", "box", (*xy, z), size, ink)
        for mm in range(0, 106, 5):
            bar(f"tick_{mm}", mm/1000, .024 if mm % 10 == 0 else .018, .0007, "navy")
        for name, lo, hi, ink in (("ring", 5, 15, "ring_ink"), ("tray", 20, 28, "stripe"),
                                  ("foot", 30, 38, "load_ink"), ("lip", 90, 101, "load_ink")):
            # Side-by-side ink columns keep the tray and foot bands distinct.
            offset = -.005 if ink == "load_ink" else .005
            loc = (xy[0]+.001, xy[1]+offset) if back else (xy[0]+offset, xy[1]+.001)
            dims = (.0005, .003, (hi-lo)/2000) if back else (.003, .0005, (hi-lo)/2000)
            decor(steel, f"{side}_{name}_band", "box", (*loc, (hi+lo)/2000), dims, ink)
    # Views of one pilot lamp, wired across the motor.
    for name, pos, size in (
        ("lamp_face", (-.169, -.220, .168), (.001, .009, .003)),
        ("lamp_plan", (-.132, -.191, .225), (.008, .008, .001)),
    ):
        decor(steel, name+"_bezel", "box", pos, size, "graphite")
        loc = np.array(pos) + ([-.0015, 0, 0] if name == "lamp_face" else [0, 0, .0015])
        ET.SubElement(steel, "geom", name=name, type="box", pos=numbers(loc), size=numbers(np.array(size)*.8),
                      rgba=".12 .15 .13 1", contype="0", conaffinity="0", mass="0")
    # The fixed hoop frames the dot on the door below it when the door is shut.
    target = ET.SubElement(steel, "body", name="latch_target",
                           pos=numbers((FRONT - HUB[0], HINGE[1] - LATCH_RADIUS, .224)))
    circ(target, "latch_witness", .010, 0, .0012, "load_ink", 32, visual=True)
    decor(steel, "witness_holder", "box", (-.161, -.187, .224), (.002, .006, .001), "load_ink")
    panel = ET.SubElement(w, "body", name="start_button", pos=numbers(BUTTON))
    ET.SubElement(
        panel,
        "joint",
        name="button",
        type="slide",
        axis="1 0 0",
        range="0 .005",
        stiffness="110",
        damping="1.0",
        armature=".002",
    )
    geom(panel, "start", "box", (0, 0, 0), (0.004, 0.010, 0.006), "teal", mass=".012")
    decor(panel, "start_mark", "ellipsoid", (-0.0041, 0, 0), (0.0004, 0.004, 0.003), "enamel")
    door = ET.SubElement(w, "body", name="door", pos=numbers((*HINGE, FLOOR)))
    ET.SubElement(
        door,
        "joint",
        name="hinge",
        type="hinge",
        axis="0 0 1",
        range=numbers((OPEN - 0.02, 0.009)),
        damping=".13",
        frictionloss=".095",
        armature=".003",
    )
    ET.SubElement(door, "inertial", mass=".70", pos="0 -.176 .105", diaginertia=".0105 .0032 .0077")
    geom(door, "window", "box", (0, -.157, .105), (.002, .123, .064), "smoked")
    for name, pos, size in (
        ("sill", (0, -.175, .012), (.009, .175, .029)),
        ("top", (0, -.175, .195), (.009, .175, .026)),
        ("hinge_edge", (0, -.017, .105), (.009, .017, .064)),
        ("door_edge", (0, -.315, .105), (.009, .035, .064)),
    ):
        geom(door, name, "box", pos, size, "graphite")
    decor(door, "latch_mark", "cylinder", (0, -LATCH_RADIUS, .222), (.0035, .0006), "stripe")
    # Tall squared pull and its two stand-offs, as on the reference front.
    for z in (.015, .185):
        geom(door, f"handle_mount_{z}", "box", (-.023, -HANDLE_RADIUS, z), (.014, .008, .007), "graphite")
    geom(door, "handle", "box", (-.035, -HANDLE_RADIUS, .100), (.009, .010, .092), "pull")
    shaft = ET.SubElement(w, "body", name="spindle", pos=numbers((*HUB, FLOOR)))
    ET.SubElement(shaft, "joint", name="shaft", type="hinge", axis="0 0 1", damping=".002", armature=".0005")
    # Rendering and the printable part share one surface. Convex collision pieces
    # preserve the recessed, three-lobed opening in MuJoCo.
    def mesh(name, vertices, faces=None):
        attrs = dict(name=name, vertex=numbers(np.ravel(vertices)/1000))
        if faces is not None:
            attrs["face"] = " ".join(map(str, faces.ravel()))
        ET.SubElement(assets, "mesh", **attrs)
    mesh("drive_cup", *coupling.cup())
    mesh("tray_lug", *coupling.lug())
    ET.SubElement(shaft, "geom", name="cup_surface", type="mesh", mesh="drive_cup",
                  material="coupler", contype="0", conaffinity="0", mass="0")
    base_angle = math.tau/3
    # Twenty sectors span one lobe. The other two are exact 120-degree copies.
    for j in range(coupling.COUNT//3):
        outer = coupling.OUTER[j:j+2]
        inner = coupling.INNER[j:j+2]
        floor_xy = np.vstack(([0, 0], outer))
        wall_xy = np.vstack((outer, inner[::-1]))
        for label, xy, low, high in (("floor", floor_xy, 2, coupling.POCKET_FLOOR),
                                      ("wall", wall_xy, coupling.POCKET_FLOOR, coupling.CUP_TOP)):
            name = f"cup_{label}_{j}"
            mesh(name, np.vstack((coupling.level(xy, low), coupling.level(xy, high))))
            for k in range(3):
                ET.SubElement(shaft, "geom", name=f"{name}_{k}", type="mesh", mesh=name, group="3",
                              material="coupler", euler=numbers((0, 0, k*base_angle)))
    ET.SubElement(
        drives, "velocity", joint="shaft", name="turntable_motor", kv=".23", forcerange="-.20 .20"
    )


def cup_surface(assets):
    # A continuous glazed shell for rendering. The separate physical walls leave the cup open.
    profile = [
        (0.026, 0),
        (0.031, 0.004),
        (0.031, 0.062),
        (0.030, 0.0675),
        (0.028, 0.0695),
        (0.0255, 0.067),
        (0.0255, 0.008),
        (0.023, 0.006),
    ]
    vertices, faces = [], []
    count = 64
    for radius, z in profile:
        for i in range(count):
            a = math.tau * i / count
            vertices.extend((radius * math.cos(a), radius * math.sin(a), z))
    for j in range(len(profile) - 1):
        for i in range(count):
            a = j * count + i
            b = j * count + (i + 1) % count
            faces.extend((a, b, a + count, b, b + count, a + count))
    ET.SubElement(
        assets, "mesh", name="ceramic_shell", vertex=numbers(vertices), face=" ".join(map(str, faces))
    )


def loose_parts(w):
    # Three short silicone drying rests leave the front edge of the tray accessible.
    for i, angle in enumerate((0, 125, 235)):
        a = math.radians(angle)
        p = TRAY_START + 0.069 * np.array([math.cos(a), math.sin(a)])
        geom(
            w,
            f"drying_rest_{i}",
            "cylinder",
            (*p, (TRAY_PICK_Z - 0.004 + TABLE) / 2),
            (0.012, (TRAY_PICK_Z - 0.004 - TABLE) / 2),
            "rubber",
        )
    for i, angle in enumerate((0, 110, 250)):
        a = math.radians(angle)
        p = RING_START + RING_R * np.array([math.cos(a), math.sin(a)])
        h = RING_PICK_Z - TABLE - 0.0034
        geom(w, f"guide_rest_{i}", "cylinder", (*p, TABLE + h / 2), (0.012, h / 2), "rubber")
    ring = ET.SubElement(w, "body", name="roller_ring", pos=numbers((*RING_START, RING_PICK_Z)))
    ET.SubElement(ring, "freejoint", name="ring_pose")
    ET.SubElement(ring, "inertial", pos="0 0 0", mass=".044", diaginertia=".00012 .00012 .00022")
    # Three open axle forks leave the nylon wheels free to roll. Keep the painted
    # centerline on the hoop ends visible in both plan and elevation.
    for j in range(60):
        a, b = np.array([j, j+1]) * math.tau/60
        from_wheel = abs(((a+b)/2 - math.pi + math.pi/3) % (math.tau/3) - math.pi/3)
        if from_wheel < math.radians(9):
            continue
        ends = numbers((RING_R*math.cos(a), RING_R*math.sin(a), 0,
                        RING_R*math.cos(b), RING_R*math.sin(b), 0))
        geom(ring, f"guide_{j}", "capsule", (0, 0, 0), (.0034,), "roller", fromto=ends)
        decor(ring, f"axle_line_{j}", "capsule", (0, 0, 0), (.0035,), "ring_ink", fromto=ends)
    for i in range(3):
        a = math.pi + i * math.tau / 3
        axis = np.array([math.cos(a), math.sin(a), 0.0])
        for side in (-1, 1):
            geom(ring, f"fork_{i}_{side}", "box", axis*(RING_R+side*.0065),
                 (.001, .016, .0035), "roller", euler=numbers((0, 0, a)))
        for side in (-1, 1):
            mid = axis*(RING_R-.0016) + np.array([-axis[1], axis[0], 0])*side*.014
            geom(ring, f"fork_bridge_{i}_{side}", "box", mid, (.0085, .002, .0034), "roller",
                 euler=numbers((0, 0, a)))
        wheel = ET.SubElement(ring, "body", name=f"wheel_{i}", pos=numbers(axis * RING_R))
        ET.SubElement(wheel, "joint", name=f"axle_{i}", type="hinge", axis=numbers(axis), damping=".000004")
        geom(
            wheel,
            f"roller_{i}",
            "cylinder",
            (0, 0, 0),
            (0.010, 0.005),
            "wheel",
            zaxis=numbers(axis),
            mass=".004",
            friction=".45 .001 .0001",
        )
    tray = ET.SubElement(w, "body", name="tray", pos=numbers((*TRAY_START, TRAY_PICK_Z)))
    ET.SubElement(tray, "freejoint", name="tray_pose")
    ET.SubElement(tray, "inertial", pos="0 0 0", mass=".480", diaginertia=".00194 .00194 .00387")
    geom(tray, "glass_disk", "cylinder", (0, 0, 0), (TRAY_R, .004), "glass", friction=".55 .003 .0001")
    circ(tray, "raised_lip", TRAY_R-.0025, .004, .0025, "glass", 72)
    circ(tray, "tray_edge_mark", TRAY_R, 0, .0009, "navy", 72, visual=True)
    decor(tray, "tray_center", "cylinder", (0, 0, .0043), (.0021, .00025), "stripe")
    decor(tray, "tray_stripe", "box", (TRAY_R-.012, 0, .0048), (.012, .004, .0005), "stripe")
    decor(tray, "stripe_edge", "box", (TRAY_R, 0, 0), (.0006, .004, .004), "stripe")
    for i in range(3):
        ET.SubElement(tray, "geom", name=f"tray_lug_{i}", type="mesh", mesh="tray_lug",
                      pos="0 0 -.013", euler=numbers((0, 0, i*math.tau/3)), material="edge")
    mug = ET.SubElement(w, "body", name="mug", pos=numbers((*MUG_START, TABLE)))
    ET.SubElement(mug, "freejoint", name="mug_pose")
    ET.SubElement(mug, "inertial", pos="0 0 .032", mass=".19", diaginertia=".00015 .00016 .00010")
    geom(mug, "mug_bottom", "cylinder", (0, 0, 0.003), (MUG_R, 0.003), "glaze", friction=".75 .002 .0001")
    for i in range(24):
        a = i * math.tau / 24
        geom(
            mug,
            f"cup_wall_{i}",
            "box",
            (0.028 * math.cos(a), 0.028 * math.sin(a), 0.036),
            (0.003, 0.00385, 0.031),
            "glaze",
            euler=numbers((0, 0, a)),
            group="3",
        )
    ET.SubElement(
        mug,
        "geom",
        name="glazed_shell",
        mesh="ceramic_shell",
        type="mesh",
        material="glaze",
        contype="0",
        conaffinity="0",
        mass="0",
    )
    circ(mug, "cup_lip", 0.028, 0.067, 0.0031, "cream", 32, visual=True)
    decor(mug, "mug_dot", "cylinder", (0, 0, 0.007), (0.005, 0.00035), "yellow")
    circ(mug, "mug_foot_band", .031, .006, .0013, "mint", 48, visual=True)
    # The open loop faces the back of the oven when the mug is set down.
    for i in range(10):
        a, b = np.array([i, i + 1]) * math.pi / 10
        ends = (
            0.027 + 0.023 * math.sin(a),
            0,
            0.037 + 0.020 * math.cos(a),
            0.027 + 0.023 * math.sin(b),
            0,
            0.037 + 0.020 * math.cos(b),
        )
        # Orient the handle into the three-quarter view.
        ends = (ends[0] * 0.67, -ends[0] * 0.742, ends[2], ends[3] * 0.67, -ends[3] * 0.742, ends[5])
        geom(mug, f"cup_handle_{i}", "capsule", (0, 0, 0), (0.0043,), "glaze", fromto=numbers(ends))


def build():
    root = ET.fromstring("""<mujoco model="Microwave turntable">
      <compiler angle="radian" autolimits="true"/>
      <option timestep=".002" integrator="implicitfast" cone="elliptic" iterations="40"/>
      <visual><global offwidth="960" offheight="576"/><quality shadowsize="1536"/>
        <headlight ambient=".48 .48 .48" diffuse=".20 .20 .20" specular=".09 .09 .09"/>
        <map znear=".006"/></visual>
      <default><geom friction=".65 .003 .0001" solref=".006 1" solimp=".94 .99 .001"/></default>
      <asset/><worldbody/><contact/><actuator/></mujoco>""")
    assets, w, drives = (root.find(n) for n in ("asset", "worldbody", "actuator"))
    colors = {
        "enamel": " .83 .84 .81 1",
        "metal": ".56 .60 .62 1",
        "graphite": ".081 .086 .097 1",
        "case": ".112 .119 .132 1",
        "pull": ".125 .134 .146 1",
        "plinth": ".27 .29 .30 1",
        "vent": ".027 .032 .039 1",
        "smoked": ".19 .23 .25 .44",
        "amber": ".97 .66 .14 1",
        "clear": ".77 .90 .94 .045",
        "navy": ".065 .105 .16 1",
        "stripe": ".02 .16 .93 1",
        "ring_ink": ".65 .07 .48 1",
        "ring_limit": ".38 .22 .39 1",
        "load_ink": ".02 .45 .33 1",
        "yellow": "1 .90 .04 1",
        "rubber": ".23 .25 .26 1",
        "teal": ".09 .43 .38 1",
        "glaze": ".88 .30 .13 1",
        "cream": ".97 .85 .71 1",
        "mint": ".23 .93 .66 1",
        "glass": ".64 .85 .88 .17",
        "edge": ".60 .75 .74 .78",
        "coupler": ".67 .57 .45 1",
        "roller": ".40 .37 .33 1",
        "wheel": ".83 .74 .54 1",
        "counter": ".68 .63 .55 1",
        "tile": ".74 .77 .75 1",
    }
    for name, rgba in colors.items():
        ET.SubElement(
            assets,
            "material",
            name=name,
            rgba=rgba.strip(),
            specular=".58" if name in ("glass", "glaze", "metal") else ".21",
            shininess=".65" if name in ("glass", "glaze", "metal") else ".20",
        )
    ET.SubElement(
        assets,
        "texture",
        name="sky",
        type="skybox",
        builtin="gradient",
        rgb1=".78 .84 .87",
        rgb2=".93 .92 .88",
        width="256",
        height="1536",
    )
    # Neutral kitchen backsplash and counter run behind the working area.
    for row in range(5):
        for col in range(17):
            y = (col - 8) * 0.16 + (row % 2) * 0.08
            decor(
                w,
                f"tile_{row}_{col}",
                "box",
                (0.866, y, TABLE + 0.05 + row * 0.102),
                (0.009, 0.0796, 0.0506),
                "tile",
            )
    geom(w, "table", "box", (0.30, 0, TABLE - 0.025), (0.55, 0.55, 0.025), "counter")
    ET.SubElement(
        w,
        "light",
        pos="-.45 -.65 2",
        dir=".50 .40 -1",
        directional="true",
        diffuse=".39 .39 .38",
        ambient=".06 .06 .06",
        specular=".14 .14 .14",
        castshadow="false",
    )
    ET.SubElement(
        w,
        "light",
        pos=numbers((FRONT + 0.12, 0.10, FLOOR + 0.20)),
        dir=".4 -.3 -1",
        diffuse=".21 .20 .18",
        specular=".05 .05 .05",
        castshadow="false",
    )
    # Evidence camera looks through the front window and sees the loose parts.
    eye = np.array([-0.38, -0.052, TABLE + 0.50])
    target = np.array([0.32, -0.014, TABLE + 0.074])
    forward = (target - eye) / np.linalg.norm(target - eye)
    right = np.cross(forward, [0, 0, 1])
    right /= np.linalg.norm(right)
    ET.SubElement(
        w,
        "camera",
        name="overview",
        pos=numbers(eye),
        xyaxes=numbers((*right, *np.cross(right, forward))),
        fovy="48",
    )
    # Independent inspection cameras through physical windows. Orthographic
    # elevations share one mm scale at every depth; the policy keeps its three cameras.
    for name, eye, axes, height in (
        ("plan", (HUB[0]-0.016, -0.010, FLOOR+0.40), "0 -1 0 1 0 0", 0.38),
        ("back", (HUB[0]+0.27, 0, FLOOR+0.048), "0 1 0 0 0 1", 0.12),
        ("side", (HUB[0], 0.34, FLOOR+0.048), "-1 0 0 0 0 1", 0.12),
    ):
        ET.SubElement(w, "camera", name=name, pos=numbers(eye), xyaxes=axes,
                      projection="orthographic", fovy=str(height))
    cup_surface(assets)
    cabinet(w, drives, assets)
    loose_parts(w)
    add_arms(root)
    return ET.tostring(root, encoding="unicode")
