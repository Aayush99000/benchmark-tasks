"""Dimensioned drawings of the grinder mock's adjustable parts, from the same numbers the task file uses.

    python make_drawings.py        # writes drawings/rest_slide_and_cam_clamp.png and drawings/tongue_guard_guide.png

Needs matplotlib. All dimensions in millimetres. The simulation's own sizes (guards.py) are imported, so the
drawings and the simulated parts cannot drift apart; the parts the simulation leaves out (channel, bolt, cam
washers, guide angles, thumb screw) are drawn with the sizes listed in BUILD.md.
"""

import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
try:  # the task file imports yam_arena; fall back to its numbers if that isn't installed
    import guards as g  # noqa: E402

    WHEEL_R, WHEEL_T = g.WHEEL_R * 1000, g.WHEEL_T * 1000
    AXLE_H = (g.AXLE_Z - g.TOP) * 1000
    REST = [v * 1000 for v in g.REST_SIZE]
    POST = [v * 1000 for v in g.POST_SIZE]
    STEM = [v * 1000 for v in g.STEM_SIZE]
    BAR = [v * 1000 for v in g.LEVER_BAR]
    TONGUE = [v * 1000 for v in g.TONGUE_SIZE]
    GAP0 = g.START_GAP_M * 1000
    REST_MAX, TONGUE_MAX = g.REST_MAX_M * 1000, g.TONGUE_MAX_M * 1000
except Exception:  # noqa: BLE001
    WHEEL_R, WHEEL_T, AXLE_H = 55.0, 25.0, 68.0
    REST, POST, STEM, BAR, TONGUE = [40, 70, 10], [20, 20, 50], [14, 14, 18], [12, 45, 12], [35, 50, 6]
    GAP0, REST_MAX, TONGUE_MAX = 25.0, 3.0, 6.0

# Parts the simulation leaves out (BUILD.md lists them)
CH_W, CH_H, CH_T, CH_L = 25.0, 15.0, 2.0, 100.0  # steel U-channel: outside width, height, wall, length
SPACER = 6.0  # spacer strips under the channel, so the bolt head clears the table
SLOT_W, SLOT_L = 5.5, 45.0  # slot in the channel floor for the clamp bolt
FELT = 3.0
CAM_D, CAM_RISE, CAM_LOW, CAM_UP = 18.0, 2.0, 3.0, 5.0  # face-cam washers: diameter, rise, lower/upper heights
POST_OFF = 25.0  # post centre from the platform centre, toward the outer end
SLOT_C = 5.0  # floor slot centre in the top view: the bolt travels from -2.5 (start) to +22.5 (rest at the wheel)
LEVER_OFF = 27.5  # cam lever axis from the platform centre


def dim(ax, p0, p1, text, off=6.0, horiz=True, fs=7):
    """A dimension line between p0 and p1, offset perpendicular by off, with the text at its middle."""
    (x0, y0), (x1, y1) = p0, p1
    if horiz:
        y = max(y0, y1) + off
        ax.annotate("", (x0, y), (x1, y), arrowprops=dict(arrowstyle="<->", lw=0.6))
        ax.plot([x0, x0], [y0, y], lw=0.3, color="0.4")
        ax.plot([x1, x1], [y1, y], lw=0.3, color="0.4")
        ax.text((x0 + x1) / 2, y + 1.0, text, ha="center", va="bottom", fontsize=fs)
    else:
        x = max(x0, x1) + off
        ax.annotate("", (x, y0), (x, y1), arrowprops=dict(arrowstyle="<->", lw=0.6))
        ax.plot([x0, x], [y0, y0], lw=0.3, color="0.4")
        ax.plot([x1, x], [y1, y1], lw=0.3, color="0.4")
        ax.text(x + 1.0, (y0 + y1) / 2, text, ha="left", va="center", fontsize=fs, rotation=90)


def box(ax, x, y, w, h, **kw):
    kw.setdefault("fill", False)
    kw.setdefault("lw", 0.9)
    ax.add_patch(Rectangle((x, y), w, h, **kw))


def rest_sheet(path):
    fig, axs = plt.subplots(1, 3, figsize=(16, 6.2), gridspec_kw=dict(width_ratios=[1.35, 1.0, 0.8]))
    fig.suptitle("Tool rest slide and quarter-turn cam clamp (mm). Rest shown at its 25 mm start gap; "
                 "OSHA 1910.215(a)(4) target: gap <= 3 mm, rest clamped", fontsize=10)

    # -- side view: looking along the axle (y to the right = toward the wheel, z up from the table) --
    ax = axs[0]
    ax.set_title("Side view (along the axle)", fontsize=9)
    rim_y = 0.0  # the wheel's rim at the rest's height
    plat_in = rim_y - GAP0  # platform inner edge
    plat_out = plat_in - REST[1]
    plat_z0 = AXLE_H - REST[2]
    # wheel (circle centred on the axle, its rim at rim_y)
    ax.add_patch(Circle((rim_y + WHEEL_R, AXLE_H), WHEEL_R, fill=False, lw=0.9, ls="--"))
    ax.text(rim_y + WHEEL_R, AXLE_H, "wheel\n(110 dia.)", ha="center", va="center", fontsize=7)
    # platform, post
    box(ax, plat_out, plat_z0, REST[1], REST[2], fc="0.85", fill=True)
    post_c = (plat_in + plat_out) / 2 - POST_OFF
    box(ax, post_c - POST[1] / 2, plat_z0 - POST[2], POST[1], POST[2], fc="0.7", fill=True)
    # channel (section: floor and walls behind), spacers
    ch_z0 = SPACER
    box(ax, post_c - CH_L / 2, ch_z0, CH_L, CH_T, fc="0.55", fill=True)
    box(ax, post_c - CH_L / 2, ch_z0, CH_L, CH_H)
    box(ax, post_c - CH_L / 2, 0, 12, SPACER, fc="0.9", fill=True)
    box(ax, post_c + CH_L / 2 - 12, 0, 12, SPACER, fc="0.9", fill=True)
    ax.plot([post_c - 80, rim_y + 2 * WHEEL_R + 10], [0, 0], color="k", lw=1.2)
    ax.text(post_c - 78, -5, "table", fontsize=7)
    # bolt, cam washers (lower one in a counterbore in the platform, upper one the stem's base), lever
    lev_y = (plat_in + plat_out) / 2 - LEVER_OFF
    ax.plot([lev_y, lev_y], [ch_z0 - 5, AXLE_H + STEM[2] + BAR[2] + 3], lw=1.6, color="tab:blue")
    box(ax, lev_y - 4, ch_z0 - 5, 8, 3, fc="tab:blue", fill=True)  # bolt head under the floor
    box(ax, lev_y - 4.5, ch_z0 - 2, 9, 2, fc="tab:cyan", fill=True)  # disc springs
    box(ax, lev_y - CAM_D / 2, AXLE_H - CAM_LOW - CAM_RISE, CAM_D, CAM_LOW + CAM_RISE, fc="tab:orange", fill=True)
    box(ax, lev_y - STEM[1] / 2, AXLE_H, STEM[1], STEM[2], fc="0.15", fill=True)
    box(ax, lev_y - CAM_D / 2, AXLE_H, CAM_D, CAM_UP, fc="tab:red", fill=True)
    box(ax, lev_y - BAR[1] / 2, AXLE_H + STEM[2], BAR[1], BAR[2], fc="0.15", fill=True)
    box(ax, lev_y - 4, AXLE_H + STEM[2] + BAR[2], 8, 3, fc="tab:blue", fill=True)  # nyloc nut
    # dimensions
    dim(ax, (plat_out, AXLE_H + STEM[2] + BAR[2]), (plat_in, AXLE_H), f"platform {REST[1]:.0f}", off=8)
    dim(ax, (plat_in, AXLE_H), (rim_y, AXLE_H), f"start gap {GAP0:.0f}", off=4)
    ax.text(post_c - POST[1] / 2 - 3, plat_z0 - POST[2] / 2 - 6, "post\n20 x 20 x 50\nsteel", ha="right", va="center",
            fontsize=7)
    dim(ax, (rim_y + 2 * WHEEL_R + 5, 0), (rim_y + 2 * WHEEL_R + 5, AXLE_H), f"axle {AXLE_H:.0f} above table",
        horiz=False, off=4)
    dim(ax, (post_c + POST[1] / 2 + 3, plat_z0 - POST[2]), (post_c + POST[1] / 2 + 3, plat_z0), f"{POST[2]:.0f}",
        horiz=False, off=2)
    dim(ax, (post_c - CH_L / 2, 0), (post_c + CH_L / 2, 0), f"U-channel {CH_L:.0f} long, on {SPACER:.0f} spacers",
        off=-16)
    ax.text(plat_out - 4, plat_z0 + REST[2] / 2, "platform 40 x 70 x 10 steel\n(5 deep counterbore\nfor the lower cam)",
            ha="right", va="center", fontsize=7)
    ax.text(lev_y - BAR[1] / 2 - 2, AXLE_H + STEM[2] + BAR[2] / 2, "T-handle\n45 x 12 x 12", ha="right",
            va="center", fontsize=7)
    ax.text(lev_y + CAM_D / 2 + 3, AXLE_H + 1, "face cams (3D printed)", fontsize=7, color="tab:red")
    ax.text(post_c - CH_L / 2 - 2, ch_z0 - 3, "M5 x 90 bolt,\n2 disc springs", ha="right", fontsize=7,
            color="tab:blue")
    ax.text(post_c + POST[1] / 2 + 14, ch_z0 + CH_H - 4, "felt strip on one\nchannel wall", fontsize=7)
    ax.set_aspect("equal")
    ax.set_xlim(post_c - 115, rim_y + 2 * WHEEL_R + 30)
    ax.set_ylim(-26, AXLE_H + 55)
    ax.axis("off")

    # -- top view: the channel, platform and lever (loose along the rest, locked across it) --
    ax = axs[1]
    ax.set_title("Top view", fontsize=9)
    box(ax, -CH_W / 2, -CH_L / 2, CH_W, CH_L)
    box(ax, -CH_W / 2 + CH_T, -CH_L / 2, CH_W - 2 * CH_T, CH_L, ls=":")
    box(ax, -CH_W / 2 + CH_T, -CH_L / 2, FELT, CH_L, fc="0.8", fill=True)
    box(ax, -SLOT_W / 2, SLOT_C - SLOT_L / 2, SLOT_W, SLOT_L, ls="--")
    box(ax, -REST[0] / 2, POST_OFF - REST[1] / 2, REST[0], REST[1], fc="0.85", fill=True, alpha=0.7)
    ly = POST_OFF - LEVER_OFF + 0.0
    box(ax, -BAR[0] / 2, ly - BAR[1] / 2, BAR[0], BAR[1], fc="0.15", fill=True)
    box(ax, -BAR[1] / 2, ly - BAR[0] / 2, BAR[1], BAR[0], ls="--", lw=1.0)
    ax.text(BAR[1] / 2 + 2, ly - 12, "locked: a quarter turn\nclockwise, to its stop", fontsize=7)
    ax.text(0, ly + BAR[1] / 2 + 6, "loose: along the rest", fontsize=7, ha="center")
    ax.annotate("", (0, CH_L / 2 + 14), (0, CH_L / 2 + 2), arrowprops=dict(arrowstyle="->"))
    ax.text(2, CH_L / 2 + 8, "toward the wheel", fontsize=7)
    dim(ax, (-CH_W / 2, -CH_L / 2), (CH_W / 2, -CH_L / 2), f"{CH_W:.0f}", off=-10)
    dim(ax, (-REST[0] / 2, POST_OFF + REST[1] / 2), (REST[0] / 2, POST_OFF + REST[1] / 2), f"{REST[0]:.0f}", off=4)
    dim(ax, (CH_W / 2, SLOT_C - SLOT_L / 2), (CH_W / 2, SLOT_C + SLOT_L / 2), f"floor slot {SLOT_W} x {SLOT_L:.0f}",
        horiz=False, off=22)
    dim(ax, (-BAR[0] / 2, ly - BAR[1] / 2), (-BAR[0] / 2, ly + BAR[1] / 2), f"T-handle {BAR[1]:.0f}", horiz=False, off=-34)
    ax.set_aspect("equal")
    ax.set_xlim(-50, 62)
    ax.set_ylim(-CH_L / 2 - 20, CH_L / 2 + 25)
    ax.axis("off")

    # -- cam detail: the lower washer's face, unrolled (rise against angle) --
    ax = axs[2]
    ax.set_title("Face cam, unrolled (each washer has two ramps)", fontsize=9)
    th = [i for i in range(0, 361)]

    def rise(a):
        a = a % 180
        if a < 80:
            return CAM_RISE * a / 80
        if a < 90:
            return CAM_RISE  # flat: holds the lock
        return 0.0 if a >= 90 else CAM_RISE

    ax.plot(th, [CAM_LOW + rise(a) for a in th], color="tab:orange")
    ax.fill_between(th, 0, [CAM_LOW + rise(a) for a in th], color="tab:orange", alpha=0.25)
    for a in (90, 270):
        ax.plot([a, a], [CAM_LOW, CAM_LOW + CAM_RISE + 1.5], color="k", lw=1.2)
        ax.text(a + 3, CAM_LOW + CAM_RISE + 1.0, "stop", fontsize=7)
    ax.set_xlabel("angle (deg)", fontsize=8)
    ax.set_ylabel("height (mm)", fontsize=8)
    ax.text(5, 1.0, f"lower washer {CAM_D:.0f} dia., 5.5 bore; ramp 0-80 deg rises {CAM_RISE:.0f}, "
                     "flat 80-90 deg, stop wall at 90 deg", fontsize=6.5)
    ax.set_xlim(0, 360)
    ax.set_ylim(0, CAM_LOW + CAM_RISE + 3)
    ax.tick_params(labelsize=7)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(path, dpi=160)
    plt.close(fig)


def tongue_sheet(path):
    """Tongue guard: a steel plate with a tab bent down at its back edge; the tab is held to the fixed guard's back
    plate by a thumbscrew through a vertical slot, with a fibre washer for friction (how bench-grinder tongue guards
    are mounted)."""
    fig, axs = plt.subplots(1, 2, figsize=(13, 6.4))
    fig.suptitle("Tongue guard and its slotted friction mount (mm). Shown at its 25 mm start gap; "
                 "OSHA 1910.215(b)(9) target: gap to the wheel's periphery <= 6 mm", fontsize=10)
    top = AXLE_H + WHEEL_R
    tz = top + GAP0  # the tongue's lower face at the start
    t_yc = 27.0  # tongue centred 27 mm left of the wheel's centre (guards.py)
    tab_w, tab_t, tab_l, screw_dz = 20.0, 2.0, 50.0, 20.0
    slot_w, slot_z0, slot_z1 = 6.5, top - 23.0, top + 12.0
    # -- front view: from the robot, so the robot's left (+y) is on the page's left: page x = -y --
    ax = axs[0]
    ax.set_title("Front view (from the robot)", fontsize=9)
    px = -t_yc  # the tongue's centre on the page
    ax.add_patch(Circle((0, AXLE_H), WHEEL_R, fill=False, lw=0.9, ls="--"))
    ax.text(0, AXLE_H, "wheel 110 dia.", ha="center", va="center", fontsize=7)
    box(ax, -55, -2, 130, 150, ls=":")  # back plate behind (y from -75 to +55, mirrored)
    ax.text(-53, 4, "fixed guard back plate (behind)", fontsize=7)
    box(ax, px - slot_w / 2, slot_z0, slot_w, slot_z1 - slot_z0, ls="--")  # slot in the back plate
    box(ax, px - tab_w / 2, tz - tab_l, tab_w, tab_l, fc="0.6", fill=True, alpha=0.6)  # tab behind the tongue
    box(ax, px - TONGUE[1] / 2, tz, TONGUE[1], TONGUE[2], fc="tab:orange", fill=True)
    ax.add_patch(Circle((px, tz - screw_dz), 3.0, color="k"))
    ax.text(px - tab_w / 2 - 3, tz - screw_dz, "M6 thumbscrew\n+ fibre washer", fontsize=7, va="center", ha="right")
    dim(ax, (px - TONGUE[1] / 2, tz + TONGUE[2]), (px + TONGUE[1] / 2, tz + TONGUE[2]), f"{TONGUE[1]:.0f}", off=6)
    dim(ax, (px + TONGUE[1] / 2 + 4, top), (px + TONGUE[1] / 2 + 4, tz), f"start gap {GAP0:.0f}", horiz=False,
        off=4)
    dim(ax, (px + tab_w / 2, slot_z0), (px + tab_w / 2, slot_z1), f"slot {slot_w} x {slot_z1 - slot_z0:.0f}",
        horiz=False, off=28)
    ax.text(px - tab_w / 2 - 3, tz - tab_l + 6, "tab 20 x 50 x 2 steel", fontsize=7, ha="right")
    ax.text(px, tz + TONGUE[2] + 14, "tongue 35 x 50 x 6 steel", fontsize=7, ha="center")
    ax.set_aspect("equal")
    ax.set_xlim(-110, 90)
    ax.set_ylim(-5, tz + 30)
    ax.axis("off")
    # -- side view: from the left (x to the right = away from the robot) --
    ax = axs[1]
    ax.set_title("Side view (from the left)", fontsize=9)
    back_x0 = 20.0  # the back plate's front face, 20 mm behind the wheel's centre plane (guards.py: 24 +/- 4)
    box(ax, -WHEEL_T / 2, AXLE_H - WHEEL_R, WHEEL_T, 2 * WHEEL_R, ls="--")
    ax.text(0, AXLE_H, "wheel", ha="center", fontsize=7)
    box(ax, back_x0, -2, 8, 150, fc="tab:blue", fill=True, alpha=0.4)
    box(ax, back_x0, slot_z0, 8, slot_z1 - slot_z0, fc="w", fill=True)  # the slot (section)
    box(ax, -TONGUE[0] / 2, tz, TONGUE[0], TONGUE[2], fc="tab:orange", fill=True)
    box(ax, TONGUE[0] / 2, tz - tab_l, tab_t, tab_l + TONGUE[2], fc="0.4", fill=True)
    ax.plot([TONGUE[0] / 2, back_x0 + 8 + 10], [tz - screw_dz, tz - screw_dz], lw=2.2, color="k")
    box(ax, back_x0 + 8, tz - screw_dz - 6, 3, 12, fc="0.2", fill=True)
    ax.text(back_x0 + 22, tz - screw_dz, "thumbscrew from behind;\nset to 3-5 N to slide", fontsize=7, va="center")
    dim(ax, (-TONGUE[0] / 2, tz), (TONGUE[0] / 2, tz), f"{TONGUE[0]:.0f}", off=-12)
    dim(ax, (TONGUE[0] / 2, top), (back_x0, top), f"{back_x0 - TONGUE[0] / 2:.1f}", off=-60)
    dim(ax, (TONGUE[0] / 2 + tab_t, tz - tab_l), (TONGUE[0] / 2 + tab_t, tz), f"tab {tab_l:.0f}", horiz=False, off=-50)
    ax.text(back_x0 + 10, 10, "fixed guard\nback plate", fontsize=7)
    ax.set_aspect("equal")
    ax.set_xlim(-60, 95)
    ax.set_ylim(-5, tz + 30)
    ax.axis("off")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(path, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    out = HERE / "drawings"
    out.mkdir(exist_ok=True)
    rest_sheet(out / "rest_slide_and_cam_clamp.png")
    tongue_sheet(out / "tongue_guard_guide.png")
    print("wrote", sorted(p.name for p in out.glob("*.png")))
