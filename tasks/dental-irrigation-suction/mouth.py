"""The mouth: geometry of a supine adult phantom head (typodont in a printed shell), and the reduced water model.

Pure NumPy, no simulator: task.py builds the scene from these numbers, and the rules here score a trial. Units: metres and
seconds in the world, millilitres for water. Positions in the world are station coordinates (x forward, y left, z up);
mouth coordinates (p, l, c) are millimetres from the lower incisal point: p toward the back along the lower occlusal
plane, l toward the patient's left, c along the lower occlusal normal (toward the upper jaw).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

TOP = 0.7475  # station table top (yam_arena TABLE_TOP["station"])

# -- pose of the head ------------------------------------------------------------------------
# Supine, crown of the head toward the robot (-x), face up. The mouth is open, so the lower occlusal plane slopes
# down toward the throat at TILT from the horizontal.
TILT = math.radians(35.0)
O = np.array([0.280, 0.0, TOP + 0.060])  # lower incisal point (between the lower central incisors' edges)
P_AX = np.array([-math.cos(TILT), 0.0, -math.sin(TILT)])  # toward the back of the mouth
L_AX = np.array([0.0, 1.0, 0.0])  # patient's left
N_AX = np.array([-math.sin(TILT), 0.0, math.cos(TILT)])  # lower occlusal normal
OPEN_DEG = 28.0  # the upper jaw turned open about the hinge: 45 mm between the incisal edges
HINGE = (88.0, 28.0)  # (p, c) mm of the jaw hinge (condyles)


def world(p, l, c) -> np.ndarray:
    """Mouth millimetres to world metres. Broadcasts."""
    p, l, c = (np.asarray(v, float) for v in (p, l, c))
    return O + (p[..., None] * P_AX + l[..., None] * L_AX + c[..., None] * N_AX) / 1000.0


def mouth(xyz) -> np.ndarray:
    """World metres to mouth millimetres (p, l, c)."""
    d = (np.asarray(xyz, float) - O) * 1000.0
    return np.stack([d @ P_AX, d @ L_AX, d @ N_AX], axis=-1)


def upper(p, l, c) -> np.ndarray:
    """A point of the upper jaw, given in its closed position (mouth mm), to world: turned open about the hinge."""
    a = math.radians(OPEN_DEG)
    dp, dc = p - HINGE[0], c - HINGE[1]
    # turn so that points in front of the hinge rise toward +c
    pp = HINGE[0] + dp * math.cos(a) + dc * math.sin(a)
    cc = HINGE[1] - dp * math.sin(a) + dc * math.cos(a)
    return world(pp, l, cc)


# -- teeth (adult sizes, Kilgore/Nissin 200-type typodont) ----------------------------------------
# name: (p, |l|, mesiodistal, buccolingual, crown height) in mm, crowns of the lower arch from c=-height to c=0.
LOWER = {
    "1": (1.5, 2.7, 5.0, 6.0, 9.0),
    "2": (4.0, 8.0, 5.5, 6.0, 9.0),
    "3": (8.0, 13.0, 7.0, 7.5, 10.0),
    "4": (14.0, 17.5, 7.0, 8.0, 8.5),
    "5": (21.0, 20.5, 7.0, 8.5, 8.0),
    "6": (30.0, 23.0, 11.0, 10.5, 7.5),
    "7": (41.0, 25.0, 10.5, 10.0, 7.0),
}
UPPER = {
    "1": (-1.0, 4.3, 8.5, 7.0, 10.5),
    "2": (2.5, 10.5, 6.5, 6.0, 9.0),
    "3": (6.5, 15.5, 7.5, 8.0, 10.0),
    "4": (13.5, 19.5, 7.0, 9.0, 8.5),
    "5": (20.5, 22.0, 6.5, 9.0, 7.5),
    "6": (29.5, 24.5, 10.0, 11.0, 7.0),
    "7": (40.0, 25.5, 9.0, 10.5, 6.5),
}
TARGETS = ("35", "36", "37")  # FDI: lower left second premolar, first and second molars
FOSSA_C = -1.5  # the occlusal fossa, where each blue dot sits


def target_spot(name: str) -> np.ndarray:
    p, l, *_ = LOWER[name[1]]
    return world(p, l, FOSSA_C)


SPOTS = np.array([target_spot(t) for t in TARGETS])
SPOT_R = 0.0035  # a stream landing within this of a dot's centre rinses it
SPOT_DOSE = 0.40  # mL that must land there to rinse the dot off (bench-calibrated, see README)

# -- soft tissue and the floor -----------------------------------------------------------------
TONGUE = dict(center=(36.0, 0.0, -6.0), semi=(30.0, 13.0, 10.0))  # ellipsoid, mm in mouth axes
TONGUE_GIVE = 1.5  # mm the soft tongue gives under a light touch: its collider is this much inside its surface
FLOOR_C = -14.0  # floor of the mouth beside the tongue and under the vestibules
# Where water collects. The head is reclined, so the floor of the mouth slopes down toward the throat at TILT. Water runs
# back along it and lies on the floor against the throat sill, either side of the tongue: a wedge as deep as the sill at
# its foot, whose level surface meets the floor SILL_H / tan(TILT) in front of it. The cheek vestibules drain into it.
SILL_P = 64.0  # p of the throat sill's front face
SILL_H = 8.5  # mm the sill stands above the floor
SILL_REACH = SILL_H / math.tan(TILT)  # 12.1 mm: how far forward of the sill the water reaches when it is full
SILL_L = (8.0, 32.0)  # |l| of the water against the sill on each side: from the tongue's side to the vestibule
SILL_CAP = SILL_H * SILL_REACH / 2 * (SILL_L[1] - SILL_L[0]) / 1000  # mL a side holds below the sill's top: about 1.24
VESTIBULE = dict(p=(12.0, 48.0), l=(30.0, 37.0))  # the cheek vestibule on each side, beside the premolars and molars
VESTIBULE_TAU = 2.5  # s for the vestibule's water to run back to the sill on its side
FILM_TAU = 0.4  # s for water on teeth and tongue to run off
TO_VESTIBULE = 0.35  # share of what lands on a tooth or the cheek side that runs to the vestibule (the rest to the sill)
SUCTION = 4.0  # mL/s a submerged saliva ejector draws
STREAM_V = 0.5  # m/s, a gently squeezed bulb's stream
SUCK_BAND = 0.008  # m: the centre of the ejector's 8 mm tip must be within this above the floor, where the water lies

# The opening between the lips, an ellipse in the lips' plane (world): centre, axes
LIP_CENTRE = (world(-3.0, 0, -3.0) + upper(-4.0, 0, 3.0)) / 2
_lip_dir = upper(-4.0, 0, 3.0) - world(-3.0, 0, -3.0)
LIP_SEMI_V = float(np.linalg.norm(_lip_dir)) / 2  # along the opening (lower to upper lip)
LIP_V = _lip_dir / np.linalg.norm(_lip_dir)
LIP_SEMI_W = 0.032  # corner to corner, 64 mm, with the cheeks retracted as for posterior work
LIP_POWER = 4.0  # the opening's outline: |u/a|^4 + |w/b|^4 = 1, a rounded rectangle
LIP_NORMAL = np.cross(LIP_V, L_AX)
LIP_NORMAL = LIP_NORMAL if LIP_NORMAL[2] > 0 else -LIP_NORMAL

# -- the tools ----------------------------------------------------------------------------------
BULB_D = 0.052  # Medline DYND70280 2 oz bulb, nominal
BULB_H = 0.060
TIP_L = 0.045  # tapered nozzle below the bulb
ML_PER_MM = 1.0  # mL dispensed per mm of squeeze across the bulb (bench-calibrated: about 40 mL over its full 35-40 mm)
BULB_WATER = 20.0  # mL in the bulb at the start
EJ_TIP = 0.0  # the ejector tip is its body point TIP (task.py)

# Tool axes, pointing toward the tool's tip, as held in the demo: the bulb leans 52 deg back (its top toward the chin) and
# its nozzle points a little toward the patient's left, across from the right side of the opening, which keeps it clear of
# the ejector at the left of the opening; the ejector's working leg leans 20 deg back.
_b = np.array([-math.sin(math.radians(52)), 0.41, -math.cos(math.radians(52))])
BULB_AXIS = _b / np.linalg.norm(_b)
EJ_AXIS = np.array([-math.sin(math.radians(20)), 0.0, -math.cos(math.radians(20))])
EJ_TARGET = (58.0, 16.0, FLOOR_C + 5.0)  # mouth mm: where the ejector's tip works, on the left, 6 mm in front of the sill, 5 mm above the floor
EJ_LEG = 0.095  # m, working leg from the bend to the tip
EJ_REACH = 0.060  # m, from the bend forward (+x) to the handle's front end

# -- limits for the score ---------------------------------------------------------------------
THROAT_MAX = 0.20  # mL
SPILL_MAX = 0.50
LEFT_MAX = 0.30  # water left in the mouth once both tools are out


def aim(target, axis, standoff: float) -> np.ndarray:
    """Where to hold the nozzle's end, `standoff` from `target` along `axis`, so that a stream at STREAM_V lands on it:
    raised by the stream's fall over that distance."""
    t = standoff / STREAM_V
    return np.asarray(target, float) - np.asarray(axis, float) * standoff + np.array([0.0, 0.0, 9.81 * t * t / 2])


def in_lips(point_a, point_b) -> bool | None:
    """Does the segment a->b pass through the opening between the lips? None if it does not cross the lips' plane."""
    a, b = np.asarray(point_a, float), np.asarray(point_b, float)
    da, db = (a - LIP_CENTRE) @ LIP_NORMAL, (b - LIP_CENTRE) @ LIP_NORMAL
    if da * db > 0:
        return None
    t = da / (da - db) if da != db else 0.0
    x = a + t * (b - a) - LIP_CENTRE
    u, w = x @ LIP_V, x @ L_AX
    return abs(u / LIP_SEMI_V) ** LIP_POWER + abs(w / LIP_SEMI_W) ** LIP_POWER <= 1.0


def landing(tip, axis) -> tuple[str, np.ndarray]:
    """Where a slow stream from `tip` along `axis` (unit, world) lands: "face", "spot<i>", "toothL/R", "tongueL/R" or
    "floorL/R", and the point. The stream leaves at STREAM_V and falls under gravity; it is followed in 1 mm steps against
    simple surfaces: the face (the lips' plane outside the opening), the crowns of the lower teeth, the tongue and the floor."""
    v = np.asarray(axis, float) / max(np.linalg.norm(axis), 1e-9) * STREAM_V
    p = np.asarray(tip, float).copy()
    inside = (p - LIP_CENTRE) @ LIP_NORMAL < 0
    for _ in range(150):
        dt = 0.001 / max(np.linalg.norm(v), 1e-6)
        q = p + v * dt
        v = v + np.array([0.0, 0.0, -9.81]) * dt
        if not inside:
            through = in_lips(p, q)
            if through is False:
                return "face", q
            if through:
                inside = True
            p = q
            continue
        m = mouth(q)
        for k, (pp, ll, md, bl, h) in LOWER.items():
            for s in (1, -1):
                if abs(m[0] - pp) <= md / 2 and abs(m[1] - s * ll) <= bl / 2 and -h <= m[2] <= FOSSA_C:
                    name = ("3" if s > 0 else "4") + k
                    if name in TARGETS:
                        return "spot" + str(TARGETS.index(name)), q
                    return "tooth" + ("L" if s > 0 else "R"), q
        t = TONGUE
        if sum(((m[i] - t["center"][i]) / t["semi"][i]) ** 2 for i in range(3)) <= 1.0:
            return "tongue" + ("L" if m[1] >= 0 else "R"), q
        if m[2] <= FLOOR_C or abs(m[1]) > 34:
            return "floor" + ("L" if m[1] >= 0 else "R"), q
        p = q
    return ("floor" + ("L" if mouth(p)[1] >= 0 else "R")) if inside else "face", p


def suction_zone(ejector_tip) -> tuple[str, str] | None:
    """Which water the ejector's tip is in: ("sill", side) on the floor against the throat sill, ("vestibule", side) in a
    cheek vestibule, or None. The tip's centre must be within SUCK_BAND above the floor."""
    p, l, c = mouth(ejector_tip)
    if not FLOOR_C - 1.0 <= c <= FLOOR_C + SUCK_BAND * 1000:
        return None
    side = "left" if l >= 0 else "right"
    if SILL_P - SILL_REACH <= p <= SILL_P and SILL_L[0] <= abs(l) <= SILL_L[1]:
        return "sill", side
    if VESTIBULE["p"][0] <= p <= VESTIBULE["p"][1] and VESTIBULE["l"][0] <= abs(l) <= VESTIBULE["l"][1]:
        return "vestibule", side
    return None


@dataclass
class Water:
    """Every millilitre is somewhere: in the bulb, on surfaces (films), in a vestibule or against the throat sill, in the throat cup, on the
    towel (spill), or in the suction trap. `step` moves it."""

    t: float = 0.0
    squeeze_max: float = 0.0  # mm: the bulb dispenses only past its deepest squeeze so far (it sucks air back on release)
    dispensed: float = 0.0
    film: dict = field(default_factory=lambda: {"left": 0.0, "right": 0.0})
    vestibule: dict = field(default_factory=lambda: {"left": 0.0, "right": 0.0})
    sill: dict = field(default_factory=lambda: {"left": 0.0, "right": 0.0})
    throat: float = 0.0
    spill: float = 0.0
    dripped: float = 0.0  # squeezed out away from the head, into a holder's drip cup
    trap: float = 0.0
    dose: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    last: str = ""
    last_point: np.ndarray = field(default_factory=lambda: np.zeros(3))
    flow: float = 0.0
    sucking: str = ""

    def in_mouth(self) -> float:
        return sum(self.film.values()) + sum(self.vestibule.values()) + sum(self.sill.values())

    def rinsed(self) -> list[bool]:
        return [d >= SPOT_DOSE - 1e-9 for d in self.dose]

    def step(self, dt: float, squeeze_mm: float, tip, axis, ejector_tip, away: bool = False) -> None:
        self.t += dt
        # 1. the bulb
        out = 0.0
        if squeeze_mm > self.squeeze_max:
            out = min((squeeze_mm - self.squeeze_max) * ML_PER_MM, BULB_WATER - self.dispensed)
            self.squeeze_max = squeeze_mm
        self.dispensed += out
        self.flow = out / dt
        if out > 0 and away:
            self.dripped += out
            self.last, self.last_point = "away", np.asarray(tip, float)
        elif out > 0:
            where, point = landing(tip, axis)
            self.last, self.last_point = where, point
            if where == "face":
                self.spill += out
            else:
                side = "left" if where.endswith("L") or where.startswith("spot") else "right"
                if where.startswith("spot"):
                    i = int(where[4:])
                    if np.linalg.norm(point - SPOTS[i]) <= SPOT_R + 0.002:
                        self.dose[i] += out
                if where.startswith("floor"):
                    self.sill[side] += out
                else:
                    self.film[side] += out
        else:
            self.last = ""
        # 2. run-off
        k = 1.0 - math.exp(-dt / FILM_TAU)
        kv = 1.0 - math.exp(-dt / VESTIBULE_TAU)
        for s in ("left", "right"):
            run = self.film[s] * k
            self.film[s] -= run
            self.vestibule[s] += run * TO_VESTIBULE
            self.sill[s] += run * (1 - TO_VESTIBULE)
            back = self.vestibule[s] * kv
            self.vestibule[s] -= back
            self.sill[s] += back
        # 3. suction: the ejector draws from the water its tip is in, against the sill or in a vestibule
        self.sucking = ""
        if ejector_tip is not None:
            where = suction_zone(ejector_tip)
            if where:
                part, s = where
                store = self.sill if part == "sill" else self.vestibule
                take = min(store[s], SUCTION * dt)
                store[s] -= take
                self.trap += take
                self.sucking = s if part == "sill" else "vestibule_" + s
        # 4. overflow to the throat
        for s in ("left", "right"):
            over = max(0.0, self.sill[s] - SILL_CAP)
            self.sill[s] -= over
            self.throat += over

    def balance(self) -> float:
        return self.dispensed - self.in_mouth() - self.throat - self.spill - self.trap - self.dripped


def score(water: Water, tools_out: bool, both_home: bool) -> float:
    """task.json's scoring, from the water ledger and the tools' state now (latched failures are in the ledger itself)."""
    if water.throat > THROAT_MAX + 1e-9 or water.spill > SPILL_MAX + 1e-9:
        return 0.0
    if not tools_out:
        return 0.0
    if water.in_mouth() > LEFT_MAX + 1e-9:
        return 0.0
    n = sum(water.rinsed())
    if n == 3 and both_home:
        return 1.0
    return 0.25 * n
