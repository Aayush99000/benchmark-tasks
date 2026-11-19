"""How hard the cannula's tubing pushes back as it goes onto the flowmeter's barbed nipple: an estimate from the
parts' sizes and the tubing's material, used by the nipple model in ``oxygen_flowmeter.py``. Pure Python.

    python barbfit.py      # prints the estimate and its range

The parts (sources in the README, "The barbs' push-back"):
- the nipple: a "6 mm (1/4 inch) barbed fitting" (WHO, COVID-19 Technical specifications for procurement of oxygen
  therapy and monitoring devices, WHO/2020-nCoV/MedDev/TS/O2T.V1, 2020): three barbs, each a cone from 3.6 mm across at its front to 6.0 mm at its crest over 4 mm, one every 5 mm
  [estimate: the cone's front size and length, the barbs' pitch];
- the tubing: oxygen tubing with a 5 mm bore ("Tubing (interior diameter 5mm)", the same WHO document), 8 mm across,
  soft PVC.

The model: the tubing's wall is a thick elastic cylinder stretched over each barb's cone (Lame's solution, rubber-like
so Poisson's ratio 0.5). Over the part of the cone wider than the bore, the stretch, and so the pressure, grows from
nothing to 0.5 mm of radius at the crest. Pushing the tubing up the cone takes that pressure times (the cone's slope
plus friction); behind the crest the tubing snaps back down the barb's steep back. Every barb already under the
tubing keeps costing that much as the tubing slides on, because each bit of the wall passing over it is stretched
again. So the push-back steps up at each barb: about 4 N, 8 N, then 13 N over the last.
"""

from __future__ import annotations

import math

MM = 0.001

# -- the parts --------------------------------------------------------------------------------------------------------
BORE_R = 2.5 * MM  # the tubing's bore, 5 mm (WHO)
TUBE_R = 4.0 * MM  # its outside, 8 mm
TIP_R = 1.8 * MM  # each barb's front [estimate]
CREST_R = 3.0 * MM  # each barb's crest, 6 mm (WHO: "6 mm (1/4 inch) barbed fitting")
CONE_L = 4.0 * MM  # each barb's cone [estimate]
PITCH = 5.0 * MM  # one barb every 5 mm [estimate]
N_BARBS = 3
BARB_AT = tuple(k * PITCH for k in range(N_BARBS))  # each cone's front, from the nipple's tip
CRESTS = tuple(z + CONE_L for z in BARB_AT)  # 4, 9 and 14 mm in

# -- the tubing's material --------------------------------------------------------------------------------------------
SHORE_A = 70.0  # [estimate: soft medical PVC tubing is about 65-80 Shore A]
POISSON = 0.5  # rubber-like
MU = 0.3  # PVC on the nipple's plastic [estimate]


def shore_a_to_modulus(shore_a: float) -> float:
    """Young's modulus (Pa) from Shore A hardness (Gent, 1958: E = 0.0981 (56 + 7.66 s) / (0.137505 (254 - 2.54 s))
    MPa)."""
    return 1e6 * 0.0981 * (56.0 + 7.66 * shore_a) / (0.137505 * (254.0 - 2.54 * shore_a))


def wall_pressure(stretch: float, e: float = None, nu: float = POISSON) -> float:
    """The pressure (Pa) between the wall and the cone where the bore is stretched ``stretch`` (m) of radius: Lame's
    thick cylinder (bore ``BORE_R``, outside ``TUBE_R``) on a rigid core."""
    e = shore_a_to_modulus(SHORE_A) if e is None else e
    a, b = BORE_R, TUBE_R
    stiff = e * (b * b - a * a) / ((1.0 + nu) * (b * b + (1.0 - 2.0 * nu) * a * a))
    return stiff * stretch / a


def contact_length() -> float:
    """How much of each cone (m, along it) is wider than the bore: where the wall touches it."""
    return CONE_L * (CREST_R - BORE_R) / (CREST_R - TIP_R)


def barb_force(e: float = None, mu: float = MU, n: int = 400) -> float:
    """The push (N) to slide the tubing over one barb: the wall's pressure over the cone, times its slope plus
    friction (a cone of half angle a: (tan a + mu) / (1 - mu tan a) per unit of radial load)."""
    tan_a = (CREST_R - TIP_R) / CONE_L
    lc = contact_length()
    radial = 0.0
    for i in range(n):  # along the contact, from where the cone first meets the bore up to the crest
        x = (i + 0.5) / n * lc
        stretch = (CREST_R - BORE_R) * x / lc
        r = BORE_R + stretch
        radial += wall_pressure(stretch, e) * 2.0 * math.pi * r * (lc / n)
    return radial * (tan_a + mu) / (1.0 - mu * tan_a)


F_BARB = barb_force()  # about 4.2 N


def push_back(depth: float, f_barb: float = F_BARB) -> float:
    """The push (N) needed to move the tubing further on with its mouth ``depth`` (m) past the nipple's tip: each
    barb's share grows with the square of how far up its cone the mouth has come (the stretch, and the length
    stretched, both grow), and stays once the mouth is over its crest."""
    lc = contact_length()
    total = 0.0
    for z in CRESTS:
        u = (depth - (z - lc)) / lc
        total += f_barb * min(1.0, max(0.0, u)) ** 2
    return total


def estimate_range() -> dict:
    """The estimate and how it moves with the two least certain inputs."""
    out = {"central": (SHORE_A, MU, F_BARB, N_BARBS * F_BARB)}
    for s, mu in ((65.0, 0.2), (65.0, 0.4), (80.0, 0.2), (80.0, 0.4)):
        f = barb_force(shore_a_to_modulus(s), mu)
        out[f"shore {s:.0f}, mu {mu}"] = (s, mu, f, N_BARBS * f)
    return out


if __name__ == "__main__":
    print(f"Young's modulus at {SHORE_A:.0f} Shore A: {shore_a_to_modulus(SHORE_A) / 1e6:.2f} MPa")
    print(f"wall pressure at the crest: {wall_pressure(CREST_R - BORE_R) / 1e6:.3f} MPa over "
          f"{contact_length() / MM:.2f} mm of each cone")
    for name, (s, mu, f, peak) in estimate_range().items():
        print(f"{name:>18}: {f:5.2f} N per barb, {peak:5.2f} N over all three")
    for d in (0, 3, 4, 6, 8, 9, 12, 14, 15, 20):
        print(f"  mouth {d:2d} mm on: {push_back(d * MM):5.2f} N")
