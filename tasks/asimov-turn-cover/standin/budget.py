"""Mass budget of the stand-in: mass, centre of mass, yaw inertia against the task's body. Run: python standin/budget.py
Axes (cm in the text, m here), from the body's centre of mass: x forward (front of the torso), y left, z up. Targets are upper_body.npz
(Menlo's URDF, torso+arms+head, arms straight): 18.276 kg, yaw inertia 0.2566 kg m2 about the hook's vertical axis through the centre of mass."""
import math
M_T, I_T = 18.276305, 0.256632
PLY, PINE, STEEL = 680.0, 500.0, 7850.0  # kg/m3: 12 mm birch ply, pine, mild steel (weigh your own and trim)
P = []  # name, mass, x, y, z, own yaw inertia


def box(name, rho, dx, dy, dz, x, y, z):
    m = rho * dx * dy * dz
    P.append((name, m, x, y, z, m * (dx * dx + dy * dy) / 12))


t, X, Y, Z, cx, cz = 0.012, 0.220, 0.250, 0.380, 0.0075, 0.0215  # torso box 220 deep x 250 wide x 380 high, open back
box("front 12x250x380", PLY, t, Y, Z, cx + X / 2 - t / 2, 0, cz)
box("side L 220x12x380", PLY, X, t, Z, cx, Y / 2 - t / 2, cz)
box("side R 220x12x380", PLY, X, t, Z, cx, -Y / 2 + t / 2, cz)
box("top 220x226x12", PLY, X, Y - 2 * t, t, cx, 0, cz + Z / 2 - t / 2)
box("bottom 220x226x12", PLY, X, Y - 2 * t, t, cx, 0, cz - Z / 2 + t / 2)
box("cover stops (2 side 340 + 2 end 200, 12x10)", PLY, t, 0.020, 0.34, 0.013, 0, 0)
box("cover stops, top and bottom", PLY, t, 0.200, 0.020, 0.013, 0, 0)
box("ballast cleats 2x(20x12x70)", PLY, 0.07, 0.040, 0.012, 0.060, 0, 0.0195)
box("head block pine 90x130x180", PINE, 0.09, 0.13, 0.18, 0.012, 0, 0.302)
box("ballast: 7 steel plates 10x100x100, stacked along x", STEEL, 0.07, 0.10, 0.10, 0.060, 0, 0.07)
ARM_Y, ARM_X, ARM_Z = 0.1365, -0.053, -0.09  # steel bar 60 (x) x 20 (y) x 450 (z), one each side, hung from the shoulders
for s in (1, -1):
    box("arm bar steel 60x20x450", STEEL, 0.06, 0.02, 0.45, ARM_X, s * ARM_Y, ARM_Z)
M = sum(p[1] for p in P)
com = [sum(p[1] * p[k] for p in P) / M for k in (2, 3, 4)]
Iz = sum(p[5] + p[1] * (p[2] ** 2 + p[3] ** 2) for p in P)
for p in P:
    print(f"{p[0]:55s} {p[1]:6.3f} kg  at x {100*p[2]:6.1f} y {100*p[3]:6.1f} z {100*p[4]:6.1f} cm")
print(f"total {M:.3f} kg (target {M_T:.3f}; add {M_T-M:+.3f} kg of fender washers on the head to trim)")
print(f"centre of mass x {100*com[0]:+.2f} y {100*com[1]:+.2f} z {100*com[2]:+.2f} cm (target 0, 0, 0)")
Iz += (M_T - M) * 0.012 ** 2
print(f"yaw inertia {Iz:.4f} kg m2 (target {I_T:.4f}, {100*(Iz/I_T-1):+.1f} %)")
