"""Steering kinematics of the version 2 front end (leading swing arm + parallel upper link), +x side, world mm (Z up, front = -Y).

Pure Python + numpy, no build:   python -I tools/steer_kin.py
Hard points: tools/_swingarm2_snapshot.py = frozen copy of the design thread's parts/swingarm2.py at 7b79b4c (read-only, not edited).

Two layouts, one 3D solver:
  A  THREAD (7b79b4c): horn 11.8 mm straight down from a servo shaft along x at (y -113.85, z 33.3); its ball on the zero-bump point
     (69, -113.85, 21.5); tie rod straight to the knuckle ball (106.5, -213.6, 14.6); kingpin vertical at x 123.
  B  CHOSEN: the same knuckle ball, tie rod and zero-bump ball, but the ball sits on a ROCKER with a vertical pin (parallel to the
     kingpin) whose output arm is the same vector as the knuckle arm. In plan, rocker + tie rod + knuckle arm + kingpin are a
     parallelogram; in side view the tie rod is parallel to the swing arm and as long as it. The carrier translates without rotating, so with
     the knuckle at the rocker's angle the vector rocker ball -> knuckle ball only swings on the arm's arc and keeps its length: the rod
     length is conserved at EVERY travel and EVERY steering angle, so bump steer is zero by construction (the rod itself turns with the
     arm swing, up to 22 deg, which sets the ball-cup cone). It stays exact for any kingpin tilt if the rocker pin is tilted the same way. The servo drives the rocker's input arm through a short drag link that is
     wholly on the body, so it sets the ratio only. The servo goes back behind the pivot axle (first snapshot position).

Labels: CALCULATED = computed here from the hard points; ASSUMED = an input not backed by geometry or a data sheet.
"""
import importlib.util
import math
import os

import numpy as np

_here = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("sa2", os.path.join(_here, "_swingarm2_snapshot.py"))
sa2 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sa2)

# ------------------------------------------------------------------------------------------------------------------------------------
# CHOSEN DESIGN (layout B), world mm, +x side; the -x side is the mirror image (x -> -x). Ready to paste into parts/swingarm2.py.
# Derived points are literals; main() re-derives them from the primary values and asserts they agree.
# ------------------------------------------------------------------------------------------------------------------------------------
STEER_DESIGN = {
    # kingpin, fixed in the carrier (the carrier translates, so the axis keeps its direction in the body frame)
    "KINGPIN_POINT": (123.0, -216.55, 14.6),      # x 123 MEASURED (old stand-in faces symmetric about it); y = wheel centre (0 trail); z = knuckle-ball plane
    "KINGPIN_AXIS": (0.0, 0.0, 1.0),              # ASSUMED vertical: 0 caster, 0 KPI. Any tilt is allowed if ROCKER_AXIS gets the same tilt
    "KINGPIN_SPAN_Z": (22.0, 59.0),               # bearing heights = knuckle ear faces (CALCULATED to clear the measured rim bore r 21.13 by 1 mm)
    "KNUCKLE_BALL": (107.31, -222.45, 14.6),      # MOVED: same 16.76 mm arm as the stand-in point, turned 20.6 deg forward of straight inboard so it is
                                                  # square to the tie rod in plan at centre (CALCULATED); stud vertical, ball inboard of the tyre face (x < 113)
    # tie rod: M4 threaded rod (M3 buckles at margin 1.05), ball ends with a >= 27 deg misalignment cone; stud vertical at both ends
    "TIE_ROD_INNER_BALL": (69.81, -122.70, 21.5), # zero-bump point: yz = PIVOT + KNUCKLE_BALL - TIP, x = KNUCKLE_BALL x - 37.5 (CALCULATED); stud down
    "TIE_ROD_L": 106.79,                          # ball to ball (CALCULATED)
    # rocker: vertical pin on a body bracket under the arm root
    "ROCKER_AXIS": (0.0, 0.0, 1.0),               # parallel to KINGPIN_AXIS (required)
    "ROCKER_PIN": (85.5, -116.8, 21.5),           # = KINGPIN_POINT + (TIE_ROD_INNER_BALL - KNUCKLE_BALL) (CALCULATED)
    "ROCKER_PIN_SPAN_Z": (19.0, 33.0),            # rocker hub (bush) z 19..33: hub bottom 19.2 mm off the ground at 10 deg lean (CALCULATED)
    "ROCKER_PLATES_Z": ((27.0, 30.0), (30.0, 33.0)),  # tie-rod arm plate, drag-link arm plate; both balls hang under their plate on 5.5 mm studs
    "ROCKER_PLATE_HALF_W": 3.5,                   # plate half width (ends round, r 3.5 round each ball stud)
    "ROD_END_HOUSINGS": {"tie": (5.0, 5.0, 3.0, 8.0), "drag": (3.75, 4.0, 2.5, 6.0)},   # (r, height, shank r, shank length) M4 / M3 rod ends, from the thread
    "ROCKER_IN_BALL": (66.0, -109.70, 24.5),      # drag-link ball, stud down from the upper plate; input arm 20.75 mm, 20 deg REARWARD of straight
                                                  # inboard (40.6 deg from the tie-rod arm: balls 13.55 mm apart in plan); drag link along y in plan
    # servo: low-profile AGFRC B44BLS class lying on the deck, output along x, back behind the pivot axle (first snapshot position)
    "SERVO_X": (31.5, 58.5),                      # thread's box x (horn plate x 59.2 .. 61.5)
    "SERVO_SHAFT": (-95.1, 38.3),     # shifted 3.3 rearward (worst-case servo ears, NOTES 27); was (-98.4, 38.3)                 # (y, z) ASSUMED from the first snapshot: box y -108.6 .. -67.8, clear of the axle (y -116.8, underside z 45)
    "HORN_L": 12.0,                               # ASSUMED metal 25T horn hole radius; sized so servo +-30 deg reaches the clearance lock (CALCULATED below)
    "HORN_BALL": (68.5, -93.70, 26.38),           # horn pointing down, 8.4 deg rearward: drag link square to the horn at centre in its yz plane
                                                  # (CALCULATED); ball 7 mm outboard of the horn plate, link 10.8 deg off y in plan (evens out the locks)
    "DRAG_LINK_L": 16.30,                         # ball to ball (CALCULATED)
    "UPPER_LINK_OUT": (101.0, -219.3, 91.0),  # MOVED inboard from x 105 (hinge along x, so the link stays parallel to the arm)
    "LOCK_DEG": 18.5,                             # mechanical lock each way the envelope is sized for (arm-limited inner lock, CALCULATED)
    # carrier / knuckle envelope (carrier frame = world at rest). Boxes (x0, y0, z0, x1, y1, z1); cyl_z = vertical cylinder about the kingpin
    # (r, z0, z1); cyl_x = cylinder along x at the axle (r, x0, x1). The thread builds the shapes inside these; checked in envelope_report().
    "CARRIER_ENVELOPE": {
        "kingpin block": ("cyl_z", 7.0, 27.0, 54.0),                         # carrier's pin block between the knuckle ears (D4 steel kingpin ASSUMED)
        "web": ("box", 104.0, -223.55, 27.0, 116.0, -209.55, 54.0),          # leaves the wheel through the tyre bore (r 26.63, x 113..127)
        "tip lug": ("box", 93.5, -224.0, 33.0, 104.0, -209.0, 48.0),         # on the arm tip pin (y -216.55, z 40.6), outboard of the tip boss
        "tower": ("box", 94.8, -225.5, 48.0, 107.2, -213.1, 96.0),           # up to the upper link clevis (hinge x 101, z 91, plates +-6.2)
    },
    "KNUCKLE_ENVELOPE": {                                                    # steers with the wheel: an outer C round the carrier block.
        "lower ear": ("box", 117.0, -222.55, 22.0, 135.0, -210.55, 26.0),    # 4 mm ears, 4 x 12 mm spine: too thin for PETG -> STEEL knuckle
        "upper ear": ("box", 117.0, -222.55, 55.0, 135.0, -210.55, 59.0),
        "outer spine": ("box", 131.0, -222.55, 22.0, 135.0, -210.55, 59.0),  # inside the rim bore (r 21.13)
        "axle boss": ("cyl_x", 8.0, 131.0, 148.0),                           # stub axle out to the measured hub face x 148 (hub boss r 10)
        "steering arm": ("arm", 4.0, 21.0, 25.0),                            # plate half width 4, z 21..25 (clears the rim lip r 21.13); ball 6.4 mm below
    },
    # rocker mount: clamp on the fixed 5 mm pivot axle between the arm's pivot bosses, hanger down to the rocker bush
    "ROCKER_MOUNT": {                             # pin hangs from above (cantilever D4 steel pin ASSUMED); nothing under the rocker
        "axle clamp + hanger": ("box", 71.2, -120.8, 34.0, 84.7, -112.8, 53.5),   # split clamp round the fixed axle (y -116.8, z 47.5) between
                                                                                  # the arm's pivot bosses (x 62.45..69.95 / 85.75..93.25)
        "foot over the rocker": ("box", 84.7, -120.8, 34.0, 90.5, -112.8, 37.5),  # carries the pin at x 85.5; 1 mm under the outer pivot boss
    },
    # keep-out the v2 arm must respect: rocker, rod ends and drag link live under the arm root
    "ARM_ROOT_L0_Z_MIN": 45.0,                    # lower root node of both arm plates (y -130) at world z >= 45 (Nov 21 arm: 34.1, collides at droop)
    "LINKAGE_ZONE": (58.0, -133.0, 12.0, 92.0, -96.0, 33.0),  # box swept by rocker, rod ends, drag link (body frame): no arm member inside at any travel
    "SERVO_TRAVEL_DEG": 30.0,                     # +-30 deg; Ackermann by mixing the two servos (one per wheel)
}

# ---- inputs (ASSUMED unless noted) ----
SERVO_TORQUE_LOW = 1.0      # N.m ASSUMED B44BLS-class rating at 6.0 V (about 10 kg.cm, not checked on a data sheet); used for the torque margin
SERVO_STALL_HIGH = 2.0      # N.m ASSUMED stall at 7.4 V (about 20 kg.cm, not checked); used for the rod forces and buckling (conservative)
MASS_KG = 2.576             # CALCULATED by @cg-tip (body + battery), taken as given
FRONT_SHARE = 0.30          # ASSUMED share of the weight on ONE front wheel
MU = 1.0                    # ASSUMED tyre friction, dry steering at standstill
PATCH_R = 10.0              # mm ASSUMED equivalent radius of the contact patch twisting on the spot
DYN = 1.5                   # ASSUMED dynamic factor (kerb, bump while steering)
TORQUE_MARGIN = 1.30        # owner's rule
E_STEEL = 200e3             # N/mm2
M3_CORE = 2.387             # mm minor diameter M3 (ISO)
TYRE_GAP = 2.0              # mm ASSUMED required running gap
RIM_R = 26.63               # mm MEASURED tyre bore (x 113..175); sidewall from here out
RIM_BORE, RIM_LIP_X = 21.13, (127.0, 129.0)   # mm MEASURED rim barrel bore (x 127..161) and inner lip (r 21.13..26.63)
HUB_BOSS_R, SPOKE_X = 10.0, 152.0             # mm MEASURED hub boss r 10 from x 148; spokes' inner face x 152
TUBE_R_ARM, TUBE_R_LINK, ROD_R = 4.2, 5.0, 4.0   # mm ASSUMED envelope radii (7 mm square chord corner, 8 mm link tube, rod-end housing)
WHEELBASE = 453.0           # mm ASSUMED unchanged (Nov 20 notes)
HUB_FACE_X = 148.0          # mm MEASURED inner hub face
LINK_R = 5.6                # mm ASSUMED upper link envelope (8 x 8 tube corner; 6 x 9 tongue, 12 mm along y at each end)

TRAVEL = (-32.9, -24.0, -16.0, -8.0, 0.0, 8.0, 16.0, 24.0, 32.0, 37.9)
XA, ZA = np.array((1.0, 0, 0)), np.array((0, 0, 1.0))
D = STEER_DESIGN


def rot(v, axis, deg):
    """Rodrigues rotation of v about the unit axis by deg (right hand)."""
    a = math.radians(deg); k = np.asarray(axis, float); k = k / np.linalg.norm(k); v = np.asarray(v, float)
    return v * math.cos(a) + np.cross(k, v) * math.sin(a) + k * np.dot(k, v) * (1 - math.cos(a))


def carrier(t):
    """Displacement of the arm tip (wheel centre) at wheel travel t (CALCULATED, snapshot wheel_shift)."""
    return np.array((0.0, sa2.wheel_shift(t), t))


_TIP0 = np.array((0.0, *sa2.TIP)); _UI, _UO = np.array(sa2.UPLINK_IN, float), np.array(sa2.UPLINK_OUT, float)
_LU, _LC = np.linalg.norm((_UO - _UI)[1:]), np.linalg.norm((_UO - _TIP0)[1:])
_pc = {}


def carrier_pitch(t):
    """Carrier rotation about x (deg, right hand) at travel t, from the real four-bar: arm 99.99 mm, upper link 100.14 mm (CALCULATED).
    The snapshot's parallelogram is not exact, so the carrier (and the kingpin) pitch slightly."""
    if t not in _pc:
        tip = (_TIP0 + carrier(t))[1:]; ui = _UI[1:]
        dvec = tip - ui; dd = np.linalg.norm(dvec)
        a = (_LU ** 2 - _LC ** 2 + dd ** 2) / (2 * dd); hgt = math.sqrt(max(_LU ** 2 - a ** 2, 0.0))
        base = ui + a * dvec / dd; perp = np.array((-dvec[1], dvec[0])) / dd
        cands = [base + hgt * perp, base - hgt * perp]
        uo = min(cands, key=lambda c: np.linalg.norm(c - ((_UO - _TIP0)[1:] + tip)))
        v0, v1 = (_UO - _TIP0)[1:], uo - tip
        _pc[t] = math.degrees(math.atan2(v0[0] * v1[1] - v0[1] * v1[0], v0 @ v1))
    return _pc[t]


PITCH_ON = True     # False = ideal translation (exact parallelogram)


def cpose(p, t):
    """A point fixed in the carrier (given at rest) at travel t: rotated by the carrier pitch about the tip, moved with the tip."""
    r = np.asarray(p, float) - _TIP0
    if PITCH_ON: r = rot(r, XA, carrier_pitch(t))
    return _TIP0 + carrier(t) + r


def cdir(v, t):
    return rot(v, XA, carrier_pitch(t)) if PITCH_ON else np.asarray(v, float)


def arm_point(p, t):
    """A point fixed in the swing arm, at wheel travel t (tip lies at -y, so tip up is a rotation about -x)."""
    a = -sa2.arm_swing(t); y, z = p[1] - sa2.PIVOT[0], p[2] - sa2.PIVOT[1]
    return np.array((p[0], sa2.PIVOT[0] + math.cos(a) * y - math.sin(a) * z, sa2.PIVOT[1] + math.sin(a) * y + math.cos(a) * z))


def solve(f, guess, step=0.5, span=70.0):
    """Root of f nearest to guess: scan outward from the guess, then bisect."""
    f0 = f(guess)
    if f0 == 0: return guess
    for i in range(1, int(span / step) + 1):
        for x in (guess + i * step, guess - i * step):
            x0 = x - step if x > guess else x + step
            if f(x0) * f(x) <= 0:
                lo, hi = x0, x
                for _ in range(50):
                    m = (lo + hi) / 2
                    if f(lo) * f(m) <= 0: hi = m
                    else: lo = m
                return (lo + hi) / 2
    return float("nan")


class Layout:
    """Kingpin + knuckle ball + a function eta -> tie rod inner ball. eta = servo angle, + pushes the +x rod forward (thread's sign)."""
    def __init__(self, name, kp, axis, kball, inner):
        self.name, self.kp, self.ax, self.k0, self.inner = name, np.array(kp, float), np.array(axis, float), np.array(kball, float), inner
        self.L0 = float(np.linalg.norm(inner(0.0) - self.k0))
        self._c = {}

    def kball(self, d, t):
        return cpose(self.kp + rot(self.k0 - self.kp, self.ax, d), t)

    def track(self, eta, t):
        """Steer angle delta (deg, right hand about +z: + turns the front toward +x) on the assembled branch, by continuation."""
        key = (round(eta, 6), round(t, 6))
        if key not in self._c:
            d = 0.0
            for e in np.linspace(0, eta, 7)[1:]:
                h = self.inner(e); d = solve(lambda q: np.linalg.norm(h - self.kball(q, 0.0)) - self.L0, d)
            h = self.inner(eta)
            for tt in np.linspace(0, t, 5)[1:]:
                d = solve(lambda q: np.linalg.norm(h - self.kball(q, tt)) - self.L0, d)
            self._c[key] = d
        return self._c[key]

    def knuckle_trans(self, d, eta, t):
        """Transmission angle (deg) tie rod / knuckle arm about the kingpin (90 ideal), and the rod's lever about the kingpin (mm)."""
        return crank_trans(cpose(self.kp, t), cdir(self.ax, t), self.kball(d, t), self.inner(eta))


def crank_trans(axis_pt, axis, crank_end, other_end):
    """Transmission angle (deg, 90 ideal, 0 = dead centre) of a link on a crank about an axis, and the link's lever about the axis (mm)."""
    r = crank_end - axis_pt; u = (other_end - crank_end) / np.linalg.norm(other_end - crank_end)
    lever = abs(np.dot(np.cross(r, u), axis)); rp = np.linalg.norm(r - axis * np.dot(r, axis))
    return math.degrees(math.asin(min(1.0, lever / rp))), lever


# ---- layout A: the thread's direct horn ----
S_A = np.array((sa2.HORN_X, *sa2.SERVO_SHAFT)); H_A0 = np.array(sa2.HORN_BALL, float)
LAY_A = Layout("A  THREAD 7b79b4c: direct horn 11.8 mm straight down, kingpin x %.0f" % sa2.KINGPIN_X,
               (sa2.KINGPIN_X, sa2.TIP[0], sa2.KNUCKLE_BALL[2]), (0, 0, 1), sa2.KNUCKLE_BALL, lambda e: S_A + rot(H_A0 - S_A, XA, -e))

# ---- layout B: rocker + drag link ----
S_B = np.array((D["HORN_BALL"][0], *D["SERVO_SHAFT"])); HB0 = np.array(D["HORN_BALL"], float)
R_P, R_AX = np.array(D["ROCKER_PIN"], float), np.array(D["ROCKER_AXIS"], float)
RIN0, ROUT0 = np.array(D["ROCKER_IN_BALL"], float), np.array(D["TIE_ROD_INNER_BALL"], float)
LD0 = float(np.linalg.norm(RIN0 - HB0))
_rk = {}


def horn_B(eta):
    return S_B + rot(HB0 - S_B, XA, -eta)


def rocker(eta):
    """Rocker angle (deg about ROCKER_AXIS) that keeps the drag link length, by continuation from 0."""
    key = round(eta, 6)
    if key not in _rk:
        r = 0.0
        for e in np.linspace(0, eta, 7)[1:]:
            hb = horn_B(e); r = solve(lambda q: np.linalg.norm(R_P + rot(RIN0 - R_P, R_AX, q) - hb) - LD0, r)
        _rk[key] = r
    return _rk[key]


LAY_B = Layout("B  CHOSEN: rocker (vertical pin) + drag link, kingpin x %.0f" % D["KINGPIN_POINT"][0], D["KINGPIN_POINT"], D["KINGPIN_AXIS"],
               D["KNUCKLE_BALL"], lambda e: R_P + rot(ROUT0 - R_P, R_AX, rocker(e)))


def derived():
    """Re-derive the CALCULATED entries of STEER_DESIGN from the primary values."""
    kp, kb = np.array(D["KINGPIN_POINT"]), np.array(D["KNUCKLE_BALL"])
    zb = np.array((D["TIE_ROD_INNER_BALL"][0], sa2.PIVOT[0] + kb[1] - sa2.TIP[0], sa2.PIVOT[1] + kb[2] - sa2.TIP[1]))
    rp = kp + (zb - kb)
    # horn rest angle: drag link square to the horn (in the servo's yz plane) at centre
    sy, sz = D["SERVO_SHAFT"]; v = np.array((RIN0[1] - sy, RIN0[2] - sz)); h = D["HORN_L"]
    phi = math.atan2(-v[0], -v[1])                       # angle of (S -> Rin) from straight down toward the front
    a = phi - math.acos(h / np.linalg.norm(v))           # horn angle from straight down toward the front (deg below)
    hb = np.array((D["HORN_BALL"][0], sy - h * math.sin(a), sz - h * math.cos(a)))
    return dict(TIE_ROD_INNER_BALL=zb, ROCKER_PIN=rp, HORN_BALL=hb, HORN_FWD_DEG=math.degrees(a),
                TIE_ROD_L=float(np.linalg.norm(zb - kb)), DRAG_LINK_L=float(np.linalg.norm(RIN0 - hb)))


# ---- swept tyre clearance --------------------------------------------------------------------------------------------------------
def _arm_segments():
    """Outer truss plane of the Nov 21 arm (parts/frontarm.py; world = ref + (0, 9.0, -15.3)). ASSUMED stand-in for the v2 arm shape."""
    ref = dict(U0=(-139.0, 75.5), U1=(-168.0, 80.5), U2=(-194.0, 67.6), U3=(-214.0, 67.6),
               L0=(-139.0, 49.4), L1=(-168.0, 47.0), L2=(-194.0, 45.5), L3=(-214.0, 45.8), T=(-225.55, 55.9))
    w = {k: (v[0] + 9.0, v[1] - 15.3) for k, v in ref.items()}
    x = sa2.ARM_PLATES_X[1]
    segs = [(np.array((x, *w[p[:2]])), np.array((x, *w[p[2:]])), TUBE_R_ARM)
            for p in ("U0U1", "U1U2", "U2U3", "L0L1", "L1L2", "L2L3", "L0U0", "L0U1", "L1U1", "L2U1", "L2U2", "L3U2", "L3U3", "U3T", "L3T")]
    segs.append((np.array((x - 3.75, *w["T"])), np.array((x + 3.75, *w["T"])), 7.5))      # tip boss OD 15 x 7.5
    return segs


ARM_SEGS = _arm_segments()


def _tyre_points():
    pts = []
    xi = sa2.HALF_TRACK - sa2.TYRE_W / 2
    for a in np.radians(np.arange(0, 360, 4)):
        for x in np.linspace(xi, xi + sa2.TYRE_W, 7):
            pts.append((x, sa2.TIP[0] + math.sin(a) * sa2.TYRE_R, sa2.TIP[1] + math.cos(a) * sa2.TYRE_R))
        for r in np.linspace(RIM_R, sa2.TYRE_R, 8):
            pts.append((xi, sa2.TIP[0] + math.sin(a) * r, sa2.TIP[1] + math.cos(a) * r))
    return np.array(pts)


TYRE0 = _tyre_points()


def _seg_dist(P, a, b):
    ab = b - a; s = np.clip(((P - a) @ ab) / (ab @ ab), 0, 1)
    return np.linalg.norm(P - (a + s[:, None] * ab), axis=1)


def link_polyline(t, x_out=None):
    """Upper link centre line at travel t: outer hinge -> 12 mm tongue along y -> tube -> 12 mm tongue -> inner hinge (as parts/uplink.py)."""
    xo = D["UPPER_LINK_OUT"][0] if x_out is None else x_out
    ui = np.array(sa2.UPLINK_IN, float); uo = cpose((xo, *D["UPPER_LINK_OUT"][1:]), t)
    d = (uo - ui)[1:]; d = d / np.linalg.norm(d)
    ji = ui + np.array((0, *(d * 12.0))); jo = uo - np.array((0, *(d * 12.0)))
    return [(uo, jo), (jo, ji), (ji, ui)]


def tyre_gaps(lay, d, t, eta=None):
    """Smallest gaps (mm) of the steered tyre at travel t to the arm's outer truss and the upper link (and the tie rod)."""
    P = _steered(lay, d, t)
    g = {"arm": min((_seg_dist(P, arm_point(a, t), arm_point(b, t)) - r).min() for a, b, r in ARM_SEGS),
         "upper link": min((_seg_dist(P, a, b) - LINK_R).min() for a, b in link_polyline(t))}
    if eta is not None:
        g["tie rod"] = (_seg_dist(P, lay.inner(eta), lay.kball(d, t)) - ROD_R).min()
    return g


def _steered(lay, d, t, pts=None):
    """Tyre (or wheel) points steered by d about the vertical kingpin, then moved with the carrier to travel t."""
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    q = (TYRE0 if pts is None else pts) - lay.kp
    P = np.column_stack((c * q[:, 0] - s * q[:, 1], s * q[:, 0] + c * q[:, 1], q[:, 2])) + lay.kp        # steered about the vertical kingpin (rest)
    a = math.radians(carrier_pitch(t)) if PITCH_ON else 0.0
    q = P - _TIP0
    return np.column_stack((q[:, 0], math.cos(a) * q[:, 1] - math.sin(a) * q[:, 2], math.sin(a) * q[:, 1] + math.cos(a) * q[:, 2])) + _TIP0 + carrier(t)


def lock_limit(lay, sign, key):
    last = 0.0
    for d in np.arange(0.25, 45.01, 0.25):
        if any(tyre_gaps(lay, sign * d, t)[key] < TYRE_GAP for t in TRAVEL):
            return last
        last = d
    return last


def ackermann_outer(di):
    return math.degrees(math.atan(1.0 / (1.0 / math.tan(math.radians(di)) + 2 * D["KINGPIN_POINT"][0] / WHEELBASE)))


def euler(L):
    return math.pi ** 2 * E_STEEL * (math.pi * M3_CORE ** 4 / 64) / L ** 2



# ---- carrier / knuckle envelope ------------------------------------------------------------------------------------------------
def _sdf(shape, P):
    """Signed distance (mm, < 0 inside) of points P to one envelope shape, carrier frame."""
    kp = np.array(D["KINGPIN_POINT"]); kind = shape[0]
    if kind == "box":
        lo, hi = np.array(shape[1:4]), np.array(shape[4:7]); c, h = (lo + hi) / 2, (hi - lo) / 2
        q = np.abs(P - c) - h
        return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)
    if kind == "cyl_z":
        r, z0, z1 = shape[1:]
        dr = np.hypot(P[:, 0] - kp[0], P[:, 1] - kp[1]) - r; dz = np.maximum(z0 - P[:, 2], P[:, 2] - z1)
        return np.hypot(np.maximum(dr, 0), np.maximum(dz, 0)) + np.minimum(np.maximum(dr, dz), 0)
    if kind == "cyl_x":
        r, x0, x1 = shape[1:]
        dr = np.hypot(P[:, 1] - sa2.TIP[0], P[:, 2] - sa2.TIP[1]) - r; dx = np.maximum(x0 - P[:, 0], P[:, 0] - x1)
        return np.hypot(np.maximum(dr, 0), np.maximum(dx, 0)) + np.minimum(np.maximum(dr, dx), 0)
    if kind == "arm":                       # flat plate from the kingpin to the knuckle ball (plan), half width w, z0..z1
        w, z0, z1 = shape[1:]
        a, b = kp[:2], np.array(D["KNUCKLE_BALL"][:2]); ab = b - a
        tt = np.clip(((P[:, :2] - a) @ ab) / (ab @ ab), 0, 1)
        dxy = np.linalg.norm(P[:, :2] - (a + tt[:, None] * ab), axis=1) - w; dz = np.maximum(z0 - P[:, 2], P[:, 2] - z1)
        return np.hypot(np.maximum(dxy, 0), np.maximum(dz, 0)) + np.minimum(np.maximum(dxy, dz), 0)
    raise ValueError(kind)


def _wheel_points():
    """MEASURED wheel seen from inside: tyre inner face (x 113, r >= 26.63), tyre bore (r 26.63, x 113..127), rim lip face (x 127, r 21.13..26.63),
    rim barrel bore (r 21.13, x 127..152), hub boss face (x 148, r <= 10) and side (r 10, x 148..152), spokes' inner face (x 152, r 10..21.13)."""
    xi = sa2.HALF_TRACK - sa2.TYRE_W / 2; pts = []
    for a in np.radians(np.arange(0, 360, 2)):
        ca, sn = math.cos(a), math.sin(a)
        rings = [(xi, r) for r in np.arange(RIM_R, sa2.TYRE_R + 0.01, 1.5)]
        rings += [(x, RIM_R) for x in np.arange(xi, RIM_LIP_X[0] + 0.01, 1.0)]
        rings += [(RIM_LIP_X[0], r) for r in np.arange(RIM_BORE, RIM_R + 0.01, 1.0)]
        rings += [(x, RIM_BORE) for x in np.arange(RIM_LIP_X[0], SPOKE_X + 0.01, 1.0)]
        rings += [(HUB_FACE_X, r) for r in np.arange(0.0, HUB_BOSS_R + 0.01, 1.0)]
        rings += [(x, HUB_BOSS_R) for x in np.arange(HUB_FACE_X, SPOKE_X + 0.01, 1.0)]
        rings += [(SPOKE_X, r) for r in np.arange(HUB_BOSS_R, RIM_BORE + 0.01, 1.0)]
        pts += [(x, r * sn, r * ca) for x, r in rings]
    return np.array(pts) + np.array((0, sa2.TIP[0], sa2.TIP[1]))


def _surface_points(shape, step=1.0):
    """Points on the surface of an envelope shape (for the knuckle vs carrier sweep)."""
    kp = np.array(D["KINGPIN_POINT"]); g = []
    if shape[0] == "box":
        lo, hi = np.array(shape[1:4]), np.array(shape[4:7])
        ax = [np.arange(lo[i], hi[i] + 1e-6, step) for i in range(3)]
        X, Y, Z = np.meshgrid(*ax, indexing="ij"); P = np.column_stack((X.ravel(), Y.ravel(), Z.ravel()))
        on = (np.isclose(P, lo) | np.isclose(P, hi)).any(axis=1); return P[on]
    if shape[0] in ("cyl_z", "arm"):
        P = np.array([(x, y, z) for x in np.arange(kp[0] - 30, kp[0] + 20, step) for y in np.arange(kp[1] - 20, kp[1] + 20, step)
                      for z in np.arange(shape[2], shape[3] + 1e-6, step)])
        dd = _sdf(shape, P); return P[(dd <= 0) & (dd > -step)]
    if shape[0] == "cyl_x":
        r, x0, x1 = shape[1:]
        return np.array([(x, sa2.TIP[0] + r * math.sin(a), sa2.TIP[1] + r * math.cos(a)) for x in np.arange(x0, x1 + 1e-6, step) for a in np.radians(np.arange(0, 360, 6))])


def _rot_kp(P, d):
    kp = np.array(D["KINGPIN_POINT"]); c, s = math.cos(math.radians(d)), math.sin(math.radians(d)); q = P - kp
    return np.column_stack((c * q[:, 0] - s * q[:, 1], s * q[:, 0] + c * q[:, 1], q[:, 2])) + kp


def envelope_report():
    L = D["LOCK_DEG"]; W = _wheel_points()
    lo = min(-L, LAY_B.track(-30.0, 0.0) - 0.5, LAY_B.track(30.0, 0.0) - 0.5); hi = max(L, LAY_B.track(30.0, 0.0) + 0.5, LAY_B.track(-30.0, 0.0) + 0.5)
    steer = np.arange(lo, hi + 1e-6, 0.5)
    print("\nCARRIER / KNUCKLE ENVELOPE (CALCULATED against the MEASURED wheel: tyre bore r %.2f, rim bore r %.2f, hub face x %.0f;\n   swept over %.1f .. %+.1f deg; travel does not change it: carrier and wheel move together)" % (RIM_R, RIM_BORE, HUB_FACE_X, lo, hi))
    print("  carrier (fixed) to the steered wheel, min gap mm:")
    worst = 99.0
    for n, sh in D["CARRIER_ENVELOPE"].items():
        g = min(_sdf(sh, _rot_kp(W, d)).min() for d in steer); worst = min(worst, g)
        print("    %-22s %6.2f" % (n, g))
    print("  knuckle (steers with the wheel) to the wheel, static, min gap mm (axle boss seats on the hub face: gap to the rest of the wheel):")
    for n, sh in D["KNUCKLE_ENVELOPE"].items():
        Wn = W[~((np.abs(W[:, 0] - HUB_FACE_X) < 0.01) & (np.hypot(W[:, 1] - sa2.TIP[0], W[:, 2] - sa2.TIP[1]) <= HUB_BOSS_R + 0.01))] if n == "axle boss" else W
        g = _sdf(sh, Wn).min(); worst = min(worst, g)
        print("    %-22s %6.2f" % (n, g))
    print("  knuckle swept to the carrier, min gap mm:")
    for n, sh in D["KNUCKLE_ENVELOPE"].items():
        P = _surface_points(sh)
        g = min(min(_sdf(c, _rot_kp(P, d)).min() for c in D["CARRIER_ENVELOPE"].values()) for d in steer); worst = min(worst, g)
        print("    %-22s %6.2f" % (n, g))
    print("  -> worst %.2f mm (target >= 1.0): %s" % (worst, "OK" if worst >= 0.995 else "NOT OK"))
    return worst


# ---- rod-end misalignment, linkage vs arm root --------------------------------------------------------------------------------
def rod_end_cones(lay=None):
    """Tilt (deg) of each link out of the plane square to its ball stud, over servo +-30 and the whole travel: the cone the ball cup needs."""
    lay = lay or LAY_B; res = {}
    for e in np.linspace(-30, 30, 13):
        rin = R_P + rot(RIN0 - R_P, R_AX, rocker(e)); hb = horn_B(e)
        ud = (rin - hb) / np.linalg.norm(rin - hb)
        for t in TRAVEL:
            k, h = lay.kball(lay.track(e, t), t), lay.inner(e); ut = (h - k) / np.linalg.norm(h - k)
            for name, u, stud in (("tie rod @ knuckle (stud vertical)", ut, cdir(ZA, t)), ("tie rod @ rocker (stud vertical)", ut, R_AX),
                                  ("drag link @ rocker (stud vertical)", ud, R_AX), ("drag link @ horn (stud along x)", ud, XA)):
                a = math.degrees(math.asin(min(1.0, abs(np.dot(u, stud / np.linalg.norm(stud))))))
                lo, hi = res.get(name, (99.0, -1.0)); res[name] = (min(lo, a), max(hi, a))
    return res


ARM_ROOT_L0_Z = None     # None = Nov 21 root node L0 (world z 34.1); a number = proposed raised root node (world z)


def _arm_root_segments():
    ref = dict(U0=(-139.0, 75.5), U1=(-168.0, 80.5), L0=(-139.0, 49.4), L1=(-168.0, 47.0))
    w = {k: np.array((v[0] + 9.0, v[1] - 15.3)) for k, v in ref.items()}; P = np.array(sa2.PIVOT)
    if ARM_ROOT_L0_Z is not None: w["L0"] = np.array((w["L0"][0], ARM_ROOT_L0_Z))
    segs = []
    for x in sa2.ARM_PLATES_X:
        for a, b in (("L0", "L1"), ("U0", "U1"), ("L0", "U0"), ("L0", "U1")):
            segs.append((np.array((x, *w[a])), np.array((x, *w[b])), TUBE_R_ARM))
        segs.append((np.array((x, *P)), np.array((x, *w["L0"])), 3.75))        # pivot neck web (hull, 7.5 wide), lower edge
        segs.append((np.array((x, *P)), np.array((x, *w["U0"])), 3.75))        # pivot neck web, upper edge
        if x == sa2.ARM_PLATES_X[0]:
            segs.append((np.array((62.2, *P)), np.array((70.2, *P)), 11.0))     # inner pivot boss, real: r 11 over x 62.2..70.2 (thread)
        else:
            segs.append((np.array((x - 3.75, *P)), np.array((x + 3.75, *P)), 9.0))  # outer pivot boss OD 18
    for k in ("L0", "U0"):
        segs.append((np.array((sa2.ARM_PLATES_X[0], *w[k])), np.array((sa2.ARM_PLATES_X[1], *w[k])), 4.5))
    return segs


def mount_vs_arm():
    """Min gap (mm) of the rocker mount boxes to the arm members over all travel (pivot bosses: coaxial, the axial gaps are printed separately)."""
    out = {}
    segs = [sg for sg in _arm_root_segments() if not (abs(sg[0][1] - sa2.PIVOT[0]) < 1e-6 and abs(sg[1][1] - sa2.PIVOT[0]) < 1e-6
                                                     and abs(sg[0][2] - sa2.PIVOT[1]) < 1e-6 and abs(sg[1][2] - sa2.PIVOT[1]) < 1e-6)]
    for n, sh in D["ROCKER_MOUNT"].items():
        P = _surface_points(sh, 0.75)
        for t in TRAVEL:
            for a, b, r in segs:
                g = (_seg_dist(P, arm_point(a, t), arm_point(b, t)) - r).min()
                if g < out.get(n, (99.0,))[0]: out[n] = (g, t)
        # rocker (hub r 5, z span) to the mount: vertical gap
    box = D["ROCKER_MOUNT"]["axle clamp + hanger"]
    ax_gap = (box[1] - 70.2, (sa2.ARM_PLATES_X[1] - 3.75) - box[4])
    boss_gap = (sa2.PIVOT[1] - 9.0) - D["ROCKER_MOUNT"]["foot over the rocker"][6]
    hub_gap = D["ROCKER_MOUNT"]["foot over the rocker"][3] - D["ROCKER_PIN_SPAN_Z"][1]
    return out, ax_gap, boss_gap, hub_gap


def linkage_vs_arm():
    """Min gap (mm) of rocker, rod ends, tie rod, drag link and horn to the arm root over servo +-30 and the whole travel (ASSUMED sizes)."""
    out = {}
    for e in (-30.0, 0.0, 30.0):
        rin = R_P + rot(RIN0 - R_P, R_AX, rocker(e)); rout = LAY_B.inner(e); hb = horn_B(e)
        z0, z1 = D["ROCKER_PIN_SPAN_Z"]; (p0, p1), (q0, q1) = D["ROCKER_PLATES_Z"]
        up = lambda v, z: np.array((v[0], v[1], z))
        items = {"rocker hub r5": [(up(R_P, z0 + 5.0), up(R_P, z1 - 5.0), 5.0)],     # capsule inside the flat-ended hub
                 "rocker arms r3": [(up(R_P, (p0 + p1) / 2), up(rout, (p0 + p1) / 2), 3.0), (up(R_P, (q0 + q1) / 2), up(rin, (q0 + q1) / 2), 3.0)],
                 "rod-end housings r5": [(rout, rout, 5.0), (rin, rin, 5.0), (hb, hb, 5.0)],
                 "drag link r2": [(hb, rin, 2.0)], "horn r4": [(S_B, hb, 4.0)]}
        for t in TRAVEL:
            k = LAY_B.kball(LAY_B.track(e, t), t)
            items["tie rod r2"] = [(rout + (k - rout) * 0.08, k, 2.0)]
            for n, lst in items.items():
                for a, b, r in lst:
                    for sa, sb, sr in _arm_root_segments():
                        sa_t, sb_t = arm_point(sa, t), arm_point(sb, t)
                        g = _seg_seg(np.array(a, float), np.array(b, float), sa_t, sb_t) - r - sr
                        if g < out.get(n, (99.0,))[0]: out[n] = (g, e, t)
    return out


def _seg_seg(p1, q1, p2, q2):
    """Distance between two segments (sampled; fine for 0.1 mm accuracy at these sizes)."""
    A = p1 + np.linspace(0, 1, 41)[:, None] * (q1 - p1)
    return _seg_dist(A, p2, q2).min() if np.linalg.norm(q2 - p2) > 1e-9 else np.linalg.norm(A - p2, axis=1).min()


# ---- rocker packaging (fix round 2) -------------------------------------------------------------------------------------------
def _pts_cyl_v(c, r, z0, z1, n=48):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False); pts = []
    for z in np.linspace(z0, z1, max(3, int((z1 - z0) / 0.5) + 1)):
        pts += [(c[0] + r * math.cos(q), c[1] + r * math.sin(q), z) for q in a]
    for rr in np.linspace(0, r, max(2, int(r / 0.5) + 1)):
        for z in (z0, z1): pts += [(c[0] + rr * math.cos(q), c[1] + rr * math.sin(q), z) for q in a]
    return np.array(pts)


def _pts_capsule(a, b, r, n=24):
    a, b = np.asarray(a, float), np.asarray(b, float); u = b - a; L = np.linalg.norm(u); u = u / L
    e1 = np.cross(u, (0, 0, 1.0)); e1 = e1 / np.linalg.norm(e1) if np.linalg.norm(e1) > 1e-6 else np.array((1.0, 0, 0)); e2 = np.cross(u, e1)
    q = np.linspace(0, 2 * np.pi, n, endpoint=False); pts = []
    for s_ in np.linspace(0, L, max(2, int(L / 0.5) + 1)):
        pts += list(a + u * s_ + r * (np.cos(q)[:, None] * e1 + np.sin(q)[:, None] * e2))
    for c in (a, b):                                                   # end spheres
        for th in np.linspace(0, np.pi, 9):
            pts += list(c + r * (math.cos(th) * u[None, :] + math.sin(th) * (np.cos(q)[:, None] * e1 + np.sin(q)[:, None] * e2)))
            pts += list(c + r * (-math.cos(th) * u[None, :] + math.sin(th) * (np.cos(q)[:, None] * e1 + np.sin(q)[:, None] * e2)))
    return np.array(pts)


def _pts_plate(a, b, w, z0, z1):
    """Stadium plate between plan points a and b (round ends r w), z0..z1."""
    a, b = np.asarray(a[:2], float), np.asarray(b[:2], float); u = b - a; L = np.linalg.norm(u); u = u / L; nrm = np.array((-u[1], u[0]))
    outline = [a + u * s_ + nrm * k * w for s_ in np.linspace(0, L, int(L / 0.5) + 1) for k in (-1, 1)]
    for c, sg in ((a, -1), (b, 1)):
        outline += [c + w * (sg * u * math.sin(q) + nrm * math.cos(q)) for q in np.linspace(0, np.pi, 13)]
    fill = [a + u * s_ + nrm * k for s_ in np.linspace(0, L, int(L / 0.7) + 1) for k in np.linspace(-w, w, 7)]
    pts = [(p[0], p[1], z) for p in outline for z in np.linspace(z0, z1, 5)] + [(p[0], p[1], z) for p in fill for z in (z0, z1)]
    return np.array(pts)


def _pts_box(lo, hi, step=0.6):
    return _surface_points(("box", *lo, *hi), step)


def rocker_bodies(eta, t):
    """Point clouds of the rocker (hub, two plates), both rod-end housings and shanks, drag link, horn at servo angle eta and travel t."""
    r = rocker(eta); rout = LAY_B.inner(eta); rin = R_P + rot(RIN0 - R_P, R_AX, r); hb = horn_B(eta)
    k = LAY_B.kball(LAY_B.track(eta, t), t)
    (p0, p1), (q0, q1) = D["ROCKER_PLATES_Z"]; w = D["ROCKER_PLATE_HALF_W"]; z0, z1 = D["ROCKER_PIN_SPAN_Z"]
    rt, ht, st, lt = D["ROD_END_HOUSINGS"]["tie"]; rd, hd, sd, ld = D["ROD_END_HOUSINGS"]["drag"]
    ut = (k - rout) / np.linalg.norm(k - rout); ud = (hb - rin) / np.linalg.norm(hb - rin)
    return {
        "rocker hub": _pts_cyl_v(R_P, 5.0, z0, z1),
        "tie plate": _pts_plate(R_P, rout, w, p0, p1),
        "drag plate": _pts_plate(R_P, rin, w, q0, q1),
        "tie housing": _pts_cyl_v(rout, rt, rout[2] - ht / 2, rout[2] + ht / 2),
        "drag housing": _pts_cyl_v(rin, rd, rin[2] - hd / 2, rin[2] + hd / 2),
        "tie shank": _pts_capsule(rout + ut * rt, rout + ut * (rt + lt), st),
        "drag shank": _pts_capsule(rin + ud * rd, rin + ud * (rd + ld), sd),
        "drag link": _pts_capsule(rin + ud * (rd + ld), hb - ud * (rd + 1.0), 2.0),
    }


ROCKER_PAIRS = [("tie housing", "drag housing"), ("tie housing", "drag plate"), ("drag housing", "tie plate"), ("tie housing", "drag shank"),
                ("drag housing", "tie shank"), ("tie shank", "drag shank"), ("tie shank", "drag plate"), ("drag link", "tie housing"),
                ("drag link", "tie plate"), ("drag shank", "tie plate"), ("tie housing", "rocker hub"), ("drag housing", "rocker hub")]


def _deck_points():
    pts = []
    for y in np.arange(-140.0, -85.0, 0.7):
        zt = 18.3 + 0.07 * (y + 205.0)
        pts += [(x, y, zt) for x in np.arange(30.0, 54.51, 0.7)] + [(54.5, y, z) for z in np.arange(zt - 6.0, zt + 0.01, 0.7)]
    return np.array(pts)


def ground_clearance(P, t_side_phi):
    """Height above the ground (mm) of body-frame points P with the body rolled by phi (low side +x)."""
    phi = t_side_phi; c, sn = math.cos(math.radians(phi)), math.sin(math.radians(phi))
    return ((P[:, 2] - sa2.GROUND_Z) * c - np.abs(P[:, 0]) * sn).min()


def rocker_package_report():
    from scipy.spatial import cKDTree
    global ARM_ROOT_L0_Z
    ARM_ROOT_L0_Z = D["ARM_ROOT_L0_Z_MIN"]
    mount = np.vstack([_pts_box(sh[1:4], sh[4:7]) for sh in D["ROCKER_MOUNT"].values()]); tm = cKDTree(mount)
    deck = _deck_points(); td = cKDTree(deck)
    arm_pts = lambda t: np.vstack([_pts_capsule(arm_point(a, t), arm_point(b, t), r) for a, b, r in _arm_root_segments()])
    gaps, inner, low = {}, {}, {}
    etas = (-30.0, -15.0, 0.0, 15.0, 30.0)
    for t in (TRAVEL[0], -16.0, 0.0, 16.0, TRAVEL[-1]):
        ta = cKDTree(arm_pts(t))
        for e in etas:
            B = rocker_bodies(e, t)
            for n, P in B.items():
                for oname, tree in (("rocker mount", tm), ("arm", ta), ("deck", td)):
                    g = tree.query(P)[0].min()
                    if g < gaps.get((n, oname), (99.0,))[0]: gaps[(n, oname)] = (g, e, t)
            for a, b in ROCKER_PAIRS:
                g = cKDTree(B[b]).query(B[a])[0].min()
                if g < inner.get((a, b), (99.0,))[0]: inner[(a, b)] = (g, e, t)
    # ground: body at 10 deg lean, low side +x, wheel travel of that side; rocker parts are body-fixed, the rods follow the side's travel
    for phi in (0.0, sa2.LEAN_DEG):
        tside = max(TRAVEL[0], min(TRAVEL[-1], sa2.side_travel(phi, 0.0, 1)))
        for extra in (-sa2.DROOP, 0.0, sa2.BUMP):
            tt = max(TRAVEL[0], min(TRAVEL[-1], tside + extra))
            for e in etas:
                for n, P in rocker_bodies(e, tt).items():
                    g = ground_clearance(P, phi)
                    if g < low.get((n, phi), (999.0,))[0]: low[(n, phi)] = (g, e, tt)
    ARM_ROOT_L0_Z = None
    print("\nROCKER PACKAGING (fix round 2), CALCULATED, surface point clouds at 0.5 mm (gaps read about 0.3 mm high at worst):")
    print("  rocker / rod ends among themselves (min over servo +-30 and travel):")
    for (a, b), (g, e, t) in inner.items():
        print("    %-14s <> %-14s %6.2f mm (horn %+.0f, travel %+.1f)" % (a, b, g, e, t))
    print("  to the rocker mount, the arm (inner boss r 11 x 62.2..70.2, root L0 z %.0f) and the deck:" % D["ARM_ROOT_L0_Z_MIN"])
    for oname in ("rocker mount", "arm", "deck"):
        worst = min(((g, n, e, t) for (n, o), (g, e, t) in gaps.items() if o == oname))
        print("    %-13s min %6.2f mm (%s, horn %+.0f, travel %+.1f)" % (oname, worst[0], worst[1], worst[2], worst[3]))
    print("  ground (body upright, and rolled 10 deg with the low side +x, each with -8 / 0 / +12 mm extra travel):")
    for phi in (0.0, sa2.LEAN_DEG):
        worst = min(((g, n, e, t) for (n, p), (g, e, t) in low.items() if p == phi))
        print("    lean %4.1f deg: lowest %6.2f mm (%s, horn %+.0f, travel %+.1f); requirement 18.0 at 10 deg" % (phi, worst[0], worst[1], worst[2], worst[3]))
    allmin = min([v[0] for v in inner.values()] + [v[0] for v in gaps.values()])
    print("  -> smallest gap %.2f mm (target >= 1.0): %s" % (allmin, "OK" if allmin >= 1.0 else "NOT OK"))


# ---- report ----------------------------------------------------------------------------------------------------------------------
def report_layout(lay, etas=(-30.0, 0.0, 30.0)):
    print("\n" + "-" * 118 + "\n" + lay.name + "   (tie rod %.2f mm)" % lay.L0)
    print("  BUMP STEER (deg) = delta(t, eta) - delta(0, eta), CALCULATED (3D)")
    print("  travel mm " + "".join("   horn %+4.0f" % e for e in etas))
    worst = {e: 0.0 for e in etas}
    for t in TRAVEL:
        row = ""
        for e in etas:
            b = lay.track(e, t) - lay.track(e, 0.0)
            worst[e] = max(worst[e], abs(b)); row += "   %+9.3f" % b
        print("   %+6.1f  " % t + row)
    print("  max |bump steer|: " + ", ".join("horn %+.0f: %.3f" % kv for kv in worst.items()))
    print("  wheel angle at horn " + " / ".join("%+.0f" % e for e in etas) + " (t 0): " + " / ".join("%+.2f" % lay.track(e, 0.0) for e in etas) + " deg")
    tmin = 90.0
    print("  TRANSMISSION ANGLES (deg; 90 ideal, 0 = dead centre; min over the whole travel), CALCULATED:")
    for e in etas:
        parts = {}
        for t in TRAVEL:
            d = lay.track(e, t)
            vals = {"knuckle/tie rod": lay.knuckle_trans(d, e, t)[0]}
            if lay is LAY_A:
                vals["horn/tie rod"] = crank_trans(S_A, XA, lay.inner(e), lay.kball(d, t))[0]
            else:
                rin = R_P + rot(RIN0 - R_P, R_AX, rocker(e))
                vals["horn/drag link"] = crank_trans(S_B, XA, horn_B(e), rin)[0]
                vals["rocker/drag link"] = crank_trans(R_P, R_AX, rin, horn_B(e))[0]
                vals["rocker/tie rod"] = crank_trans(R_P, R_AX, lay.inner(e), lay.kball(d, t))[0]
            for k, v in vals.items(): parts[k] = min(parts.get(k, 90.0), v)
        tmin = min([tmin] + list(parts.values()))
        print("    horn %+3.0f: " % e + ", ".join("%s %.1f" % kv for kv in parts.items()))
    print("    minimum %.1f deg -> %s" % (tmin, "no dead centre" if tmin > 30 else "CLOSE TO DEAD CENTRE"))
    return worst, tmin


def main():
    print("=" * 118)
    print("Fugu V2 steering kinematics, +x side (tools/steer_kin.py), snapshot 7b79b4c. Travel %.1f .. %+.1f mm, horn +-30 deg." % (TRAVEL[0], TRAVEL[-1]))
    dv = derived()
    for k in ("TIE_ROD_INNER_BALL", "ROCKER_PIN", "HORN_BALL", "TIE_ROD_L", "DRAG_LINK_L"):
        assert np.allclose(np.array(D[k], float), dv[k], atol=0.06), "STEER_DESIGN[%s] = %s, derived %s" % (k, D[k], dv[k])
    print("STEER_DESIGN derived entries re-checked (|diff| <= 0.06 mm). Horn %.1f mm, %.1f deg forward of straight down." % (D["HORN_L"], dv["HORN_FWD_DEG"]))

    # cross-check of the 3D solver against the thread's own steer_angle (layout A)
    print("\nCROSS-CHECK vs thread sa2.steer_angle (layout A, +x side), deg:")
    global PITCH_ON
    for t, e in ((0.0, 30.0), (37.9, -30.0), (-32.9, -30.0), (16.0, 15.0)):
        PITCH_ON = False; LAY_A._c.clear(); a0 = LAY_A.track(e, t)
        PITCH_ON = True; LAY_A._c.clear(); a1 = LAY_A.track(e, t)
        print("  t %+5.1f horn %+4.0f: thread %+8.4f | steer_kin, pure translation %+8.4f | steer_kin, real 4-bar pitch %+8.4f" % (t, e, sa2.steer_angle(t, e, 1), a0, a1))
    print("  carrier pitch from the 4-bar (upper link 100.14 vs arm 99.99 mm): %+.4f deg at %+.1f, %+.4f deg at %+.1f (CALCULATED); used below"
          % (carrier_pitch(TRAVEL[0]), TRAVEL[0], carrier_pitch(TRAVEL[-1]), TRAVEL[-1]))

    kp, kb = np.array(D["KINGPIN_POINT"]), np.array(D["KNUCKLE_BALL"]); arm = kb - kp
    print("\nKINGPIN (layout B): vertical through (%.1f, %.2f), x MEASURED from the old stand-in; scrub at the ground %.1f mm, trail 0, caster 0, KPI 0 (CALCULATED from it)."
          % (kp[0], kp[1], sa2.HALF_TRACK - kp[0]))
    print("  knuckle arm %.2f mm, %.1f deg forward of straight inboard (CALCULATED); knuckle ball radius from the axle %.1f mm (inboard of the tyre face, x < 113)"
          % (np.linalg.norm(arm[:2]), -math.degrees(math.atan2(arm[1], -arm[0])), math.hypot(kb[1] - sa2.TIP[0], kb[2] - sa2.TIP[1])))

    print("\nLOCK LIMIT from the swept tyre, gap >= %.0f mm at every travel (CALCULATED from hard points + ASSUMED envelopes):" % TYRE_GAP)
    lims = {}
    for key, note in (("arm", "Nov 21 outer truss as the v2 arm shape, ASSUMED"),
                      ("upper link", "outer hinge x %.1f, envelope r %.1f ASSUMED" % (D["UPPER_LINK_OUT"][0], LINK_R))):
        lims[key] = (lock_limit(LAY_B, +1, key), lock_limit(LAY_B, -1, key))
        print("  %-12s +%5.2f / -%5.2f deg   (%s; + = rear of the tyre swings inboard)" % (key, lims[key][0], lims[key][1], note))
    lim_p = lims["arm"][0]
    print("  -> design lock LOCK_DEG = +-%.1f (servo mixing keeps the outer wheel at the Ackermann angle; +-%.1f both ways is the failsafe case)"
          % (D["LOCK_DEG"], D["LOCK_DEG"]))
    for xo in (105.0, 103.0, 101.0, 99.0, 97.0, 95.0):
        g = min(min((_seg_dist(_steered(LAY_B, dd, t), a, b) - LINK_R).min() for a, b in link_polyline(t, xo))
                for dd in (-D["LOCK_DEG"], -D["LOCK_DEG"] / 2, D["LOCK_DEG"] / 2, D["LOCK_DEG"]) for t in TRAVEL)
        print("    upper link outer hinge x %.0f: swept tyre gap over +-%.1f deg and all travel %+.2f mm" % (xo, D["LOCK_DEG"], g))
    print("  Ackermann (ASSUMED wheelbase %.0f, kingpin spacing %.0f): inner %.1f -> ideal outer %.1f deg; done by servo mixing (one servo per wheel)"
          % (WHEELBASE, 2 * D["KINGPIN_POINT"][0], lim_p, ackermann_outer(lim_p)))

    wa, ta = report_layout(LAY_A)
    wb, tb = report_layout(LAY_B)
    for e in (-30.0, 30.0):
        g = min((tyre_gaps(LAY_B, LAY_B.track(e, t), t, e)["tie rod"] for t in TRAVEL))
        print("  B tyre to tie rod at horn %+.0f, min over travel: %.1f mm (CALCULATED)" % (e, g))
    print("  B wheel angles at horn +-30: %+.2f / %+.2f deg; right turn: +x wheel inner %.2f, -x wheel (mirror) outer %.2f; ideal outer for %.2f inner: %.2f"
          % (LAY_B.track(30, 0), LAY_B.track(-30, 0), LAY_B.track(30, 0), -LAY_B.track(-30, 0), LAY_B.track(30, 0), ackermann_outer(LAY_B.track(30, 0))))
    print("  -> mixing: limit the outer servo so its wheel stops at the ideal outer angle; the inner servo uses its full +-30.")

    # torque and forces (layout B)
    N = MASS_KG * 9.81 * FRONT_SHARE
    M_kp = MU * N * ((sa2.HALF_TRACK - kp[0]) + PATCH_R) / 1000.0 * DYN
    rmax, F_tie, F_drag = 0.0, 0.0, 0.0
    for e in np.linspace(-30, 30, 7):
        for t in TRAVEL:
            r = abs(LAY_B.track(min(e + 0.5, 30.5), t) - LAY_B.track(e - 0.5, t)) / 1.0       # d delta / d eta
            rmax = max(rmax, r)
            d = LAY_B.track(e, t)
            M_stall = SERVO_STALL_HIGH * 1000.0 / r                                            # kingpin moment with the wheel blocked
            F_tie = max(F_tie, M_stall / LAY_B.knuckle_trans(d, e, t)[1])
            rin = R_P + rot(RIN0 - R_P, R_AX, rocker(e))
            F_drag = max(F_drag, SERVO_STALL_HIGH * 1000.0 / crank_trans(S_B, XA, horn_B(e), rin)[1])
    T_req = M_kp * rmax
    print("\n" + "=" * 118 + "\nTORQUE, FORCES, BUCKLING (layout B)")
    print("  kingpin moment ASSUMED: wheel load %.2f N (share %.2f of %.3f kg), mu %.1f, lever scrub %.1f + patch %.0f mm, x %.1f dynamic = %.3f N.m"
          % (N, FRONT_SHARE, MASS_KG, MU, sa2.HALF_TRACK - kp[0], PATCH_R, DYN, M_kp))
    print("  max d(delta)/d(eta) %.3f (CALCULATED) -> servo torque needed %.3f N.m; x %.2f margin = %.3f N.m vs ASSUMED servo %.2f N.m: %s (reserve over the margined need %.0f %%)"
          % (rmax, T_req, TORQUE_MARGIN, T_req * TORQUE_MARGIN, SERVO_TORQUE_LOW, "OK" if T_req * TORQUE_MARGIN <= SERVO_TORQUE_LOW else "NOT OK",
             (SERVO_TORQUE_LOW / (T_req * TORQUE_MARGIN) - 1) * 100))
    Pt, Pd = euler(LAY_B.L0), euler(LD0)
    print("  servo stall ASSUMED %.2f N.m, wheel blocked, worst pose: tie rod %.0f N, drag link %.0f N (CALCULATED)" % (SERVO_STALL_HIGH, F_tie, F_drag))
    print("  Euler, pinned-pinned, steel M3 core %.3f mm, length ball to ball (conservative): tie rod %.1f mm -> %.0f N, margin %.2f; drag link %.1f mm -> %.0f N, margin %.1f"
          % (M3_CORE, LAY_B.L0, Pt, Pt / F_tie, LD0, Pd, Pd / F_drag))
    Pt4 = euler(LAY_B.L0) * (3.141 / M3_CORE) ** 4
    print("  M4 threaded rod (core 3.141 mm) as the tie rod: %.0f N, margin %.2f (CALCULATED) -> recommended; M3 is only %.2f" % (Pt4, Pt4 / F_tie, Pt / F_tie))

    envelope_report()
    print("\nROD-END MISALIGNMENT (tilt of the link out of the plane square to the stud; min .. max over servo +-30 and all travel), CALCULATED:")
    for n, (lo, hi) in rod_end_cones().items():
        print("  %-36s %5.1f .. %5.1f deg" % (n, lo, hi))
    print("  -> rod ends need a misalignment cone of at least the max value each side (+5 deg reserve); swivel about the stud is free")
    global ARM_ROOT_L0_Z
    for z, txt in ((None, "Nov 21 arm root (L0 at z 34.1)"), (D["ARM_ROOT_L0_Z_MIN"], "root node L0 raised to z %.1f (STEER_DESIGN)" % D["ARM_ROOT_L0_Z_MIN"])):
        ARM_ROOT_L0_Z = z
        print("\nLINKAGE vs ARM ROOT, %s; min gap mm over servo +-30 and all travel, CALCULATED (part sizes ASSUMED):" % txt)
        for n, (g, e, t) in linkage_vs_arm().items():
            print("  %-24s %6.2f  (horn %+.0f, travel %+.1f)" % (n, g, e, t))
    ARM_ROOT_L0_Z = D["ARM_ROOT_L0_Z_MIN"]
    mv, axg, bg, hg = mount_vs_arm()
    print("\nROCKER MOUNT (clamp on the pivot axle between the arm plates), CALCULATED, arm root raised as above:")
    for n, (g, t) in mv.items():
        print("  %-24s to arm members %6.2f mm (travel %+.1f)" % (n, g, t))
    print("  axial gap clamp to pivot bosses %.2f / %.2f mm; foot to outer boss underside %.2f mm; rocker hub to foot %.2f mm" % (axg[0], axg[1], bg, hg))
    lo_z = D["ROCKER_PIN_SPAN_Z"][0]
    print("  lowest rocker part z %.1f at x %.1f: %.1f mm off the ground at %.0f deg lean (low side), deck front 18.0" % (lo_z, R_P[0] + 5, sa2.clearance_at_lean(R_P[0] + 5, lo_z), sa2.LEAN_DEG))
    ARM_ROOT_L0_Z = None
    rocker_package_report()
    print("\nSUMMARY  A thread: max bump steer %.2f deg (horn +-30), min transmission %.1f deg.   B chosen: max bump steer %.4f deg, min transmission %.1f deg."
          % (max(wa.values()), ta, max(wb.values()), tb))


if __name__ == "__main__":
    main()
