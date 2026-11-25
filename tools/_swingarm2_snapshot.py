"""Front end, version 2 of the leading swing arms (hard points and kinematics only, no shapes yet).

World frame of the model (mm, Z up, front = -Y, ground z = -15.6), +x side; the -x side is the mirror image.

Borrowed from the swing arm front end of November 21 (commit 06452ea: parts/frontarm.py, uplink.py, steer_hw.py, rwd.py):
- leading arm, 100 mm, on a fixed 5 mm axle along x in two deck blocks; parallel upper link 50.4 mm above it, carrier, knuckle, one servo per side.
  Arm and link hinge on x axes fixed in the body, so the carrier keeps its attitude to the body: the wheels lean with the body.

New in version 2:
- Travel: every joint is laid out for +38 / -33 mm of wheel travel (10 deg of lean plus 12 mm bump / 8 mm droop). It was +20 / -14.
- Spring: a lever on each arm root (a tapered blade in the arm's inner plane, x LEVER_X) reaches up and back. A ball on each lever top carries
  one end of a floating balance beam; ONE heave shock runs from the beam's centre back along the centre line to an eye under a new cage spine.
  Bump moves both lever tops back and compresses the shock; lean moves one back and one forward, the beam yaws and the shock keeps its length.
  (A cross shock straight between the two arms would do the opposite: sprung in lean, free in bump.)
- Steering: the tie rod is a third parallel link, as long as the arm and parallel to it (horn ball minus knuckle ball = arm pivot minus arm tip),
  so wheel travel does not steer the wheel. The knuckle ball stays where it was on November 21 (steering arm square to the rod). The servo lies
  on the deck UNDER the pivot axle with its horn pointing straight down (square to the rod); the axle's deck blocks move inboard for it.

Priorities set by the owner (November 23): centre of gravity, then ground clearance, then articulation.
Run `python -I tools/front2_check.py` for the numbers.
"""
import math

GROUND_Z = -15.6
HALF_TRACK = 144.0
TYRE_R, TYRE_W = 56.2, 62.0
TYRE_RC = TYRE_W / 2                    # shoulder radius used for the contact height of a leaning wheel (as in parts/wishbone.py)

# ---- borrowed hard points (06452ea, moved to the world frame: y + 9.0, z - 15.3) ----
PIVOT = (-116.8, 47.5)                  # arm pivot axle (y, z), along x
TIP = (-216.55, 40.6)                   # arm tip = wheel centre (y, z)
ARM_L = math.hypot(TIP[0] - PIVOT[0], TIP[1] - PIVOT[1])
ARM_PLATES_X = (66.2, 89.5)             # inner and outer truss plane of the arm
BLOCK_X = (40.0, 54.5)                  # pivot block on the deck
UPLINK_IN = (68.5, -119.4, 97.9)        # upper link, inner hinge (on a cage lug)
UPLINK_OUT = (105.0, -219.3, 91.0)      # upper link, outer hinge (on top of the carrier)
HORN_X = 69.0

# ---- targets ----
LEAN_DEG = 10.0
BUMP, DROOP = 12.0, 8.0

# ---- new: spring lever, balance beam, heave shock ----
LEVER_X = ARM_PLATES_X[0]               # the lever's two struts lie in the arm's inner truss plane
LEVER_BALL_X = 54.0                     # ball on a stud pointing inboard from the lever top; the beam's end cup sits on it
LEVER_R = 55.0                          # pivot axle to lever top
LEVER_TILT = 40.0                       # deg, lever top behind the pivot at rest: the blade stays behind the upper link lug (y -113.4) at full droop
SHOCK_L = 68.0                          # heave shock, eye to eye at rest (the bought shock of the wishbone front end)
CAGE_RAIL_TOP = 64.4                    # top of the cage's lower rail once it is lowered to z 60 (the beam passes over it)
SPINE_Z = 114.0                         # cage spine and its two cross tubes (y -48 and y 38); the shock's rear eye hangs under the spine

# ---- new: steering as a third parallel link ----
KNUCKLE_BALL = (106.5, -213.6, 14.6)    # tie rod ball on the knuckle's steering arm (point of the old stand-in: 3 mm behind and 26 mm under the axle)
KNUCKLE_BALL_YZ = KNUCKLE_BALL[1:]
HORN_BALL = (HORN_X, PIVOT[0] + KNUCKLE_BALL[1] - TIP[0], PIVOT[1] + KNUCKLE_BALL[2] - TIP[1])    # rod parallel to the arm and as long as it
SERVO_BOX = (26.5, 40.8, 20.2)          # x, y, z of the low-profile servo lying on the deck, output shaft along x (AGFRC B44BLS class)
SERVO_X = (32.0, 58.5)                  # the horn (x 59.2 .. 61.5) stays inboard of the arm's inner plate (x 62.2)
SERVO_SHAFT_IN = 10.2                   # output shaft from the servo's front end
SERVO_TILT = 4.0                        # deg: the servo lies parallel to the deck slope (0.07 mm per mm)
SERVO_POCKET = 2.0                      # mm: seat let into the deck top under the servo, so its top stays 1.8 mm under the pivot axle
SERVO_SHAFT = (HORN_BALL[1], 33.3)      # output axis (y, z) straight above the horn ball and under the pivot axle (axle underside z 45.0)
HORN_L = SERVO_SHAFT[1] - HORN_BALL[2]  # horn points straight down: square to the rod at centre
PIVOT_BLOCK_X = (12.0, 30.0)            # the axle's two deck blocks move inboard (were x 40 .. 54.5) to make room for the servos under the axle

_REST = math.atan2(TIP[1] - PIVOT[1], -(TIP[0] - PIVOT[0]))      # arm slope at rest (rad, + = tip above the pivot)


def travel_for_lean(phi, s):
    """Wheel travel (mm, relative to the body) that keeps the wheel of side s on the ground with the body rolled by phi deg (+phi lowers +x)."""
    h = (TYRE_R - TYRE_RC) * math.cos(math.radians(phi)) + TYRE_RC
    c, sn = math.cos(math.radians(phi)), math.sin(math.radians(phi))
    # wheel centre in the body frame (s * HALF_TRACK, TIP z + t), rolled about the ground line: its height must be h
    return (h + sn * s * HALF_TRACK) / c - (TIP[1] - GROUND_Z)


def arm_swing(t):
    """Arm rotation (rad, + = tip up) for wheel travel t."""
    return math.asin((TIP[1] - PIVOT[1] + t) / ARM_L) - _REST


def wheel_shift(t):
    """How far the wheel centre moves toward the rear (mm) at wheel travel t."""
    return ARM_L * math.cos(_REST) - ARM_L * math.cos(_REST + arm_swing(t))


def lever_top(t):
    """Lever top (y, z) at wheel travel t: tip up turns the top toward the rear."""
    a = math.radians(LEVER_TILT) + arm_swing(t)
    return PIVOT[0] + LEVER_R * math.sin(a), PIVOT[1] + LEVER_R * math.cos(a)


def shock_stroke(t_plus, t_minus):
    """Compression of the heave shock (mm) for the wheel travels of the +x and -x side: the beam centre follows the mean of the lever tops."""
    y0 = lever_top(0.0)[0]
    return (lever_top(t_plus)[0] + lever_top(t_minus)[0]) / 2 - y0


def beam_yaw(t_plus, t_minus):
    """Yaw of the balance beam (deg)."""
    return math.degrees(math.atan2(lever_top(t_plus)[0] - lever_top(t_minus)[0], 2 * LEVER_BALL_X))


def bump_steer(t, lever=20.0):
    """Steering error (deg) at wheel travel t for a steering arm of the given length: zero for the parallel tie rod."""
    a = _REST + arm_swing(t)
    ky = KNUCKLE_BALL_YZ[0] + wheel_shift(t)
    kz = KNUCKLE_BALL_YZ[1] + t
    L0 = math.hypot(HORN_BALL[1] - KNUCKLE_BALL_YZ[0], HORN_BALL[2] - KNUCKLE_BALL_YZ[1])
    return math.degrees((math.hypot(HORN_BALL[1] - ky, HORN_BALL[2] - kz) - L0) / lever)


def clearance_at_lean(x, z, phi=LEAN_DEG):
    """Height above the ground (mm) of the body point (|x|, z) on the low side with the body rolled by phi deg."""
    c, sn = math.cos(math.radians(phi)), math.sin(math.radians(phi))
    return (z - GROUND_Z) * c - abs(x) * sn


# ---------------------------------------------------------------------------------------------------------------------------------
# 3D pose of the moving parts (body frame = world frame at rest). Used by tools/front2_pose.py and the animation.
KINGPIN_X = 123.0                       # kingpin (vertical, through the old drive joint stand-in 'mock joint': x 118 .. 128); ESTIMATE until the knuckle is redrawn
BEAM_EYE_BACK = 3.0                     # the shock's front eye sits this far behind the beam's ball line


def side_travel(phi, extra, s):
    """Wheel travel of side s (mm, relative to the body) for body roll phi (deg, + lowers +x) plus `extra` mm of travel on both wheels."""
    return travel_for_lean(phi, s) + extra


def lever_ball(t, s):
    y, z = lever_top(t)
    return (s * LEVER_BALL_X, y, z)


def horn_ball(eta, s):
    """Horn ball of side s with the servo turned by eta deg. + eta pushes the +x rod forward and the -x rod back (both wheels steer the same way)."""
    a = math.radians(s * eta)
    dy, dz = HORN_BALL[1] - SERVO_SHAFT[0], HORN_BALL[2] - SERVO_SHAFT[1]
    return (s * HORN_BALL[0], SERVO_SHAFT[0] + dy * math.cos(a) + dz * math.sin(a), SERVO_SHAFT[1] - dy * math.sin(a) + dz * math.cos(a))


def knuckle_ball(t, delta, s):
    """Tie rod ball on the knuckle of side s at wheel travel t, knuckle steered by delta deg about the kingpin (right hand about +z)."""
    sh = wheel_shift(t)
    kx, ky = s * KINGPIN_X, TIP[0] + sh
    rx, ry = s * KNUCKLE_BALL[0] - kx, KNUCKLE_BALL[1] + sh - ky
    c, sn = math.cos(math.radians(delta)), math.sin(math.radians(delta))
    return (kx + c * rx - sn * ry, ky + sn * rx + c * ry, KNUCKLE_BALL[2] + t)


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


ROD_L = _dist(HORN_BALL, KNUCKLE_BALL)


def steer_angle(t, eta, s):
    """Knuckle steer angle (deg, right hand about +z) of side s that keeps the tie rod's length, or nan if the rod cannot reach."""
    h = horn_ball(eta, s)
    f = lambda d: _dist(knuckle_ball(t, d, s), h) - ROD_L
    # the root nearest to straight ahead: scan outward from 0 in both directions
    best = None
    for sign in (1, -1):
        prev, dprev = f(0.0), 0.0
        for i in range(1, 121):
            d = sign * i * 0.5
            cur = f(d)
            if prev == 0.0 or prev * cur < 0:
                lo, hi = dprev, d
                for _ in range(60):
                    mid = (lo + hi) / 2
                    if f(lo) * f(mid) <= 0: hi = mid
                    else: lo = mid
                r = (lo + hi) / 2
                if best is None or abs(r) < abs(best): best = r
                break
            prev, dprev = cur, d
    return float('nan') if best is None else best


def transmission_angles(t, eta, s):
    """(rod to steering arm, rod to horn) in deg, measured in 3D between the rod and each lever: 90 is ideal, near 0 or 180 is dead centre."""
    d = steer_angle(t, eta, s)
    h, kb = horn_ball(eta, s), knuckle_ball(t, d, s)
    sh = wheel_shift(t)
    kp = (s * KINGPIN_X, TIP[0] + sh, kb[2])
    sv = (h[0], SERVO_SHAFT[0], SERVO_SHAFT[1])

    def ang(u, v):
        du = math.sqrt(sum(c * c for c in u)); dv = math.sqrt(sum(c * c for c in v))
        return math.degrees(math.acos(max(-1.0, min(1.0, sum(u[i] * v[i] for i in range(3)) / (du * dv)))))
    rod = tuple(h[i] - kb[i] for i in range(3))
    return ang(rod, tuple(kp[i] - kb[i] for i in range(3))), ang(tuple(-c for c in rod), tuple(sv[i] - h[i] for i in range(3)))


def beam_pose(t_plus, t_minus):
    """Ball centres of the +x and -x lever, the beam's stretch (ball distance minus its rest value) and the shock's eye-to-eye length change (3D).
    The beam is taken to sit centred between the two balls (one end cup must be a slot that takes the stretch)."""
    bp, bm = lever_ball(t_plus, 1), lever_ball(t_minus, -1)
    stretch = _dist(bp, bm) - 2 * LEVER_BALL_X
    mid = tuple((bp[i] + bm[i]) / 2 for i in range(3))
    L = _dist(bp, bm)
    ux, uy = (bp[0] - bm[0]) / L, (bp[1] - bm[1]) / L                       # beam axis in plan
    n = math.hypot(ux, uy)
    eye = (mid[0] - uy / n * BEAM_EYE_BACK, mid[1] + ux / n * BEAM_EYE_BACK, mid[2])      # the eye boss points to the rear, square to the beam
    y0, z0 = lever_top(0.0)
    rear = (0.0, y0 + BEAM_EYE_BACK + SHOCK_L, z0)
    return bp, bm, stretch, eye, rear, SHOCK_L - _dist(eye, rear)
