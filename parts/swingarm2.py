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
CAGE_RAIL_TOP = 60.3                    # top of the cage's lower rail (CAGE_Z 57 in parts/cage.py): the beam's fork corners come down to z 63.0
SPINE_Z = 114.0                         # cage spine and its two cross tubes (y -48 and y 38); the shock's rear eye hangs under the spine

# ---- new: steering as a third parallel link ----
KNUCKLE_BALL = (106.5, -213.6, 14.6)    # tie rod ball on the knuckle's steering arm (point of the old stand-in: 3 mm behind and 26 mm under the axle)
KNUCKLE_BALL_YZ = KNUCKLE_BALL[1:]
HORN_BALL = (HORN_X, PIVOT[0] + KNUCKLE_BALL[1] - TIP[0], PIVOT[1] + KNUCKLE_BALL[2] - TIP[1])    # rod parallel to the arm and as long as it
# Steering servo, worst-case envelope of the class (NOTES decision 27, owner: "... actual production off the shelf parts and measurements"):
# Tekin T-180 40 x 20 x 27 and base-to-tabs 18.4 (maker), Savox SC-1251MG+ 40.8 x 20.2 x 25.6 (maker), ear overhang 8.0 each end (no maker
# publishes it: worst case), tab 2.5 thick (ASSUMED). The case base rests on the clip leg (x 31); a lower case than 27 mm gets a printed shim
# there so the top face and the horn plane stay at x 58.5 (Savox 1.4 mm). Ear geometry: MEASURE ON ARRIVAL (docs/v2/bought_part_datasheets_2026-11-24.md).
SERVO_ENV = dict(L=40.8, W=20.2, H=27.0, tab_from_base=18.4, tab_t=2.5, ear_overhang=8.0)
# Savox SC-1251MG+ MEASURED on the maker's STEP (inputs/savox_sc1251mgp, owner approved the download): over the ears 54.5 (6.85 each end), tab
# 16.55 .. 18.95 above the case base (2.4 thick, rib 3.6), holes 49.5 x 10 (maker drawing), shaft 9.04 from the case end, centred in the width,
# spline 3.8 proud; a cable boss leaves the front end 4.4 .. 7.0 above the base, +-2.6 wide. Tekin's "base-to-tabs 18.4" does not say which face.
# The tab zone covers both servos in the world frame (Tekin base x 31.5, Savox base x 32.9 with its 1.4 shim): Savox 49.45 .. 51.85, Tekin
# 47.4 .. 49.9 (18.4 to the top face) or 49.9 .. 52.4 (to the bottom face).
SERVO_TAB_X = (47.4, 52.4)
SERVO_SHIFT = (3.3, 0.0)                # (dy rearward, dz up) against the first layout: the worst-case front ear clears the D5 pivot axle by 1.0 mm
                                        # (it reached 15.9 mm3 into it); the rear ear gets 1 mm pockets in the deck tops (model.py) instead of a raise,
                                        # which put the clip's strap into the cage rail (rail underside z 51.5 there)
SERVO_BOX = (SERVO_ENV['H'], SERVO_ENV['L'], SERVO_ENV['W'])   # x, y, z of the case lying on the deck, output shaft along x
SERVO_X = (58.5 - SERVO_ENV['H'], 58.5)     # the horn (x 59.2 .. 61.5) stays inboard of the arm's inner plate (x 62.2)
SERVO_SHAFT_IN = 10.2                   # output shaft from the servo's front end
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


# ---------------------------------------------------------------------------------------------------------------------------------
# Hard points from the orchestrator's workers (November 23). CALCULATED by tools/steer_kin.py (STEER_DESIGN) and tools/beam_kin.py (BEAM_DESIGN);
# docs/v2/steering.md and docs/v2/beam_guide.md explain them. The shapes in parts/swingarm_front.py are built on these.
# -- steering: rocker on a vertical pin under the arm root, drag link from the servo horn, tie rod parallel to the arm
KINGPIN = (123.0, TIP[0])               # vertical kingpin (x, y); x is still the estimate from the old drive joint stand-in
KINGPIN_Z = (21.0, 60.0)                # kingpin pin, bottom / top (knuckle ear faces z 22 .. 59)
KNUCKLE_BALL_B = (107.31, -222.45, 14.6)    # tie rod ball on the knuckle arm (16.76 mm arm, 20.6 deg forward of straight inboard)
ROD_INNER_BALL = (64.31, -122.70, 21.5)     # tie rod inner ball on the rocker's output arm (was x 69.81) (moved, owner's decision "Move the pin", NOTES 42: pin and tie rod ball 5.5 mm inboard, drag link ball 2.0 mm inboard)
ROCKER_PIN = (80.0, -116.8)             # vertical pin (x, y), under the hanger's full section (was x 85.5, at the thin foot's root)
ROCKER_IN_BALL = (64.0, -109.70, 24.5)  # (was x 66.0; input arm now 17.50 mm) drag link ball on the rocker's input arm (20.75 mm, 20 deg rearward of straight inboard: 4.8 mm between the rod end housings)
ROCKER_HUB_Z = (19.0, 33.0)             # rocker hub: its underside is 19.2 mm off the ground at 10 deg lean
ROCKER_PLATES_Z = ((27.0, 30.0), (30.0, 33.0))   # tie rod arm plate, drag link arm plate; both balls hang under their plate on 5.5 mm studs
SERVO_SHAFT_B = (-98.4 + SERVO_SHIFT[0], 38.3 + SERVO_SHIFT[1])   # servo output axis (y, z): lying flat on the deck behind the pivot axle
HORN_BALL_B = (68.5, -93.70, 26.38)     # 12 mm metal horn, 6.7 deg rearward of straight down, square to the drag link at centre (steer_kin derived(), servo shifted)
UPLINK_OUT_B = (101.0, -219.3, 91.0)    # upper link outer hinge moved inboard from x 105 (tyre gap at lock)
LOCK_DEG = 18.5                         # mechanical steering lock each way
ARM_ROOT_L0_Z = 45.0                    # lower root node of the arm (y -130) raised from z 34.1: the rocker and rod ends swing under it
LINKAGE_ZONE = (58.0, -133.0, 12.0, 92.0, -96.0, 33.0)     # no arm member in this box at any travel
# -- wheel (measured on the built model, 9da7463): barrel bore r 21.13 from x 127, inner hub face x 148, hex pocket r 6.9 from x 149 to 158, outer face x 161
WHEEL_BORE_R, WHEEL_IN_X, WHEEL_DISC_X, WHEEL_OUT_X = 21.13, 127.0, 148.0, 161.0
ROCKER_MOUNT_X = (71.2, 84.7, 88.5)     # clamp and hanger on the pivot axle from x 71.2 to 84.7 (the pin's top in it), foot out to the strap at x 88.5
ARM_FLOOR_Z = 38.5                      # no arm material below this over the mount (x 71.2 .. 90.5, y -121 .. -112) at any travel
# -- heave spring: bellcrank carrier under the spine's front cross tube, beam on a tilted yaw pin, cup blocks sliding in the beam's end forks
BEAM_C = (0.0, -81.45, 85.4)            # beam yaw pin on the carrier
BEAM_N = (0.0, 0.5990, 0.8007)          # yaw pin axis (tilted 36.8 deg back from vertical)
BEAM_M = (0.0, 0.8007, -0.5990)         # slot normal = push direction of a beam end
CARRIER_PC = (0.0, -50.6, 98.5)         # carrier pin along x, clevis 15.5 mm under the front cross tube
SHOCK_S = (0.0, -45.6, 81.5)            # shock front eye on the carrier
SHOCK_E = (0.0, 20.54, 97.3)            # shock rear eye, clevis under the spine
CARRIER_TAIL = (0.0, -42.1, 98.5)       # carrier tail with the heave stop dowel (round 3)
STOP_BUMP_GAP, STOP_DROOP_GAP = 2.67, 1.61     # the tail rises / drops this far to its pad: two-wheel +12 / -8 mm (carrier +15.46 / -9.28 deg)
STOP_YAW_GAP, STOP_YAW_X = 5.01, 20.0   # gap along M between the beam's rear face and the yaw pads on the carrier at |x| 20: +-14.5 deg of beam yaw = 10 deg lean
SHOCK_BODY_R, SHOCK_EYE_W = 8.25, 6.0   # ASSUMED 1/10 class shock: spring OD 16.5, eye width 6, 72 mm extended, 16 mm stroke


# ---------------------------------------------------------------------------------------------------------------------------------
# Build guard for the load-bearing printed parts of the front end. model.py calls guard() on each of them: the build FAILS if a part is not exactly
# one valid solid or if its volume is more than 1 % off the value below. Reason: a fuse once dropped two yaw pad posts from the beam carrier
# without any error (builds up to c725daa). When a part is changed on purpose, update its value here in the same commit (the failure prints it).
GUARD_VOLUMES = {      # Front carrier 16922 -> 14415 (Nov 24): kingpin block and web shortened to z 31..50 for the PETG knuckle
                       # Rocker bracket 6313.6 -> 6096.2 (Nov 24): clamp slit to the rear, M3 clamp screw and insert, blind pin bore (parts/swap_interface.py)
                       # mm3, +x side where there are two
                       # Rocker bracket 6096.2 -> 5558.0, Rocker 1485.5 -> 1407.6 (Nov 25, NOTES 42): pin moved 5.5 mm inboard into the hanger, D4 x 24
                       # Pad block 7430.1 -> 8682.3 (Nov 25, NOTES 49): gusset under the ear on the post, keel under the block's rear and the strut lump
                       # Beam carrier 49851.3 -> 50300.7 (Nov 26, NOTES 57): yaw pin boss r 11 with a cap, bore for the bronze sleeve, tail web 26.2
                       # Beam carrier 50300.7 -> 50456.8 (Nov 26, NOTES 57 attempt 2): 1 mm more hub wall under the carrier pin
                       # arm 49788.0 -> 52792.8 (Nov 26, NOTES 59): 3.5 mm outward rib on the spring lever's blade and strut, strut half width 5,
                       # inner upper chord U0-U1 D9.5; 54124.3 (attempt 3): crotch fillet R 10, back edge +4 at the root, rib end ramped;
                       # 53572.6 (attempt 3 repair): rib ramp from r 15.5 (was 14.5), back edge +2.5 (clearance to the Rocker bracket, horn and Rocker in motion)
                       # Beam carrier 50456.8 -> 50169.2, Pad block 8682.3 -> 9369.7 (Nov 26, NOTES 60, item 1(b)): tail at r 10.4 with two bronze
                       # sleeves (bore 12, top wall 1.2); pad block rear face 1.9 mm further back with the dowel's arc slot
                       # Pad block 9369.7 -> 9606.8 (NOTES 60, cage mesh fix): exact arc window, floor under the droop pockets to z 87.5 from y -44.6
                       # Pad block 9606.8 -> 10011.7 (NOTES 64, 4 mm bump saddle): stadium pockets the slot's width ending at |x| 35.9, 1.2 mm pad over them
    'Beam carrier': 50169.2, 'Balance beam': 27035.9, 'arm': 53572.6, 'Rocker bracket': 5558.0, 'Front carrier': 14415.0, 'Rocker': 1407.6, 'Pad block': 10011.7,
}
GUARD_TOL = 0.01


def guard(name, shape):
    """Check one load-bearing part; raises AssertionError (which fails `soma build`) with the measured numbers."""
    base = name.rsplit(' ', 1)[0] if name.rsplit(' ', 1)[-1] in ('1', '-1') else name
    if base not in GUARD_VOLUMES:
        return False
    s = shape.val() if hasattr(shape, 'val') else shape
    n, vol, want = len(s.Solids()), s.Volume(), GUARD_VOLUMES[base]
    assert s.isValid() and n == 1, 'GUARD: %s is not one valid solid (%d solids, valid %s)' % (name, n, s.isValid())
    assert abs(vol - want) <= GUARD_TOL * want, 'GUARD: %s volume %.1f mm3 differs from the expected %.1f by %.2f %% (limit 1 %%)' % (name, vol, want, 100.0 * (vol - want) / want)
    return True
