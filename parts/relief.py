"""Trim the scaled shell around the 1:1 chassis, including suspension and steering travel.

Tool data comes from tools/make_relief.py (parts/relief.json): hull vertices of every chassis part that reaches
into the shell, tire cylinders and the rear swingarm pivot. Frame = shell frame at 1:1 (X lateral, Y = -forward,
Z up, ground z = 0); the tools are moved by CHASSIS_Y / CHASSIS_Z.
"""
import json, math, os
import cadquery as cq
from parts.util import hull
from parts.params import *

_DATA = json.load(open(os.path.join(os.path.dirname(__file__), 'relief.json')))
PIVOT = (_DATA['pivot'][1], _DATA['pivot'][2])            # rear swingarm pivot (y, z)
REAR_AXLE = (_DATA['rear_axle'][1], _DATA['rear_axle'][2])  # rear axle at the original 112 mm tire
SHOCK_TOP0 = (0.0, 136.3, 129.5)     # upper eye on the frame (x, y, z), on the wheel centre plane, raised by REAR_SHOCK_RISE
SHOCK_BOT0 = (0.0, 150.0, 114.0)   # lower eye (arm frame): the pin of the shock bridge over the tire, final (215, 139) at ride
BRIDGE_ARM = [(77.0, 133.0, 114.0), (30.0, 133.0, 114.0), (-9.3, 133.0, 114.0), (-9.3, 157.5, 114.0), (9.3, 157.5, 114.0)]   # outer plate apex, arch, ears around the shock eye pin (new rear arm)   # shock bridge path in the arm frame (x, y, z): apex node, up, across
FRONT_X, FRONT_Y = 144.0, -225.55                         # front axle centres (|x|, y)


def rear_lift_angle():
    """Swingarm rotation (rad) that lifts the rear axle to the new tire radius."""
    dy, dz = REAR_AXLE[0]-PIVOT[0], REAR_AXLE[1]-PIVOT[1]
    L = math.hypot(dy, dz)
    return math.asin((REAR_TIRE_OD/2 - PIVOT[1])/L) - math.atan2(dz, dy)

def rot_yz(p, ang, c=PIVOT):
    """Rotate point p=(x,y,z) about the lateral axis through c=(y,z); positive raises the tail."""
    x, y, z = p; dy, dz = y-c[0], z-c[1]
    return (x, c[0] + dy*math.cos(ang) - dz*math.sin(ang), c[1] + dy*math.sin(ang) + dz*math.cos(ang))

def rot_xy(p, ang, c):
    """Rotate point about a vertical axis through c=(x,y) (steering)."""
    x, y, z = p; dx, dy = x-c[0], y-c[1]
    return (c[0] + dx*math.cos(ang) - dy*math.sin(ang), c[1] + dx*math.sin(ang) + dy*math.cos(ang), z)

def _grown(pts, g):
    out = []
    for x, y, z in pts:
        out += [(x+g, y, z), (x-g, y, z), (x, y+g, z), (x, y-g, z), (x, y, z+g), (x, y, z-g)]
    return out

def _circle(cx, cy, cz, r, hw, n=28):
    pts = []
    for i in range(n):
        a = 2*math.pi*i/n
        for sx in (-hw, hw):
            pts.append((cx+sx, cy + r*math.cos(a), cz + r*math.sin(a)))
    return pts

def _circle2(c, r, n=12):
    return [(c[0]+r*math.cos(2*math.pi*i/n), c[1]+r*math.sin(2*math.pi*i/n), c[2]+s*r) for i in range(n) for s in (-1, 1)]

FRONT_CLEAR = 3.0        # mm: clearance of the shell around the moving wishbone parts (their geometry is exact, unlike the old stand-in hulls)
FRONT_UP_DEG = 26.0      # arm angle upward the shell has to clear: 10 deg lean (15.6 deg at the arm) plus 12 mm bump (7.3 deg)
FRONT_STEER_DEG = 22.0   # knuckle steering angle each way
FRONT_TYRE_LIFT = 40.0   # mm: front wheel travel upward relative to the body: 25.4 at 10 deg lean, 12 bump, margin
FRONT_ARM_L = 91.0       # wishbone length (parts/wishbone.py ARM_L)


def wishbone_relief_tools():
    """Cut-out tools (world coordinates) for the travel of the new front end (parts/wishbone.py): each upper arm with its shock post swept upward
    about its pivot rod, each knuckle carried along and steered. Only what rises toward the shell matters; the lower arm stays under it."""
    from parts import wishbone as wb
    V = cq.Vector
    arm, kn = wb.a_arm(wb.ZU, True), wb.knuckle()
    A = [v.toTuple() for v in arm.Vertices()]
    K = [v.toTuple() for v in kn.Vertices()]
    tools = []
    for s in (1, -1):
        pa, pk = [], []
        for k in range(5):
            psi = -s * FRONT_UP_DEG * k / 4.0                      # wheel up on side s
            loc = wb._loc_axis((s * wb.XI, 0, wb.ZU), (0, 1, 0), psi)
            T = loc.wrapped.Transformation()
            for x, y, z in A:
                q = V(s * x, y, z).toPnt().Transformed(T); pa.append((q.X(), q.Y(), q.Z()))
            dx, dz = wb.arm_ball(s, psi)
            for d in (-FRONT_STEER_DEG, 0.0, FRONT_STEER_DEG):
                c, sn = math.cos(math.radians(d)), math.sin(math.radians(d))
                for x, y, z in K:
                    rx, ry = s * x - s * wb.KX, y - wb.KY
                    pk.append((s * wb.KX + c * rx - sn * ry + dx, wb.KY + sn * rx + c * ry, z + dz))
        tools.append(hull(_grown(pa, FRONT_CLEAR)))
        tools.append(hull(_grown(pk, FRONT_CLEAR)))
    # front tyres over the wishbone travel (lean plus bump lifts the wheel up to FRONT_TYRE_LIFT relative to the body and carries it inboard on the
    # arm's arc), steered FRONT_STEER_DEG each way about the kingpin: one hull per travel step, so the cut hugs the swept tyre
    L = math.radians(FRONT_STEER_DEG)
    for t in _DATA['tires']:
        if t['name'] == 'mock_rear_tire_ring': continue
        cx, cy, cz = t['center']; s = 1 if cx > 0 else -1
        lifts = [0.0, FRONT_TYRE_LIFT / 2, FRONT_TYRE_LIFT]
        ring = {}
        for lift in lifts:
            dx = -s * (FRONT_ARM_L - math.sqrt(FRONT_ARM_L ** 2 - lift ** 2))
            kx = s * wb.KX + dx
            ring[lift] = [rot_xy((p[0] + dx, p[1], p[2]), a, (kx, cy)) for a in (-L, 0, L)
                          for p in _circle(cx, cy, cz + lift, t['r'] + TIRE_GAP, t['hw'] + TIRE_GAP)]
        for l0, l1 in zip(lifts, lifts[1:]):
            tools.append(hull(ring[l0] + ring[l1]).translate(cq.Vector(0, CHASSIS_Y, CHASSIS_Z)))
    return tools


def relief_tools():
    """Cut-out tools in final (scaled) shell coordinates: tire arches and suspension travel."""
    off = cq.Vector(0, CHASSIS_Y, CHASSIS_Z)
    T = BUMP_TRAVEL; L = math.radians(STEER_LOCK)
    th0 = rear_lift_angle(); thb = math.asin(min(1.0, T/math.hypot(REAR_AXLE[0]-PIVOT[0], REAR_AXLE[1]-PIVOT[1])))
    tools = []
    # --- tires
    for t in _DATA['tires']:
        cx, cy, cz = t['center']
        if t['name'] == 'mock_rear_tire_ring':
            R = REAR_TIRE_OD/2
            pts = []
            for k in (0, 0.5, 1.0):                      # axle path along the swingarm arc
                c = rot_yz((cx, cy, cz), th0 + k*thb)
                pts += _circle(0, c[1], c[2], R + RELIEF, t['hw'] + RELIEF)
        else:
            pts = []
            for a in (-L, 0, L):                         # steering lock sweep about the wheel centre
                for lift in (0, T):                      # bump travel
                    pts += [rot_xy(p, a, (cx, cy)) for p in _circle(cx, cy, cz+lift, t['r'] + TIRE_GAP, t['hw'] + TIRE_GAP)]
        tools.append(hull(pts).translate(off))
    # --- suspension, steering and swingarm parts
    for h in _DATA['hulls']:
        P = [tuple(p) for p in h['pts']]; kind = h['kind']
        if kind != 'rear': continue      # the front hulls belong to the old front end (swingarms, uplinks, belts, lean servo): wishbone_relief_tools() replaces them
        if kind == 'susp':
            pts = P + [(x, y, z + T*min(1.0, max(0.0, (abs(x)-30)/114.0))) for x, y, z in P]
        elif kind == 'steer':
            cx = math.copysign(FRONT_X, P[0][0])
            pts = []
            for a in (-L, 0, L):
                for lift in (0, T):
                    pts += [rot_xy((x, y, z+lift), a, (cx, FRONT_Y)) for x, y, z in P]
        elif kind == 'rear':
            pts = [rot_yz(p, th0) for p in P] + [rot_yz(p, th0 + thb) for p in P]
        else:
            pts = P
        tools.append(hull(_grown(pts, RELIEF)).translate(off))
    # the wishbone travel (wishbone_relief_tools) is cut from the finished front shell in model.py
    # --- rear shock strut: top mount fixed on the frame (raised), bottom eye rides on the swingarm
    top = (SHOCK_TOP0[0], SHOCK_TOP0[1], SHOCK_TOP0[2] + REAR_SHOCK_RISE)
    pts = []
    for k in (0, thb):
        b = rot_yz(SHOCK_BOT0, th0 + k)
        for f in (0.0, 0.5, 1.0):
            pts += _circle2(tuple(top[i] + f*(b[i]-top[i]) for i in range(3)), 12.0 + RELIEF)
    tools.append(hull(pts).translate(off))
    # --- shock bridge over the rear tire swings with the arm
    bp = []
    for k in (0, thb):
        bp += [rot_yz(p, th0 + k) for p in BRIDGE_ARM]
    tools.append(hull(_grown(bp, RELIEF + 8.0)).translate(off))
    return tools


SWING_STEPS = (0.0, 14.3, 28.7, 43.0)     # wheel travel upward relative to the body: one wheel alone reaches +41.9 (no per-arm bump stop, NOTES decision 14), 1.1 margin
SWING_KINGPIN_X = 123.0                   # kingpin through the old drive joint stand-in (estimate, parts/swingarm2.py KINGPIN_X)


def swingarm_relief_tools():
    """Cut-out tools (world coordinates) for the travel of the version 2 swing arm front end: arm (with its spring lever), upper link, carrier,
    knuckle and tyre, moved the way the linkage moves them (arm and link turn about their hinges, carrier, knuckle and wheel ride on the arm tip
    without turning), the knuckle and tyre also steered STEER_LOCK each way. Only upward travel matters for the shell."""
    from parts import swingarm2 as sa, uplink as up
    off = cq.Vector(0, CHASSIS_Y, CHASSIS_Z)
    L = math.radians(STEER_LOCK)
    hi, _ = up.hinges()
    pv = (sa.PIVOT[0] - CHASSIS_Y, sa.PIVOT[1] - CHASSIS_Z)
    lt = sa.lever_top(0.0)
    hulls = {h['name']: [tuple(p) for p in h['pts']] for h in _DATA['hulls']}

    def turn(p, c, a):                       # about an x axis through c = (y, z): the arm tip (at -y) rises by a
        dy, dz = p[1] - c[0], p[2] - c[1]
        return (p[0], c[0] + dy * math.cos(a) + dz * math.sin(a), c[1] - dy * math.sin(a) + dz * math.cos(a))

    tools = []
    for s in (1, -1):
        arm = hulls['arm_%d' % s] + [(s * x, lt[0] - CHASSIS_Y + dy, lt[1] - CHASSIS_Z + dz) for x in (50.0, 70.0) for dy in (-6, 6) for dz in (-6, 6)]
        link = [(s * x, y, z) for x, y, z in up.sample_points()]
        pa, pl, pc, pk = [], [], [], []
        for t in SWING_STEPS:
            a = sa.arm_swing(t); sh = sa.wheel_shift(t)
            pa += [turn(p, pv, a) for p in arm]
            pl += [turn(p, (hi.y, hi.z), a) for p in link]
            pc += [(x, y + sh, z + t) for x, y, z in hulls['carrier_%d' % s]]
            for d in (-L, 0.0, L):
                pk += [rot_xy((x, y + sh, z + t), d, (s * SWING_KINGPIN_X, FRONT_Y + sh)) for x, y, z in hulls['knuckle_%d' % s]]
        for pts in (pa, pl, pc, pk):
            tools.append(hull(_grown(pts, FRONT_CLEAR)).translate(off))
    for t in _DATA['tires']:
        if t['name'] == 'mock_rear_tire_ring': continue
        cx, cy, cz = t['center']; s = 1 if cx > 0 else -1
        ring = {}
        for lift in SWING_STEPS:
            sh = sa.wheel_shift(lift)
            ring[lift] = [rot_xy(p, a, (s * SWING_KINGPIN_X, cy + sh)) for a in (-L, 0, L)
                          for p in _circle(cx, cy + sh, cz + lift, t['r'] + TIRE_GAP, t['hw'] + TIRE_GAP)]
        for l0, l1 in zip(SWING_STEPS, SWING_STEPS[1:]):
            tools.append(hull(ring[l0] + ring[l1]).translate(off))
    return tools
