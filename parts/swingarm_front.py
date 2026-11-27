"""Shapes of the version 2 swing arm front end (hard points and kinematics: parts/swingarm2.py). Prototype level: functional shapes, no fillets.

World frame (mm, Z up, front = -Y) unless a function says "ref" (frame of parts/chassis_ref.step: world = ref + (0, CHASSIS_Y, CHASSIS_Z)).
Shapes are built for the +x side and mirrored for -x.

Borrowed unchanged: the printed truss arm (parts/frontarm.py), the upper link with its clevises and pins (parts/uplink.py), the pivot axle,
pivot blocks and tip pins (parts/rwd.py).
New here: the spring lever on the arm root; carrier and knuckle (the old stand-ins sat inside the wheel); steering by a rocker on a vertical pin
under the arm root with a drag link from the servo horn and an M4 tie rod parallel to the arm (docs/v2/steering.md); the heave spring with a
bellcrank carrier, a balance beam on a tilted yaw pin, cup blocks in the beam's end forks and one shock under the cage spine (docs/v2/beam_guide.md).
"""
import math
import os
import cadquery as cq
from parts import swingarm2 as k
from parts.params import CHASSIS_Y, CHASSIS_Z
from parts.tubes import square_bar

V = cq.Vector
OFF = V(0, CHASSIS_Y, CHASSIS_Z)
G = 'Chassis/Front suspension'
PRINT, STEEL, SHOCK_RED, SERVO_BLACK, POM = (0.55, 0.57, 0.60), (0.78, 0.80, 0.84), (0.80, 0.15, 0.12), (0.12, 0.12, 0.13), (0.92, 0.92, 0.88)
BALL_R = 3.4               # bought 6.8 mm ball stud
STRUT = 8.0                # lever blade thickness (x), as thick as the arm's chord tubes
LEVER_ROOT, LEVER_TIP = 22.0, 11.0     # blade width at the pivot boss and at the top
LEVER_STRUT_R = 24.0       # the strut from U0 meets the blade's centre line this far from the pivot
LEVER_RIB = 3.5            # (NOTES 59) blade and strut thickened outward by this (x 70.2 .. 73.7): the ball's 12.2 mm offset twists the blade
LEVER_RIB_R0 = 19.0        # (attempt 3 repair: 18.0 -> 19.0, ramp starts at r 15.5 on the blade face; at 14.5 it swept into the Rocker bracket at the one-wheel corner) # the rib stays this far from the pivot axis: the pivot pulley and belt (x 70.2 .. 73.8, r <= 8.2) and the rocker
                           # bracket (x 71 .. 74 behind the pivot; NOTES 59 attempt 1 hit it at r 12) are there
LEVER_BACK = 2.5           # (attempt 3) back edge widened at the root, tapering to 0 at the top; 4.0 came 0.75 mm to the steer horn and 0.97 to the Rocker at the one-wheel corner, 2.5 keeps both over 1.5
LEVER_CROTCH_R = 10.0      # (NOTES 59 attempt 3) radius in the crotch between the strut's upper edge and the blade's front edge (FEM: x1.7 there)
LEVER_STRUT_W = 5.0        # (NOTES 59 attempt 2) half width of the strut from U0 to the blade (was 4): the strut crotch peaked
FORK = (6.0, 6.5, 13.5)    # beam end fork, half size along the beam / M / N (docs/v2/beam_guide.md)
PUCK_R, PUCK_H = 4.5, 4.5  # round puck on the lever ball: radius, half height along M
BEAM_SIDE = 10.0
HUB_R, HUB_H = 6.5, 8.0    # beam hub on the yaw pin: radius, half length along N
PIN_R, BORE_R = 1.5, 1.55  # 3 mm steel pins in reamed printed bores
CUP_R, CUP_H = 5.0, 5.0    # rod end cups (bought ball ends)
ROD_R = 2.0                # M4 tie rod (buckling margin 3.16 at servo stall; M3 only 1.05)
CHEEK = 3.0                # clevis and fork plates
TAIL_R = 3.5               # round end of the carrier's stop tail


def _mir(s, sx):
    return s if sx > 0 else s.mirror('YZ')


def _cyl(p, d, r, h):
    return cq.Solid.makeCylinder(r, h, V(*p), V(*d))


def _ball(r, c):
    """Full sphere as two fused halves. cq.Solid.makeSphere makes the upper half only by default (the first builds had half balls and half
    sockets); a one-piece full sphere fused with a stud across its seam comes back from the STEP file inside out (volume 81 instead of 248 mm3,
    found on build 30f7796 + fixes). Do NOT call .clean() on a ball stud: it merges the halves again and can crash."""
    return cq.Solid.makeSphere(r, c, V(0, 0, 1)).fuse(cq.Solid.makeSphere(r, c, V(0, 0, -1)))


def _box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def _fuse(*s):
    r = s[0]
    for q in s[1:]:
        r = r.fuse(q)
    return r.clean()


def _slab(p, rp, q, rq, x0, x1):
    """Flat link in a plane x = const: hull of a circle at p and one at q ((y, z) each), from x0 to x1."""
    L = math.hypot(q[0] - p[0], q[1] - p[1])
    ny, nz = -(q[1] - p[1]) / L, (q[0] - p[0]) / L
    web = cq.Workplane('YZ').workplane(offset=x0).polyline([(p[0] + rp * ny, p[1] + rp * nz), (q[0] + rq * ny, q[1] + rq * nz),
                                                             (q[0] - rq * ny, q[1] - rq * nz), (p[0] - rp * ny, p[1] - rp * nz)]).close().extrude(x1 - x0).val()
    return _fuse(web, _cyl((x0, p[0], p[1]), (1, 0, 0), rp, x1 - x0), _cyl((x0, q[0], q[1]), (1, 0, 0), rq, x1 - x0))


def _plate(p, rp, q, rq, z0, z1):
    """Flat link in a plane z = const: hull of a circle at p and one at q ((x, y) each), from z0 to z1."""
    L = math.hypot(q[0] - p[0], q[1] - p[1])
    nx, ny = -(q[1] - p[1]) / L, (q[0] - p[0]) / L
    web = cq.Workplane('XY').workplane(offset=z0).polyline([(p[0] + rp * nx, p[1] + rp * ny), (q[0] + rq * nx, q[1] + rq * ny),
                                                             (q[0] - rq * nx, q[1] - rq * ny), (p[0] - rp * nx, p[1] - rp * ny)]).close().extrude(z1 - z0).val()
    return _fuse(web, _cyl((p[0], p[1], z0), (0, 0, 1), rp, z1 - z0), _cyl((q[0], q[1], z0), (0, 0, 1), rq, z1 - z0))


def _local(origin):
    """Plane at `origin` with x along the beam (world x), y along M, z along N."""
    return cq.Plane(origin=origin, xDir=(1, 0, 0), normal=k.BEAM_N)


def _lbox(pl, x0, x1, m0, m1, n0, n1):
    return cq.Workplane(pl).workplane(offset=n0).center((x0 + x1) / 2, (m0 + m1) / 2).rect(x1 - x0, m1 - m0).extrude(n1 - n0).val()


def _ncyl(pl, r, n0, n1):
    return cq.Workplane(pl).workplane(offset=n0).circle(r).extrude(n1 - n0).val()


# ---------------------------------------------------------------------------------------------------------------- spring lever, beam, shock
def lever_top_w():
    """Lever top node at rest, world (x, y, z)."""
    y, z = k.lever_top(0.0)
    return (k.LEVER_X, y, z)


def lever_ref(sx=1):
    """Spring lever (ref frame), to be fused into the printed arm: a tapered blade in the inner truss plane from the pivot boss up and back to the
    top, and a boss toward the inside that carries the ball stud. By hand: 118 N at the top (65 N at the wheel), 6.5 N m at the root,
    8 x 22 mm section: about 10 MPa (FEM still to do)."""
    from parts.frontarm import PIVOT_YZ
    x = k.LEVER_X
    top = V(*lever_top_w()) - OFF
    a = math.radians(k.LEVER_TILT)
    sj = (PIVOT_YZ[0] + LEVER_STRUT_R * math.sin(a), PIVOT_YZ[1] + LEVER_STRUT_R * math.cos(a))

    def outline(x0, x1):
        """Blade + strut + back widening + crotch fillet, as one plan outline extruded from x0 to x1."""
        o = _slab((PIVOT_YZ[0], PIVOT_YZ[1]), LEVER_ROOT / 2, (top.y, top.z), LEVER_TIP / 2, x0, x1)
        # strut from the upper chord root U0 to the blade at r 24: it closes the re-entrant corner between the blade's front edge and the neck
        # (FEM: 17.3 MPa, safety 1.16 there)
        o = o.fuse(_slab((-139.0, 75.5), LEVER_STRUT_W, sj, LEVER_STRUT_W, x0, x1))
        P = (PIVOT_YZ[0], PIVOT_YZ[1]); Lb = math.hypot(top.y - P[0], top.z - P[1])
        d = ((top.y - P[0]) / Lb, (top.z - P[1]) / Lb); n = (-d[1], d[0])         # blade direction and its front-edge normal
        if LEVER_BACK > 0:
            # (NOTES 59 attempt 3) back edge moved back by LEVER_BACK at the root, 0 at the top
            rb = LEVER_ROOT / 2 + LEVER_BACK
            pts = [(P[0] - n[0] * (LEVER_ROOT / 2 - 1.0), P[1] - n[1] * (LEVER_ROOT / 2 - 1.0)), (P[0] - n[0] * rb, P[1] - n[1] * rb),
                   (top.y - n[0] * LEVER_TIP / 2, top.z - n[1] * LEVER_TIP / 2), (top.y - n[0] * (LEVER_TIP / 2 - 1.0), top.z - n[1] * (LEVER_TIP / 2 - 1.0))]
            o = o.fuse(cq.Workplane('YZ').workplane(offset=x0).polyline(pts).close().extrude(x1 - x0).val())
        if LEVER_CROTCH_R > 0:
            # (NOTES 59 attempt 3) fillet piece in the crotch of the strut's upper edge and the blade's front edge
            Ls = math.hypot(sj[0] + 139.0, sj[1] - 75.5); ds = ((sj[0] + 139.0) / Ls, (sj[1] - 75.5) / Ls); ns = (-ds[1], ds[0])
            A0 = (-139.0 + ns[0] * LEVER_STRUT_W, 75.5 + ns[1] * LEVER_STRUT_W)            # a point on the strut's upper edge
            B0 = (P[0] + n[0] * LEVER_ROOT / 2, P[1] + n[1] * LEVER_ROOT / 2)               # the blade's front edge from root to top corner
            B1 = (top.y + n[0] * LEVER_TIP / 2, top.z + n[1] * LEVER_TIP / 2)
            Lf = math.hypot(B1[0] - B0[0], B1[1] - B0[1]); df = ((B1[0] - B0[0]) / Lf, (B1[1] - B0[1]) / Lf)
            det = ds[0] * (-df[1]) - ds[1] * (-df[0])
            t = ((B0[0] - A0[0]) * (-df[1]) - (B0[1] - A0[1]) * (-df[0])) / det
            C = (A0[0] + ds[0] * t, A0[1] + ds[1] * t)                                      # corner
            u1, u2 = df, (-ds[0], -ds[1])                                                   # free directions: up the blade edge, back along the strut
            phi = math.acos(max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1]))) / 2
            bis = (u1[0] + u2[0], u1[1] + u2[1]); bn = math.hypot(*bis); bis = (bis[0] / bn, bis[1] / bn)
            tl = LEVER_CROTCH_R / math.tan(phi)
            T1 = (C[0] + u1[0] * tl - bis[0] * 0.3, C[1] + u1[1] * tl - bis[1] * 0.3)
            T2 = (C[0] + u2[0] * tl - bis[0] * 0.3, C[1] + u2[1] * tl - bis[1] * 0.3)
            Ci = (C[0] - bis[0] * 0.6, C[1] - bis[1] * 0.6)
            M = (C[0] + bis[0] * LEVER_CROTCH_R / math.sin(phi), C[1] + bis[1] * LEVER_CROTCH_R / math.sin(phi))
            fp = cq.Workplane('YZ').workplane(offset=x0).polyline([Ci, T1, T2]).close().extrude(x1 - x0).val()
            fp = fp.cut(_cyl((x0 - 1.0, M[0], M[1]), (1, 0, 0), LEVER_CROTCH_R, x1 - x0 + 2.0))
            o = o.fuse(fp)
        return o

    s = outline(x - STRUT / 2, x + STRUT / 2)
    if LEVER_RIB > 0:
        # (NOTES 59) outward rib on blade and strut, the same outline, x + STRUT/2 .. + LEVER_RIB, kept out of r LEVER_RIB_R0 round the pivot;
        # (attempt 3) its end is a 45 deg ramp (r LEVER_RIB_R0 - LEVER_RIB at the blade face, LEVER_RIB_R0 at the rib's face), not a step
        x0, x1 = x + STRUT / 2 - 0.01, x + STRUT / 2 + LEVER_RIB
        rib = outline(x0, x1)
        cone = cq.Solid.makeCone(LEVER_RIB_R0 - LEVER_RIB - 0.01, LEVER_RIB_R0 + 0.01, x1 - x0 + 0.02, V(x0 - 0.01, PIVOT_YZ[0], PIVOT_YZ[1]), V(1, 0, 0))
        rib = rib.cut(cone)
        s = s.fuse(rib)
    # (redesign round, D) inner pivot boss 2.4 mm longer toward the inside (to x 60.0, 1.5 mm from the servo's corner at x 58.5): with pinned
    # bores the arm read 14.6 MPa at this bore's edge
    s = s.fuse(_cyl((x - STRUT / 2 - 2.2, PIVOT_YZ[0], PIVOT_YZ[1]), (1, 0, 0), 9.0, 2.4))
    s = s.cut(_cyl((x - 14.0, PIVOT_YZ[0], PIVOT_YZ[1]), (1, 0, 0), 2.7, 26.0))                          # the pivot axle passes through
    s = s.fuse(_cyl((top.x, top.y, top.z), (-1, 0, 0), 4.2, top.x - k.LEVER_BALL_X - FORK[0] - 3.2))     # stud boss toward the ball, clear of the beam fork
    s = s.clean().cut(_cyl((top.x + 6.0, top.y, top.z), (-1, 0, 0), BORE_R, 20.0))                       # bore for the ball stud's shank
    return _mir(s, sx)


def stud_bore_seg(sx=1):
    """FEM only (environment FUGU_FEM_BORE_SEG=1, NOTES 58; ref frame): the stud bore again, 0.001 mm larger, as two cylinders that meet at the
    blade's mid plane (x = LEVER_X), cut after the arm's clean(): its wall then has a mouth half and a far half, so a case can put the stud's two
    bearing forces on them (the ball's 12.2 mm offset as a couple). The build never sets the switch."""
    top = V(*lever_top_w()) - OFF
    s = _cyl((top.x + 6.0, top.y, top.z), (-1, 0, 0), BORE_R + 0.001, 6.0).fuse(_cyl((top.x, top.y, top.z), (-1, 0, 0), BORE_R + 0.001, 14.0))
    return _mir(s, sx)


def lever_ball(sx=1):
    top = lever_top_w()
    c = V(k.LEVER_BALL_X, top[1], top[2])
    # FEM only (FUGU_FEM_STUD_BOND=1, NOTES 56): the shank at the bore's radius, so the stud bonds to the arm and the lever load can act on the
    # ball itself (12.2 mm inboard of the blade) instead of on the boss face; the build never sets it
    r = BORE_R if os.environ.get('FUGU_FEM_STUD_BOND') == '1' else PIN_R
    s = _ball(BALL_R, c).fuse(_cyl((c.x, c.y, c.z), (1, 0, 0), r, k.LEVER_X - k.LEVER_BALL_X + 3.0))
    return _mir(s, sx)


def cup_block(sx=1):
    """Round puck (POM or brass) on the lever ball: it slides in the flat slot of the beam's end fork, along the beam and along N, and is free
    to spin about M (the slot faces are square to M). Neck opening of 20 deg toward the lever."""
    b = V(*lever_ball_c()); m = V(*k.BEAM_M)
    s = cq.Solid.makeCylinder(PUCK_R, 2 * PUCK_H, b - m * PUCK_H, m)
    s = s.cut(_ball(BALL_R + 0.1, b)).cut(_cyl(b.toTuple(), (1, 0, 0), 3.2, PUCK_R + 1.0))
    return _mir(s, sx)


def lever_ball_c():
    top = lever_top_w()
    return (k.LEVER_BALL_X, top[1], top[2])


def _slot(pl, sx):
    x0, x1 = (-PUCK_R - 0.1, FORK[0] + 3.0) if sx > 0 else (-FORK[0] - 3.0, PUCK_R + 0.1)
    return _lbox(pl, x0, x1, -PUCK_H - 0.1, PUCK_H + 0.1, -FORK[2] + 1.3, FORK[2] - 1.3)


def beam():
    """Balance beam (one printed part): a hub on the yaw pin at C, a straight bar square to M and N, and an end fork on each side with a flat
    slot (faces square to M, open toward the lever) in which the puck of a lever ball slides: 2.4 mm along the beam, +-7.6 mm along N."""
    C = k.BEAM_C
    pc = _local(C)
    s = _ncyl(pc, HUB_R, -HUB_H, HUB_H)
    for sx in (1, -1):
        b = lever_ball_c(); b = (sx * b[0], b[1], b[2])
        pl = _local(b)
        x0, x1 = (-FORK[0], FORK[0] + 1.2) if sx > 0 else (-FORK[0] - 1.2, FORK[0])
        fork = _lbox(pl, x0, x1, -FORK[1], FORK[1], -FORK[2], FORK[2])
        bx = (HUB_R - 1.5, b[0] - FORK[0] + 0.5) if sx > 0 else (b[0] + FORK[0] - 0.5, -HUB_R + 1.5)
        bar = _lbox(pc, bx[0], bx[1], -BEAM_SIDE / 2, BEAM_SIDE / 2, -BEAM_SIDE / 2, BEAM_SIDE / 2)
        s = s.fuse(bar).fuse(fork).cut(_slot(pl, sx))
    s = s.clean().cut(_ncyl(pc, BORE_R, -HUB_H - 2.0, HUB_H + 2.0))
    assert s.isValid() and len(s.Solids()) == 1, 'balance beam is not one solid (%d)' % len(s.Solids())
    return s


YOKE_N = (HUB_H + 0.2, HUB_H + 4.2)      # the carrier's two lugs on the yaw pin, above and below the beam hub (along N)
CARRIER_HALF = k.SHOCK_EYE_W / 2 + 0.2 + CHEEK      # half thickness of the carrier (x): shock eye, side gaps, two cheeks


def beam_carrier():
    """Bellcrank carrier on the centre line (one printed part): it turns on a pin along x at PC in a clevis under the spine's front cross tube,
    holds the beam's yaw pin in a yoke at C (double shear) and the shock's front eye in a fork at S; its tail runs between the heave stop pads."""
    PC, C, S, T = k.CARRIER_PC, k.BEAM_C, k.SHOCK_S, k.CARRIER_TAIL
    h = CARRIER_HALF
    pc = _local(C)
    yoke = _fuse(_lbox(pc, -h, h, -6.0, 14.0, YOKE_N[0], YOKE_N[1]), _lbox(pc, -h, h, -6.0, 14.0, -YOKE_N[1], -YOKE_N[0]),
                 _lbox(pc, -h, h, 9.0, 14.0, -YOKE_N[1], YOKE_N[1]),                 # bridge behind the beam (the bar swings +-14.5 deg in front of it)
                 _lbox(pc, -k.STOP_YAW_X - 3.0, k.STOP_YAW_X + 3.0, BEAM_SIDE / 2 + k.STOP_YAW_GAP, 14.0, -4.0, 4.0))   # yaw stop pads at |x| 20: the bar's rear face meets them at +-14.5 deg
    top = pc.toWorldCoords((0.0, 11.5, YOKE_N[1] - 2.0))
    s = _fuse(yoke, _slab((PC[1], PC[2]), 6.0, (top.y, top.z), 4.5, -h, h), _slab((PC[1], PC[2]), 6.0, (S[1], S[2]), 5.5, -h, h),
              _slab((top.y, top.z), 4.5, (S[1], S[2]), 5.5, -h, h), _slab((PC[1], PC[2]), 6.0, (T[1], T[2]), TAIL_R, -h, h))
    s = s.cut(_ncyl(pc, BORE_R, -YOKE_N[1] - 1.0, YOKE_N[1] + 1.0))                  # yaw pin
    s = s.cut(_cyl((-h - 1.0, PC[1], PC[2]), (1, 0, 0), BORE_R, 2 * h + 2.0))        # carrier pin
    u = V(k.SHOCK_E[0] - S[0], k.SHOCK_E[1] - S[1], k.SHOCK_E[2] - S[2]).normalized()
    slot = cq.Solid.makeBox(k.SHOCK_EYE_W + 0.4, 30.0, 24.0, V(-k.SHOCK_EYE_W / 2 - 0.2, S[1] - 7.5, S[2] - 12.0))      # fork for the shock eye, open to the rear
    s = s.cut(slot).cut(_cyl((-h - 1.0, S[1], S[2]), (1, 0, 0), BORE_R, 2 * h + 2.0))
    s = max(s.Solids(), key=lambda q: q.Volume())
    assert s.isValid(), 'beam carrier is not valid'
    return s


def clevis(P, top_z, y0=-6.5, y1=6.5, z_low=6.5, stops=False):
    """Clevis hanging from a cage tube: two cheeks beside x = +-CARRIER_HALF (0.2 mm side gap) and a bar under the tube (fused into the cage).
    stops: the two heave stop pads between the cheeks, above and under the carrier's tail (bump and droop)."""
    g = CARRIER_HALF + 0.2
    s = None
    for x0 in (g, -g - CHEEK):
        p = _box(x0, x0 + CHEEK, P[1] + y0, P[1] + y1, P[2] - z_low, top_z)
        s = p if s is None else s.fuse(p)
    s = s.fuse(_box(-g - CHEEK, g + CHEEK, P[1] + y0, P[1] + y1, top_z - 3.0, top_z))
    if stops:
        t = k.CARRIER_TAIL
        s = s.fuse(_box(-g, g, t[1] - 3.0, t[1] + 3.0, t[2] + TAIL_R + k.STOP_BUMP_GAP, t[2] + TAIL_R + k.STOP_BUMP_GAP + 3.0))
        s = s.fuse(_box(-g, g, t[1] + 0.7, t[1] + 4.0, t[2] - TAIL_R - k.STOP_DROOP_GAP - 3.0, t[2] - TAIL_R - k.STOP_DROOP_GAP))   # behind the tail's lowest point: the carrier's shock arm swings to y -41.2 here at full bump
    return s.clean().cut(_cyl((-g - CHEEK - 1.0, P[1], P[2]), (1, 0, 0), BORE_R, 2 * (g + CHEEK) + 2.0))


def carrier_clevis(top_z):
    """Clevis of the beam carrier under the spine's front cross tube, with the bump and droop stop pads for the carrier's tail."""
    # stops=False: the two pads between the cheeks are out (they clashed with the redesigned tail boss); the heave stop becomes a steel dowel through
    # the tail on TPU saddles in pad blocks under the front cross tube (design by the beam helper, not built yet)
    return clevis(k.CARRIER_PC, top_z, y0=-6.5, y1=k.CARRIER_TAIL[1] - k.CARRIER_PC[1] + 4.0, z_low=TAIL_R + k.STOP_DROOP_GAP + 3.0, stops=False)


def shock_parts():
    """Heave shock (bought, ASSUMED 1/10 class: 68 mm eye to eye at ride height): body with the spring envelope at the rear, shaft toward the carrier."""
    a, b = V(*k.SHOCK_S), V(*k.SHOCK_E)
    u = (a - b).normalized()
    eye = lambda c: _cyl((c.x - k.SHOCK_EYE_W / 2, c.y, c.z), (1, 0, 0), 4.5, k.SHOCK_EYE_W).cut(_cyl((c.x - 5.0, c.y, c.z), (1, 0, 0), BORE_R, 10.0))
    body = _fuse(cq.Solid.makeCylinder(k.SHOCK_BODY_R, 34.0, b + u * 9.0, u), cq.Solid.makeCylinder(2.5, 6.0, b + u * 4.0, u), eye(b))
    shaft = _fuse(cq.Solid.makeCylinder(1.6, (a - b).Length - 43.0 - 4.0, a - u * 4.0, u * -1.0), eye(a))
    return body, shaft


DROOP_FACE = ((-113.00, 80.0), (-108.08, 90.0))      # droop stop contact face (y, z), CALCULATED by tools/arm_stops.py: the blade's front face at -32.9 mm
DROOP_X = (61.7, 70.7)


def droop_pad(sx=1):
    """Droop stop (TPU 95A wedge, glued on): on the rear face of the upper link's rail lug and its chamfered lower corner, over the lever
    blade's band. The blade's front face lands flat on it at -32.9 mm of wheel travel (74 N for an ASSUMED 30 N at the wheel)."""
    from parts import uplink as up
    yr = k.UPLINK_IN[1] + up.WIDTH / 2
    (y0, z0), (y1, z1) = DROOP_FACE
    pts = [(y0, z0), (y1, z1), (yr, z1), (yr, z0 + up.DROOP_CHAMFER[1]), (yr - up.DROOP_CHAMFER[0], z0)]
    s = cq.Workplane('YZ').workplane(offset=DROOP_X[0]).polyline(pts).close().extrude(DROOP_X[1] - DROOP_X[0]).val()
    return _mir(s, sx)


def pins():
    """Steel pins of the spring: carrier pin, yaw pin, the two shock eye pins."""
    h = CARRIER_HALF + 0.2 + CHEEK
    pc = _local(k.BEAM_C)
    return [('Carrier pin', _cyl((-h, k.CARRIER_PC[1], k.CARRIER_PC[2]), (1, 0, 0), PIN_R, 2 * h)),
            ('Beam yaw pin', _ncyl(pc, PIN_R, -YOKE_N[1], YOKE_N[1])),
            ('Shock pin front', _cyl((-CARRIER_HALF, k.SHOCK_S[1], k.SHOCK_S[2]), (1, 0, 0), PIN_R, 2 * CARRIER_HALF)),
            ('Shock pin rear', _cyl((-h, k.SHOCK_E[1], k.SHOCK_E[2]), (1, 0, 0), PIN_R, 2 * h))]


# ---------------------------------------------------------------------------------------------------------------------- carrier and knuckle
AXLE_YZ = k.TIP
from parts import v2_knuckle as _v2k
KP_BLOCK_Z = _v2k.BLOCK_Z         # carrier's kingpin block and web: z 31..50 for the PETG knuckle's 8 mm ears (was 27..54 with the steel stand-in)
EAR_Z = ((22.0, 26.0), (55.0, 59.0))     # knuckle ears under and over it (1 mm gaps for shim washers)
EAR_X = (117.0, 135.0)
SPINE_X = (131.0, 135.0)           # knuckle spine outboard of the block, inside the wheel barrel (bore r 21.13 from x 127)
ARM_Z = (21.0, 25.0)               # steering arm plate
HEX_X = (148.3, 157.5)             # 12 mm wheel hex hub in the wheel's pocket (bought)


def carrier(sx=1):
    """Wheel carrier (printed): a lug on the arm's tip pin, a tower up to the upper link's clevis and a web out through the tyre bore to the
    kingpin block inside the wheel barrel. It rides on the arm tip without turning (arm and upper link are a parallelogram).
    Built to CARRIER_ENVELOPE of tools/steer_kin.py (at least 2.14 mm to the steered wheel)."""
    from parts import uplink as up
    kx, ky = k.KINGPIN
    ay, az = AXLE_YZ
    ho = V(*k.UPLINK_OUT_B)
    s = _fuse(_box(93.6, 104.0, ay - 7.45, ay + 7.45, 33.0, 48.0),                                  # tip lug
              _box(94.8, 107.2, ho.y - 6.0, ho.y + 6.0, 44.0, ho.z - 11.0),                         # tower
              up.clevis_u(ho - OFF, 1).moved(cq.Location(OFF)),                                     # clevis for the upper link's outer end
              _box(104.0, kx, ky - 7.0, ky + 7.0, KP_BLOCK_Z[0], KP_BLOCK_Z[1]),                    # web
              _cyl((kx, ky, KP_BLOCK_Z[0]), (0, 0, 1), 7.0, KP_BLOCK_Z[1] - KP_BLOCK_Z[0]))         # kingpin block
    s = s.cut(_cyl((90.0, ay, az), (1, 0, 0), 2.15, 21.5)).cut(_cyl((kx, ky, 0.0), (0, 0, 1), 2.05, 100.0))
    assert s.isValid() and len(s.Solids()) == 1, 'carrier is not one solid (%d)' % len(s.Solids())
    return _mir(s, sx)


def knuckle(sx=1):
    """Knuckle: the printed PETG knuckle of parts/v2_knuckle.py (owner: "Printed + servo", NOTES decision 26)."""
    return _mir(_v2k.knuckle(), sx)


def knuckle_steel_standin(sx=1):
    """The former STAND-IN (steel sizes, KNUCKLE_ENVELOPE of tools/steer_kin.py): kept for the K-STEEL option; not built."""
    kx, ky = k.KINGPIN
    ay, az = AXLE_YZ
    kb = k.KNUCKLE_BALL_B
    parts = [_box(SPINE_X[0], SPINE_X[1], ky - 6.0, ky + 6.0, EAR_Z[0][0], EAR_Z[1][1]), _cyl((SPINE_X[0], ay, az), (1, 0, 0), 8.0, k.WHEEL_DISC_X - SPINE_X[0])]
    for z0, z1 in EAR_Z:
        parts.append(_box(EAR_X[0], EAR_X[1], ky - 6.0, ky + 6.0, z0, z1))
    parts.append(_plate((kx, ky), 4.0, (kb[0], kb[1]), 4.0, ARM_Z[0], ARM_Z[1]))
    s = _fuse(*parts)
    s = s.cut(_cyl((kx, ky, 0.0), (0, 0, 1), 2.05, 100.0)).cut(_cyl((SPINE_X[0] - 1.0, ay, az), (1, 0, 0), 2.05, 40.0))
    s = s.cut(_cyl((kb[0], kb[1], 10.0), (0, 0, 1), BORE_R, 20.0))
    assert s.isValid() and len(s.Solids()) == 1, 'knuckle is not one solid (%d)' % len(s.Solids())
    return _mir(s, sx)


def knuckle_hardware(sx=1):
    """Kingpin, axle bolt, nut and sleeve, the wheel's flanged bronze bushing (was the 12 mm hex hub) and the tie rod's ball stud under the knuckle arm (all bought, steel)."""
    kx, ky = k.KINGPIN
    ay, az = AXLE_YZ
    kb = k.KNUCKLE_BALL_B
    from parts.wheels import FRONT_BUSH as FB, FRONT_BUSH_PART as BP
    x0 = _v2k.I.KN_HUB_FACE_X                                  # flange on the knuckle's seat ring, flush with the wheel hub's inner face
    bush = _cyl((x0, ay, az), (1, 0, 0), BP[3] / 2, BP[4]).fuse(_cyl((x0, ay, az), (1, 0, 0), FB[0] / 2, BP[4] + BP[2]))   # body modelled pressed (D seat)
    bush = bush.cut(_cyl((x0 - 1.0, ay, az), (1, 0, 0), BP[0] / 2, BP[4] + BP[2] + 2.0))
    stud = _ball(BALL_R, V(*kb)).fuse(_cyl(kb, (0, 0, 1), 2.0, _v2k.ARM_Z[1] - 1.0 - kb[2]))      # M4 stud into the insert in the arm's ball boss
    out = [('Kingpin', _cyl((kx, ky, k.KINGPIN_Z[0]), (0, 0, 1), 2.0, k.KINGPIN_Z[1] - k.KINGPIN_Z[0])),
           ('Front wheel bushing', bush), ('Knuckle ball stud', stud)] + _v2k.hardware()
    return [(n, _mir(s, sx)) for n, s in out]


# ---------------------------------------------------------------------------------------------------------------------------------- steering
TIE_HOUSING, DRAG_HOUSING = (5.0, 5.0), (3.75, 4.0)      # rod end housings (r, height): M4 tie rod ends, M3 drag link ends (bought)
TIE_HOUSING_OUT = (4.1, 5.0)       # the tie rod's knuckle end needs a slim rod end (housing OD 8.2 at most; 8.4 left 0.99 mm at the one-wheel corner): with r 5 it passed 0.25 mm from the
                                   # tyre's inner face (x 113) at full lock with lean and bump; the ball itself stays (bump steer depends on it)
ROCKER_HALF_W = 3.5
MOUNT_Y = (-120.8, -112.8)
STRAP_X = (85.5, 88.5)             # strap outboard of the rocker hub (hub edge x 84.5; was 91 .. 94 with the pin at x 85.5)
STRAP_Y = (-120.8, -103.8)         # not ahead of y -120.8: the arm's neck comes down there at droop (0.47 mm with a strap from -124.8)
REAR_WALL_Y = (-106.8, -103.8)     # wall behind the arm's outer pivot boss (r 9, rear edge y -107.8), from the foot up to the hanger's full height
SHOE_Z = (16.5, 18.7)              # shoe under the hub (hub underside z 19.0): the lowest point of the front end at lean
FOOT_Z = (34.0, 37.5)              # foot over the rocker, 1 mm under the arm's outer pivot boss (underside z 38.5)


def _servo_box(grow=0.0):
    """Worst-case servo of the class (k.SERVO_ENV): case plus both mounting ears (8.0 mm overhang, at 18.4 mm from the case base)."""
    w, l, h = k.SERVO_BOX
    e = k.SERVO_ENV
    y0 = k.SERVO_SHAFT_B[0] - k.SERVO_SHAFT_IN
    z0 = k.SERVO_SHAFT_B[1] - h / 2
    case = cq.Solid.makeBox(w + 2 * grow, l + 2 * grow, h + grow, V(k.SERVO_X[0] - grow, y0 - grow, z0 - grow))
    tx0, tx1 = k.SERVO_TAB_X
    ears = cq.Solid.makeBox(tx1 - tx0 + 2 * grow, l + 2 * e['ear_overhang'] + 2 * grow, h + grow, V(tx0 - grow, y0 - e['ear_overhang'] - grow, z0 - grow))
    return case.fuse(ears).clean()


def servo(sx=1):
    """Steering servo stand-in (bought): box lying flat on the deck behind the arm pivot axle, output shaft along x toward the outside."""
    s = _servo_box().fuse(_cyl((k.SERVO_X[1], k.SERVO_SHAFT_B[0], k.SERVO_SHAFT_B[1]), (1, 0, 0), 3.0, 0.7)).clean()
    return _mir(s, sx)


def horn(sx=1):
    """Servo horn (bought, metal, 25T, 12 mm hole radius): plate from the output shaft down to the horn ball, ball on a 7 mm stud toward the outside."""
    x0, t = k.SERVO_X[1] + 0.7, 2.3
    hx, hy, hz = k.HORN_BALL_B
    s = _slab(k.SERVO_SHAFT_B, 5.0, (hy, hz), 3.5, x0, x0 + t)
    s = s.fuse(_cyl((x0 + t, hy, hz), (1, 0, 0), PIN_R, hx - x0 - t)).clean().fuse(_ball(BALL_R, V(hx, hy, hz)))
    return _mir(s, sx)


def _rod(a, b, r, hs, cup_a, cup_b, neck_a, neck_b, cone=27.0, hs_b=None, open_r=(1.9, 1.9)):
    """Rod with a ball end housing (r, height) on each end: housing axes cup_a / cup_b, the stud's neck leaves each housing along neck_a / neck_b
    through a cone of `cone` deg."""
    A, B = V(*a), V(*b)
    u = (B - A).normalized()
    hb = hs_b or hs
    s = cq.Solid.makeCylinder(r, (B - A).Length - (hs[0] - 1.0) - (hb[0] - 1.0), A + u * (hs[0] - 1.0), u)
    for c, ax, h_ in ((A, cup_a, hs), (B, cup_b, hb)):
        ax = V(*ax)
        s = s.fuse(cq.Solid.makeCylinder(h_[0], h_[1], c - ax * (h_[1] / 2), ax))
    s = s.clean()
    for c, n, o in ((A, neck_a, open_r[0]), (B, neck_b, open_r[1])):
        n = V(*n)
        s = s.cut(_ball(BALL_R + 0.1, c)).cut(cq.Solid.makeCone(o, o + hs[1] * math.tan(math.radians(cone)), hs[1], c, n))
    return s


def tie_rod(sx=1):
    """Tie rod (bought): M4 threaded rod with a rod end on each side (cone >= 27 deg), parallel to the arm and as long as it. Both balls hang under
    their plates: the inner one under the rocker's output arm, the outer one under the knuckle arm."""
    s = _rod(k.ROD_INNER_BALL, k.KNUCKLE_BALL_B, ROD_R, TIE_HOUSING, (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), hs_b=TIE_HOUSING_OUT,
             open_r=(1.9, 2.4))      # knuckle end: the M4 ball stud's 4 mm neck (+0.4); the bought rod end must pass it at 27 deg: MEASURE ON ARRIVAL
    assert s.isValid() and len(s.Solids()) == 1, 'tie rod is not one solid'
    return _mir(s, sx)


def drag_link(sx=1):
    """Drag link (bought M3 rod ends, cone >= 25 deg, on a short rod): from the horn ball (stud along x) to the rocker's input ball (hanging)."""
    s = _rod(k.HORN_BALL_B, k.ROCKER_IN_BALL, 1.5, DRAG_HOUSING, (1, 0, 0), (0, 0, 1), (-1, 0, 0), (0, 0, 1), cone=25.0)
    assert s.isValid() and len(s.Solids()) == 1, 'drag link is not one solid'
    return _mir(s, sx)


def rocker(sx=1):
    """Rocker (printed) on the vertical pin under the arm root: a hub, the output arm (same vector as the knuckle arm) and above it the input arm
    (20 deg rearward of straight inboard). Both ball studs hang under their plates."""
    px, py = k.ROCKER_PIN
    ro, ri = k.ROD_INNER_BALL, k.ROCKER_IN_BALL
    s = _fuse(_cyl((px, py, k.ROCKER_HUB_Z[0]), (0, 0, 1), 4.5, k.ROCKER_HUB_Z[1] - k.ROCKER_HUB_Z[0]),
              _plate((px, py), 4.5, (ro[0], ro[1]), ROCKER_HALF_W, *k.ROCKER_PLATES_Z[0]), _plate((px, py), 4.5, (ri[0], ri[1]), ROCKER_HALF_W, *k.ROCKER_PLATES_Z[1]))
    s = s.cut(_cyl((px, py, 0.0), (0, 0, 1), 2.05, 60.0))
    for b, pz in ((ro, k.ROCKER_PLATES_Z[0]), (ri, k.ROCKER_PLATES_Z[1])):
        s = s.cut(_cyl((b[0], b[1], pz[0] - 1.0), (0, 0, 1), BORE_R, pz[1] - pz[0] + 2.0))
    assert s.isValid() and len(s.Solids()) == 1, 'rocker is not one solid'
    return _mir(s, sx)


def rocker_hardware(sx=1):
    px, py = k.ROCKER_PIN
    ro, ri = k.ROD_INNER_BALL, k.ROCKER_IN_BALL
    stud = lambda b, pz: _ball(BALL_R, V(*b)).fuse(_cyl(b, (0, 0, 1), PIN_R, pz[1] - b[2]))
    from parts import swap_interface as I
    out = [('Rocker pin', _cyl((px, py, SHOE_Z[0] + I.RM_SHOE_FLOOR), (0, 0, 1), I.RM_PIN[0] / 2, I.RM_PIN[1])),    # ISO 8734 D4 x 24
           ('Rocker ball stud out', stud(ro, k.ROCKER_PLATES_Z[0])), ('Rocker ball stud in', stud(ri, k.ROCKER_PLATES_Z[1]))]
    return [(n, _mir(s, sx)) for n, s in out]


def rocker_bracket(sx=1):
    """Rocker mount (printed): a split clamp round the fixed pivot axle between the arm's two plates, a hanger down from it and a foot over the
    rocker that holds the 4 mm rocker pin from above (the pin hangs down, nothing reaches under the rocker). Built to ROCKER_MOUNT of
    tools/steer_kin.py. Clamp screw and the pin's retention are not drawn."""
    px, py = k.ROCKER_PIN
    ay, az = k.PIVOT
    x0, x1, x2 = k.ROCKER_MOUNT_X
    s = _fuse(_box(x0, x1, MOUNT_Y[0], MOUNT_Y[1], FOOT_Z[0], az + 6.0), _box(x1, x2, MOUNT_Y[0], MOUNT_Y[1], FOOT_Z[0], FOOT_Z[1]),
              # (B1, second attempt) foot, strap and shoe alone all hung on the 3.5 mm foot root and changed nothing (157 MPa). Now the hanger is
              # 9 mm longer to the rear and a 3 mm wall runs behind the arm's outer boss from the hanger to the strap, 19.5 mm high: it takes the
              # pin's moment about x that the thin foot could not
              _box(x0, x1, MOUNT_Y[1], REAR_WALL_Y[1], FOOT_Z[0], az + 6.0),
              _box(x1, STRAP_X[1], REAR_WALL_Y[0], REAR_WALL_Y[1], FOOT_Z[0], az + 6.0),
              # (redesign round, B1) second support under the rocker, so the pin is held at both ends: the foot widens outboard of the rocker hub,
              # a strap 3 x 16 mm runs down outboard of the hub (the rocker's arms swing inboard), a shoe under the hub holds the pin's lower end.
              # With the foot alone the pin was a cantilever: 157 MPa in the foot at the assumed servo stall.
              _box(STRAP_X[0] - 4.0, STRAP_X[1], STRAP_Y[0], STRAP_Y[1], FOOT_Z[0], FOOT_Z[1]),
              _box(STRAP_X[0], STRAP_X[1], STRAP_Y[0], STRAP_Y[1], SHOE_Z[0], FOOT_Z[1]),
              _box(px - 5.5, STRAP_X[1], STRAP_Y[0], STRAP_Y[1], SHOE_Z[0], SHOE_Z[1]))
    from parts import swap_interface as I   # every interface feature (shared with the metal bracket B2)
    s = s.cut(_cyl((60.0, ay, az), (1, 0, 0), I.RM_AXLE[1] / 2, 40.0))
    floor = SHOE_Z[0] + I.RM_SHOE_FLOOR
    s = s.cut(_cyl((px, py, floor), (0, 0, 1), I.RM_PIN_BORE / 2, I.RM_PIN[1]))           # pin bore, blind at both ends: from the shoe's floor up into the hanger's full section (NOTES 42)
    # clamp: slit in the plane of the axle, from the axle to the rear face (the C opens rearward, the hinge is the wall ahead of the axle),
    # one M3 from the top through the upper jaw into an insert in the hanger (was a slit above the axle with no screw: never drawn)
    w, ln = I.RM_SLIT
    s = s.cut(_box(x0 - 1.0, STRAP_X[1] + 1.0, ay, REAR_WALL_Y[1] + 1.0, az - w / 2, az + w / 2))
    cx, cy = I.RM_CLAMP_SCREW[1]
    s = s.cut(_cyl((cx, cy, az), (0, 0, 1), I.RM_CLAMP_HOLE / 2, 20.0))
    s = s.cut(_cyl((cx, cy, az - w / 2 - I.RM_CLAMP_INSERT[1]), (0, 0, 1), I.RM_CLAMP_INSERT[0] / 2, I.RM_CLAMP_INSERT[1] + 0.1))
    assert s.isValid() and len(s.Solids()) == 1, 'rocker bracket is not one solid'
    return _mir(s, sx)


def parts():
    """[(name, shape, group, colour)] of the parts that are registered on their own (world frame), both sides."""
    out = []      # beam, beam carrier, pucks, shock, stops and servo holder: parts/swingarm_guide.py (final guide design, round 3)
    for sx in (1, -1):
        out += [('Lever ball stud %d' % sx, lever_ball(sx), G, STEEL),
                ('Droop stop pad %d' % sx, droop_pad(sx), G, (0.15, 0.15, 0.16)),
                ('Front carrier %d' % sx, carrier(sx), G, PRINT), ('Front knuckle %d' % sx, knuckle(sx), G, PRINT),
                ('Steer servo %d' % sx, servo(sx), G, SERVO_BLACK), ('Steer horn %d' % sx, horn(sx), G, STEEL),
                ('Drag link %d' % sx, drag_link(sx), G, STEEL), ('Rocker %d' % sx, rocker(sx), G, PRINT),
                ('Rocker bracket %d' % sx, rocker_bracket(sx), G, PRINT), ('Tie rod %d' % sx, tie_rod(sx), G, STEEL)]
        out += [('%s %d' % (n, sx), s, G, STEEL) for n, s in knuckle_hardware(sx) + rocker_hardware(sx)]
    return out
