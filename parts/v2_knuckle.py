"""Front knuckle, printed PETG (owner, November 24: "Printed + servo"; option K-PETG = option B of the knuckle study).

Ported from the knuckle worker's sc-trike-v2_knuckle/parts/v2_knuckle.py (read there, not edited) into this tree; the study and its FEM are in
that worker's docs/v2/knuckle_options.md. +x side, world mm; parts/swingarm_front.py mirrors it for -x.
Hard points are this tree's (parts/swingarm2.py): kingpin vertical through KINGPIN, bearing faces z 22 / 59, knuckle ball KNUCKLE_BALL_B.

Layout (the study's option B): an outer C round the carrier's kingpin block, 8 mm ears (z 22..30 and 51..59: the carrier block is shortened to
z 31..50), a deep spine inside the rim barrel (x 131..143, y +-9, clipped to r 20.03 about the axle), the axle boss r 8 to x 147 and a seat ring
r 6 to the hub face x 148, the steering arm 12 mm wide and 9 mm deep (z 21..30) with a ball boss r 6.
Steel: D4 kingpin; a 6 x 1 steel tube pressed into the axle bore (x 134.4..148); an M4 12.9 axle bolt from outboard into an M4 nut in a hex
pocket in the spine's inboard face. Changed against the study: the tie rod ball stud is M4 (the study's M3 stud tapped into PETG failed at the
steering stall), screwed from below into an M4 heat-set insert pressed in from the top of the ball boss.
"""
import math
import cadquery as cq
from parts import swingarm2 as k
from parts import swap_interface as I        # every interface feature (shared with the metal knuckle)

KX, Y0 = I.KN_KINGPIN_AXIS
Z0 = I.KN_AXLE_AXIS[1]
BALL = I.KN_BALL_AXIS + (I.KN_BALL_Z,)

GAP = I.KN_BLOCK_GAP
EAR_LO, EAR_HI = I.KN_EAR_LO, I.KN_EAR_HI
BLOCK_Z = I.KN_BLOCK_Z
EAR_W = 7.5                         # ear half width (y)
BOSS_R = 7.5                        # kingpin boss radius
SPINE_X = I.KN_ENVELOPE['spine_x']   # spine (x 131 = carrier block r 7 + 1 mm)
SPINE_W = 9.0                       # spine half width before clipping
BODY_R = I.KN_ENVELOPE['spine_r_max']   # 20.03: clip radius about the axle inside the rim bore
AXLE_BOSS = (8.0, 143.0, 147.0)     # r, x0, x1
SEAT = (I.KN_SEAT_R, I.KN_HUB_FACE_X - 1.0, I.KN_HUB_FACE_X)          # seat ring on the hub face (running face, carries the side force)
ARM_Z = I.KN_ARM_Z                  # steering arm plate
ARM_BALL_R = 6.0                    # boss round the ball stud
THRUST = I.KN_THRUST                 # raised thrust-washer seats: r, height (0.5 mm washer between seat and carrier block)
KINGPIN_D = I.KN_KINGPIN_BORE
AXLE_D = I.KN_AXLE_BORE
SLEEVE_D = I.KN_SLEEVE[0]                      # 6 x 1 steel tube, pressed in (the study: without it the bore lip reached safety 1.29)
NUT_AF, NUT_T = I.KN_NUT_POCKET[:2]   # M4 nut pocket (7.0 AF + 0.2), depth
STUD_INSERT_D = I.KN_STUD_INSERT[0]   # M4 heat-set insert pocket, through the ball boss from the top
STUD_INSERT_DEPTH = ARM_Z[1] - ARM_Z[0]


def _xcyl(r, x0, x1, y=Y0, z=Z0):
    return cq.Workplane("YZ").workplane(offset=x0).center(y, z).circle(r).extrude(x1 - x0)


def _zcyl(r, z0, z1, x=KX, y=Y0):
    return cq.Workplane("XY").workplane(offset=z0).center(x, y).circle(r).extrude(z1 - z0)


def _box(x0, y0, z0, x1, y1, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def _arm_outline():
    """Plan outline of the steering arm: hull of the kingpin boss circle and the ball boss circle."""
    c1, r1 = (KX, Y0), BOSS_R
    c2, r2 = BALL[:2], ARM_BALL_R
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dy); u = (dx / d, dy / d); n = (-u[1], u[0])
    ca = (r1 - r2) / d; sa = math.sqrt(1 - ca * ca)
    pts = []
    for s in (1, -1):
        e = (ca * u[0] + s * sa * n[0], ca * u[1] + s * sa * n[1])
        pts.append(((c1[0] + r1 * e[0], c1[1] + r1 * e[1]), (c2[0] + r2 * e[0], c2[1] + r2 * e[1])))
    return [pts[0][0], pts[0][1], pts[1][1], pts[1][0]]


def knuckle():
    """The printed PETG knuckle (one solid, +x side)."""
    spine = _box(SPINE_X[0], Y0 - SPINE_W, EAR_LO[0] - 1.0, SPINE_X[1], Y0 + SPINE_W, EAR_HI[1] + 1.0).intersect(_xcyl(BODY_R, SPINE_X[0], SPINE_X[1]))
    body = spine.union(_xcyl(AXLE_BOSS[0], AXLE_BOSS[1] - 0.5, AXLE_BOSS[2]))
    for z0, z1 in (EAR_LO, EAR_HI):
        body = body.union(_zcyl(BOSS_R, z0, z1).union(_box(KX, Y0 - EAR_W, z0, SPINE_X[0] + 0.5, Y0 + EAR_W, z1)))
    arm = cq.Workplane("XY").workplane(offset=ARM_Z[0]).polyline(_arm_outline()).close().extrude(ARM_Z[1] - ARM_Z[0])
    body = body.union(arm.union(_zcyl(ARM_BALL_R, ARM_Z[0], ARM_Z[1], x=BALL[0], y=BALL[1])))
    body = body.union(_zcyl(THRUST[0], EAR_LO[1], EAR_LO[1] + THRUST[1])).union(_zcyl(THRUST[0], EAR_HI[0] - THRUST[1], EAR_HI[0]))
    body = body.union(_xcyl(SEAT[0], SEAT[1] - 0.5, SEAT[2]))
    body = body.cut(_zcyl(KINGPIN_D / 2, ARM_Z[0] - 1, EAR_HI[1] + 1))
    body = body.cut(_zcyl(STUD_INSERT_D / 2, ARM_Z[0] - 1, ARM_Z[1] + 1, x=BALL[0], y=BALL[1]))
    body = body.cut(_xcyl(AXLE_D / 2, SPINE_X[0] - 1, SEAT[2] + 1))
    body = body.cut(_xcyl(SLEEVE_D / 2, SPINE_X[0] + NUT_T, SEAT[2] + 1))
    nut = cq.Workplane("YZ").workplane(offset=SPINE_X[0] - 0.01).center(Y0, Z0).polygon(6, NUT_AF / math.cos(math.pi / 6)).extrude(NUT_T + 0.01)
    s = body.cut(nut).val()
    assert s.isValid() and len(s.Solids()) == 1, 'PETG knuckle is not one solid'
    return s


def hardware():
    """Steel of the knuckle (+x side): axle bolt M4 x 35 (12.9, head outboard of the wheel), its nut in the pocket, the 6 x 1 sleeve."""
    x_out = k.WHEEL_OUT_X + 0.4
    bolt = _xcyl(2.0, SPINE_X[0] + 0.5, x_out).union(_xcyl(3.5, x_out, x_out + 4.0)).val()
    nut = cq.Workplane("YZ").workplane(offset=SPINE_X[0] + 0.2).center(Y0, Z0).polygon(6, 7.0 / math.cos(math.pi / 6)).extrude(3.2) \
        .cut(_xcyl(2.0, SPINE_X[0], SPINE_X[0] + 4.0)).val()
    sleeve = _xcyl(SLEEVE_D / 2, SPINE_X[0] + NUT_T, SEAT[2]).cut(_xcyl(2.0, SPINE_X[0], SEAT[2] + 1)).val()
    return [('Front axle bolt', bolt), ('Front axle nut', nut), ('Axle sleeve', sleeve)]
