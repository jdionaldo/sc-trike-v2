"""Front suspension and steering after the lean-steer build in the reference video (inputs/..., 6:20-8:30): a double-wishbone parallelogram on each
side of one narrow head that stands on the front deck on the centre line, BEHIND the skid.

World frame of the model (mm, Z up, front = -Y, ground z = -15.6). Shapes are built for the +x side and mirrored for -x.

- Head: one printed tube frame (parts/tubeframe.py): a front and a rear centre post with a cross bar at each pivot height, base tubes on the deck,
  a top and a bottom tie. Four pivot rods along Y at x = +-XI. Pivots near the centre line keep the arms long: 10 deg of body lean is about 15 deg
  at the arms (the earlier head with pivots at x +-50 needed 23 deg).
- Arms: chamfered square tube A-arms, both the same length, pivot rods as far apart as the ball joints, so the knuckle stays parallel to the head and
  the wheels lean with the body.
- Ball joints: bought 6.8 mm ball studs and ball cups (1/8 scale RC). The arm studs lie along Y in the knuckle: lean turns the cup about the stud
  axis (free), steering tilts it (about 20 deg, inside a ball cup's range), the wheel load is radial on the cup (its strong direction).
- Heave spring: one horizontal coilover between two truss posts standing on the UPPER arms. Lean moves both posts the same way and leaves it alone,
  a bump moves them toward each other and compresses it. The upper arm therefore carries the wheel load (bending), the lower arm only push and pull.
- Steering: servo standing in its mount on the deck behind the head, shaft up, T horn on a tall hub with the ball studs hanging down, two tie rods (ball cups, studs vertical) to the steering arms of the knuckles. The
  horn balls sit near the pivot axes and between the arm planes, so the tie rods move with the arms (little bump steer).
"""
import math
import cadquery as cq

V = cq.Vector

# ---- wheel and ground (from the model) ----
KY = -216.55          # wheel centre y
ZC = 40.6             # wheel centre z
GROUND_Z = -15.6
WHEEL_IN_X = 113.0    # inner face of the tyre: above and below the rim (radius 24) the knuckle must stay inboard of this
# ---- deck under the head (measured on the built model: top z 18.3 at y -205, in line with the rear deck, rising 0.07 per mm to the rear) ----
DECK_Z0, DECK_SLOPE, DECK_HALF = 18.3, 0.07, 54.5
deck_top = lambda y: DECK_Z0 + DECK_SLOPE * (y + 205.0)
# ---- pivots and ball joints ----
ZL, ZU = 42.0, 70.0   # lower / upper arm plane: the lower arm clears the deck edge when it drops 18 deg (lean + droop)
KX = 106.0            # kingpin x (both ball joints), inboard of the tyre
XI = 15.0             # pivot rod x (the arms' square pivot barrels, corner radius 8.4, clear the head posts, half width 5.7, at every arm angle)
ARM_L = KX - XI       # pivot axis to ball centre, both arms
HEAD_D = 13.0         # head tube, nominal (chamfered square, side 0.88 x this); FEM: 10 gave safety 1.03 at the front post / upper cross bar
ARM_D_UP, ARM_D_LO = 16.0, 14.0   # the upper arm carries the heave shock posts (bending), the lower arm only push and pull (FEM: 12 / 10 were below safety 2)
POST_D = 18.0         # shock post tubes on the upper arm: the fork cheeks beside the 3.5 mm gap are 6.2 mm each (FEM: 4.4 mm cheeks were at safety 0.83)
TS = HEAD_D * 0.88
HY0 = -201.0 + TS / 2 # front post centre y: its front face is 0.6 mm behind the skid (y -201.6)
BOSS_L, BOSS_R = 8.0, 6.5
STUB_X = 24.0         # each arm leg leaves the pivot barrel square (a stub along x to here, clear of the head's bosses at x <= 22), then turns to the apex
YF = HY0 + TS / 2 + 0.4 + BOSS_L / 2      # front leg pivot y
YR = YF + 16.0                             # rear leg pivot y
HY1 = YR + BOSS_L / 2 + 0.4 + TS / 2       # rear post centre y
ROD_D = 6.0
APEX_BACK = 9.0       # arm end face to ball centre along x
HEAD_TOP = 82.0       # top face of the head (the heave shock passes above it)
FOOT_X = (22.0, 34.0)  # half length of the front / rear base tube: the lower arm's front leg passes over the front frame outboard of x 23.7 and drops there
HEAD_SCREWS = ((16.0, HY0), (28.0, HY1))   # M3 from below through the deck into inserts in the base tubes (x, y), +x side
# ---- ball joints (bought): 6.8 mm ball stud, M3, with a ball cup ----
BALL_R, CUP_OD, CUP_T = 3.4, 10.5, 6.0
NECK_R, NECK_L, FLANGE_R, FLANGE_T = 1.6, 3.5, 3.5, 2.0
TAB_Y0 = KY + CUP_T / 2 + NECK_L + FLANGE_T      # knuckle plate: front face (the arm studs screw into it from the front)
TAB_T = 10.0          # FEM: a 5 mm plate was at safety 0.58 around the upper stud (the stud pries 11 mm ahead of it)
# ---- heave shock on the upper arms ----
POST_X = (34.0, 70.0)           # feet of the truss post on the upper arm: an upright under the eye and a diagonal from further out, so nothing
                                # of the post lies inboard under the shock body; the eye at x 34 stays inboard of the cage's front tubes at x 45
SHOCK_H = 24.0                  # eye above the upper arm plane
FORK_GAP = 3.5                  # gap between the fork cheeks at the post eye
SHOCK_R = 8.0
# ---- steering ----
ZT = 60.0                                  # tie rod axis height (between the arm planes)
STEER_ARM_XY = (101.0, -146.0)             # tie rod ball on the knuckle (x, y), +x side
HORN_BALL = (8.0, -146.0)                  # horn ball of the +x tie rod at rest
HORN_L = 37.0
SERVO_SHAFT_Y = HORN_BALL[1] + HORN_L      # the servo lies on the deck behind the head, shaft up, horn pointing forward
SERVO_BOX = (20.2, 40.8, 26.5)             # x, y, z of the low-profile servo (AGFRC B44BLS class) standing on its base, output shaft up
SERVO_EAR = (7.0, 2.5, 17.5)               # mounting ears at both ends: length along y, thickness, underside above the servo's base
SERVO_SHAFT_IN = 10.2                      # output shaft from the servo's front end
SERVO_Z0 = 28.2                            # servo underside (above the deck under it; the cradle fills the wedge)
HORN_Z0, HORN_T = ZT + CUP_T / 2 + NECK_L + FLANGE_T, 2.5      # the horn's arm lies above the tie rods on a tall hub; the ball studs hang down from it                # horn plate
ARM_PLATE_Z0, ARM_PLATE_T = ZT - CUP_T / 2 - NECK_L - FLANGE_T - 4.5, 4.5      # steering arm plate on the knuckle (under the tie rod cup)


def cyl(p0, p1, r):
    d = V(*p1) - V(*p0)
    return cq.Solid.makeCylinder(r, d.Length, V(*p0), d.normalized())


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def _fuse(*shapes):
    out = shapes[0]
    for s in shapes[1:]:
        out = out.fuse(s)
    out = out.clean()
    assert out.isValid(), 'fused shape is not valid'
    return out


def _stub_y(d, rear):
    """y of a leg stub's centre line: its outer flank lies 1.2 mm inside the barrel's end (the turning arm's bend joint then clears the head's bar ends), so nothing of the joint has to be trimmed."""
    off = -BOSS_L / 2 + 1.2 + d * 0.44
    return YR - off if rear else YF + off


def _leg_y(z_unused, x, rear, d=None):
    """y of an arm leg's centre line at x >= STUB_X (from the stub's end to the apex node)."""
    y0 = _stub_y(ARM_D_UP if d is None else d, rear)
    return y0 + (x - STUB_X) / (KX - 17.0 - STUB_X) * (KY - y0)


_PM = [((_leg_y(0, x, False) + _leg_y(0, x, True)) / 2) for x in POST_X]
SHOCK_EYE = (POST_X[0], round(_PM[0], 1), ZU + SHOCK_H)      # +x post eye (x, y, z)


def a_arm(z, upper):
    """+x A-arm at height z in chamfered square tube: two legs from the pivot bosses to one apex node, the apex boss for the ball cup's shank,
    bores for the pivot rod. The upper arm also carries the truss post of the heave shock: two braces between the legs, two tubes up to the eye."""
    from parts.tubeframe import Frame
    d = ARM_D_UP if upper else ARM_D_LO
    ax = KX - APEX_BACK                     # arm end face x
    ap = (KX - 17.0, KY, z)                 # apex node (inside the apex boss)
    f = Frame()
    # pivot barrel: one square tube along the pivot axis between the head's two frames; each leg leaves it as a square stub and turns to the apex at STUB_X, so
    # nothing has to be trimmed afterwards (the earlier round bosses with trimmed leg ends left slivers and a notch: FEM safety 0.87 there)
    f.tube((XI, YF - BOSS_L / 2, z), (XI, YR + BOSS_L / 2, z), d)
    for rear in (False, True):
        ys = round(_stub_y(d, rear), 1)
        f.tube((XI, ys, z), (STUB_X, ys, z), d); f.tube((STUB_X, ys, z), ap, d)
    f.add(cyl((ax - 14.0, KY, z), (ax, KY, z), 6.0))   # apex boss for the cup shank
    if upper:
        e = SHOCK_EYE; pw = POST_D * 0.88
        for x, ym in zip(POST_X, _PM):
            f.tube((x, round(_leg_y(z, x, False), 1), z), (x, round(_leg_y(z, x, True), 1), z), HEAD_D)      # brace between the legs
            f.tube((x, ym, z), (e[0], e[1], e[2] + 7.0) if x == POST_X[0] else e, POST_D)   # upright runs 7 mm past the pin, the diagonal ends on it
        # wedge in the inner corner between the upright post and the arm's top (FEM: the sharp corner there was at safety 1.32)
        from parts.tubeframe import hull_solid
        xi, zt = e[0] - pw / 2, z + d * 0.44
        f.add(hull_solid([(x_, y_, z_) for y_ in (e[1] - pw / 2, e[1] + pw / 2) for x_, z_ in ((xi - 9.0, zt - 1.0), (xi + 1.0, zt - 1.0), (xi + 1.0, zt + 9.0), (xi - 9.0, zt))]))
    arm = f.build()
    y0, y1 = YF - BOSS_L / 2 + 0.4, YR + BOSS_L / 2 - 0.4       # guard: beside the head's frames (x < 22.5) only the barrel's end collars reach its ends
    arm = arm.cut(box(-20, 22.5, y0 - 30, y0, z - 30, z + 30)).cut(box(-20, 22.5, y1, y1 + 30, z - 30, z + 30))
    arm = arm.fuse(cyl((XI, YF - BOSS_L / 2, z), (XI, YR + BOSS_L / 2, z), 5.0)).clean()      # round thrust collars: the barrel's ends, proud of the trimmed joint
    arm = arm.cut(cyl((XI, y0 - 2, z), (XI, y1 + 2, z), ROD_D / 2 + 0.1))
    arm = max(arm.Solids(), key=lambda q: q.Volume())
    arm = arm.cut(cyl((ax - 15.0, KY, z), (ax + 1, KY, z), 2.05))                        # M3 insert / shank
    if upper:
        # fork for the shock eye: a slot around the eye, open upward and outboard (the eye drops in from above) and a channel inboard for the
        # shock's neck. The post tubes below the pocket stay whole, so each cheek is a full 4.4 mm (FEM: a slot down through the posts left
        # 2.65 mm cheeks, safety 0.34).
        e = SHOCK_EYE; g = FORK_GAP / 2
        arm = arm.cut(box(e[0] - 5.2, e[0] + 9.0, e[1] - g, e[1] + g, e[2] - 5.2, e[2] + 14.0))      # open to the outboard side too: no thin web between the cheeks
        # neck channel as a fan: the shock swings up to 25 degrees against the post (lean plus droop)
        fan = cq.Workplane('XZ', origin=(0, e[1] + g, 0)).polyline([(e[0] - 5.2, e[2] - 5.2), (e[0] - 18.0, e[2] - 5.2 - 12.8 * math.tan(math.radians(25.0))),
                                                                     (e[0] - 18.0, e[2] + 14.0), (e[0] - 5.2, e[2] + 14.0)]).close().extrude(2 * g).val()
        arm = arm.cut(fan)
        arm = arm.cut(cyl((e[0], e[1] - POST_D, e[2]), (e[0], e[1] + POST_D, e[2]), 2.05))                               # shock pin
    arm = max(arm.Solids(), key=lambda q: q.Volume())
    assert arm.isValid(), 'arm is not valid'
    return arm


def ball_cup(c, axis, d):
    """Ball cup on the ball at c: ring (OD CUP_OD, CUP_T thick) with its axis along `axis` ('y' or 'z'), a spherical socket open toward both faces by a
    52 degree cone (the stud's neck, 31 degrees wide at the ball, swings 20 degrees in it; the ball stays held: opening 5.4 mm, ball 6.8), neck and M3 shank along the unit direction d (in the ring's plane)."""
    c = V(*c); a = V(0, 1, 0) if axis == 'y' else V(0, 0, 1)
    d = V(*d).normalized()
    ring = cq.Solid.makeCylinder(CUP_OD / 2, CUP_T, c - a * (CUP_T / 2), a)
    neck = cq.Solid.makeCylinder(2.6, 8.0, c, d)
    shank = cq.Solid.makeCylinder(1.95, 13.5, c + d * 8.0, d)
    cup = _fuse(ring.fuse(neck), shank).cut(cq.Solid.makeSphere(BALL_R + 0.05, c, angleDegrees1=-90))
    for q in (1, -1):        # relief cone on each face: apex at the ball centre, 52 degrees half angle
        cup = cup.cut(cq.Solid.makeCone(0.05, 0.05 + 6.0 * math.tan(math.radians(52.0)), 6.0, c, a * q))
    return cup


def ball_stud(c, d):
    """6.8 mm ball stud: ball at c, neck, flange and M3 thread (7 mm) along the unit direction d."""
    c = V(*c); d = V(*d).normalized()
    ball = cq.Solid.makeSphere(BALL_R, c, angleDegrees1=-90)
    neck = cq.Solid.makeCylinder(NECK_R, CUP_T / 2 + NECK_L, c, d)
    fl = cq.Solid.makeCylinder(FLANGE_R, FLANGE_T, c + d * (CUP_T / 2 + NECK_L), d)
    th = cq.Solid.makeCylinder(1.45, 12.0, c + d * (CUP_T / 2 + NECK_L + FLANGE_T), d)
    return _fuse(ball, neck, fl, th)


def knuckle():
    """+x knuckle: an upright plate behind the two ball joints (their studs screw into it along Y), a block and the axle boss inside the rim,
    the steering arm at the plate's foot running back to the tie rod ball."""
    y0, y1 = TAB_Y0, TAB_Y0 + TAB_T
    pieces = [box(KX - 7.0, WHEEL_IN_X - 0.5, y0, y1, ZL - 7.0, ZU + 7.0)]                 # upright plate, inboard of the tyre
    pieces.append(box(WHEEL_IN_X - 0.5, 119.5, KY - 9.0, y1, ZC - 12.0, ZC + 12.0))        # block inside the rim
    pieces.append(cyl((WHEEL_IN_X - 0.5, KY, ZC), (142.0, KY, ZC), 8.0))                   # axle boss
    pieces.append(box(WHEEL_IN_X - 0.5, 119.5, y0, y0 + 6.0, ZC + 12.0, ZC + 17.0))        # gusset behind the plate, inside the rim (its far corner is 22.3 from the axle, rim 24)
    sx, sy = STEER_ARM_XY
    arm = cq.Workplane('XY', origin=(0, 0, ARM_PLATE_Z0)).polyline([(KX - 7.0, y0), (KX + 6.5, y0), (KX + 6.5, y1), (sx + 5.0, sy), (sx - 5.0, sy)]).close().extrude(ARM_PLATE_T).val()
    pad = cq.Workplane('XY', origin=(0, 0, ARM_PLATE_Z0)).moveTo(sx, sy).circle(5.0).extrude(ARM_PLATE_T).val()
    pieces += [arm, pad]
    k = _fuse(*pieces)
    for z in (ZU, ZL):
        k = k.cut(cyl((KX, y0 - 1, z), (KX, y1 + 3, z), 1.5))                              # M3 thread for the arm studs
    k = k.cut(cyl((sx, sy, ARM_PLATE_Z0 - 1), (sx, sy, ARM_PLATE_Z0 + ARM_PLATE_T + 1), 1.5))
    k = k.cut(cyl((WHEEL_IN_X - 1.5, KY, ZC), (143.0, KY, ZC), ROD_D / 2 + 0.05))
    assert k.isValid() and len(k.Solids()) == 1, 'knuckle is not one valid solid'
    return k


def head():
    """The head as one printed tube frame on the deck behind the skid: a front and a rear centre post, each with a cross bar at the two pivot
    heights (the bars carry the pivot rod bores 7 mm from their ends), a base tube across the deck under each post with a gusset strut on each side,
    and a top and a bottom tube joining the two posts. model.py seats the base tubes on the deck and makes the screw holes."""
    from parts.tubeframe import Frame
    d = HEAD_D
    zt = HEAD_TOP - TS / 2
    f = Frame()
    zb = {}
    for y in (HY0, HY1):
        zb[y] = round(deck_top(y) + TS / 2 - 0.3, 1)                      # 0.3 mm into the deck: model.py cuts it flat on the deck
        f.tube((0, y, zb[y]), (0, y, zt), d)                              # post
        fx = FOOT_X[0] if y == HY0 else FOOT_X[1]
        f.tube((-fx, y, zb[y]), (fx, y, zb[y]), d)                        # base tube
        for z in (ZL, ZU):
            f.tube((-XI - 7.0, y, z), (XI + 7.0, y, z), d)                # cross bar: runs 7 mm past each pivot bore (no round bosses: they left slivers)
        for s in (1, -1):
            f.tube((s * (fx - 8.0), y, zb[y]), (0, y, ZL - 8.0), d)       # gusset strut
    f.tube((0, HY0, zt), (0, HY1, zt), d)                                 # top tie
    f.tube((0, HY0, zb[HY0]), (0, HY1, zb[HY1]), d)                       # bottom tie on the deck
    h = f.build()
    for s in (1, -1):
        for z in (ZU, ZL):
            h = h.cut(cyl((s * XI, HY0 - TS, z), (s * XI, HY1 + TS, z), ROD_D / 2 + 0.05))
    assert h.isValid() and len(h.Solids()) == 1, 'head is not one valid solid'
    return h


def pivot_hardware(s):
    """Side s: the two pivot rods (6 mm shoulder screws from the rear: head behind the rear post, thread in the front post): [(name, shape)]."""
    out = []
    for nm, z in (('upper', ZU), ('lower', ZL)):
        rod = cyl((s * XI, HY0 - TS / 2 + 1.0, z), (s * XI, HY1 + TS / 2, z), ROD_D / 2)
        hd = cyl((s * XI, HY1 + TS / 2 + 0.05, z), (s * XI, HY1 + TS / 2 + 2.0, z), 4.5)      # low head: the tie rod ends pass 1.5 mm behind it
        out.append(('Pivot rod %s %d' % (nm, s), _fuse(rod, hd)))
    return out


def shock_parts():
    """Heave shock between the two post eyes: body at -x, shaft at +x. Each end has a flat eye and a thin neck that fits the fork gap."""
    ex, yg, ez = SHOCK_EYE
    eye_t = FORK_GAP - 0.2
    nk = 14.0
    def end(x, q):                                        # eye at x, neck toward q (+1 / -1)
        eye = cq.Solid.makeCylinder(4.5, eye_t, V(x, yg - eye_t / 2, ez), V(0, 1, 0))
        neck = cyl((x, yg, ez), (x + q * nk, yg, ez), 1.6)
        return _fuse(eye, neck).cut(cq.Solid.makeCylinder(2.0, eye_t + 2, V(x, yg - eye_t / 2 - 1, ez), V(0, 1, 0)))
    xb = -ex + nk + 28.0                                  # shaft end of the body
    body = _fuse(end(-ex, 1), cyl((-ex + nk, yg, ez), (xb, yg, ez), SHOCK_R)).cut(cyl((xb - 20.0, yg, ez), (xb + 1.0, yg, ez), 3.2))   # bore the shaft slides in
    shaft = _fuse(end(ex, -1), cyl((xb - 12.0, yg, ez), (ex - nk, yg, ez), 3.0))
    return body, shaft


# ---------------------------------------------------------------------------------------------------------------------------------
# assembly: every part in world coordinates, both sides
PRINT = (0.16, 0.17, 0.20)       # printed PETG
STEEL = (0.78, 0.80, 0.84)
TIE_END = 10.5                    # ball centre to the end of the tube
OLD_FRONT = {'arm', 'uplink', 'knuckle', 'carrier', 'steer horn', 'steer servo', 'tie rod', 'tip shaft', 'mock axle', 'w3 arm pivot block'}


def _mir(shape, s):
    return shape if s > 0 else shape.mirror('YZ')


def tie_rod_ends(s):
    """Tie rod axis end points (steering arm ball, horn ball) on side s at rest, z = ZT."""
    return V(s * STEER_ARM_XY[0], STEER_ARM_XY[1], ZT), V(s * HORN_BALL[0], HORN_BALL[1], ZT)


def horn_ball(s, eta=0.0):
    """Horn ball of side s (x, y) with the horn turned by eta (deg, right hand about +z)."""
    hx, hy = s * HORN_BALL[0], HORN_BALL[1] - SERVO_SHAFT_Y
    c, sn = math.cos(math.radians(eta)), math.sin(math.radians(eta))
    return c * hx - sn * hy, SERVO_SHAFT_Y + sn * hx + c * hy


SERVO_SCREWS = ((12.5, 'front'), (12.5, 'rear'))      # M3 from below through the deck into the mount's pillars (x, which pillar), +x side


def _servo_y():
    y0 = SERVO_SHAFT_Y - SERVO_SHAFT_IN
    return y0, y0 + SERVO_BOX[1]


def servo_pillar_y(which):
    y0, y1 = _servo_y()
    return y0 - SERVO_EAR[0] / 2 if which == 'front' else y1 + SERVO_EAR[0] / 2


def servo_parts():
    """Servo standing on the deck side of its mount (shaft up, ears at both ends), shaft stub, the printed T horn on a tall hub above it, and the
    printed mount: a pillar under each ear (two servo screws from above) joined by a rail along each side of the servo, everything on top of the
    deck (model.py cuts the deck out of it and screws it on from below)."""
    bx, by, bz = SERVO_BOX
    y0, y1 = _servo_y()
    el, et, ez = SERVO_EAR
    body = box(-bx / 2, bx / 2, y0, y1, SERVO_Z0, SERVO_Z0 + bz)
    for ya, yb in ((y0 - el, y0), (y1, y1 + el)):
        ear = box(-bx / 2, bx / 2, ya, yb, SERVO_Z0 + ez, SERVO_Z0 + ez + et)
        for q in (1, -1):
            ear = ear.cut(cyl((q * 5.0, (ya + yb) / 2, SERVO_Z0 + ez - 1), (q * 5.0, (ya + yb) / 2, SERVO_Z0 + ez + et + 1), 1.5))
        body = body.fuse(ear)
    body = body.clean()
    zt = SERVO_Z0 + bz
    shaft = cyl((0, SERVO_SHAFT_Y, zt), (0, SERVO_SHAFT_Y, zt + 4.0), 2.5)
    wp = cq.Workplane('XY', origin=(0, 0, HORN_Z0))
    hub = cyl((0, SERVO_SHAFT_Y, zt + 0.5), (0, SERVO_SHAFT_Y, HORN_Z0 + HORN_T), 6.5)                  # tall hub from the servo's top up to the arm
    bx_, by_ = HORN_BALL
    strip = wp.polyline([(-5.5, SERVO_SHAFT_Y), (5.5, SERVO_SHAFT_Y), (bx_ + 4.5, by_), (-bx_ - 4.5, by_)]).close().extrude(HORN_T).val()
    horn = _fuse(hub, strip, *[wp.moveTo(q * bx_, by_).circle(5.5).extrude(HORN_T).val() for q in (1, -1)])
    horn = horn.cut(cyl((0, SERVO_SHAFT_Y, zt), (0, SERVO_SHAFT_Y, zt + 4.3), 2.55))                    # blind bore for the output shaft (spline)
    horn = horn.cut(cyl((0, SERVO_SHAFT_Y, zt + 4.3), (0, SERVO_SHAFT_Y, HORN_Z0 + HORN_T + 1), 1.6))   # horn screw from above
    for q in (1, -1):
        horn = horn.cut(cyl((q * bx_, by_, HORN_Z0 - 1), (q * bx_, by_, HORN_Z0 + HORN_T + 1), 1.5))
    # mount
    g, t = 0.5, 3.0
    xo = bx / 2 + g
    ze = SERVO_Z0 + ez                                                    # ear underside = pillar top
    parts = []
    for ya, yb in ((y0 - el, y0 - g), (y1 + g, y1 + el)):
        parts.append(box(-xo - t, xo + t, ya, yb, 8.0, ze))               # pillar under the ear
    for q in (1, -1):
        xa, xb = (xo, xo + t) if q > 0 else (-xo - t, -xo)
        parts.append(box(xa, xb, y0 - el, y1 + el, 8.0, SERVO_Z0 + 8.0))  # side rail joining the pillars
    mount = _fuse(*parts)
    for w in ('front', 'rear'):
        yc = servo_pillar_y(w)
        for q in (1, -1):
            mount = mount.cut(cyl((q * 5.0, yc, ze - 10.0), (q * 5.0, yc, ze + 1), 1.25))      # servo screw pilot holes (M3 self-tapping)
    return body, shaft, horn, mount


def tie_rod(s):
    a, b = tie_rod_ends(s)
    u = (b - a).normalized()
    p0, p1 = a + u * TIE_END, b - u * TIE_END
    tube = cq.Solid.makeCylinder(3.5, (p1 - p0).Length, p0, u).cut(cq.Solid.makeCylinder(1.95, (p1 - p0).Length, p0, u))
    return tube, ball_cup(tuple(a.toTuple()), 'z', tuple(u.toTuple())), ball_cup(tuple(b.toTuple()), 'z', tuple((-u).toTuple()))


def all_parts():
    """[(name, shape, group, colour)] for the whole front suspension and steering, both sides."""
    out = []
    G = 'Chassis/Front suspension'
    out.append(('Lean head', head(), G, PRINT))
    lo, up, kn = a_arm(ZL, False), a_arm(ZU, True), knuckle()
    body, shaft = shock_parts()
    out += [('Heave shock body', body, G, (0.80, 0.15, 0.12)), ('Heave shock shaft', shaft, G, STEEL)]
    bd, sv, hn, mt = servo_parts()
    out += [('Steer servo', bd, 'Chassis/Steering', (0.20, 0.20, 0.22)), ('Steer servo shaft', sv, 'Chassis/Steering', STEEL), ('Steer horn', hn, 'Chassis/Steering', PRINT),
            ('Steer servo mount', mt, 'Chassis/Steering', PRINT)]
    ex, ey, ez = SHOCK_EYE
    for s in (1, -1):
        out += [('Lower arm %d' % s, _mir(lo, s), G, PRINT), ('Upper arm %d' % s, _mir(up, s), G, PRINT), ('Knuckle %d' % s, _mir(kn, s), G, PRINT)]
        for nm, sh in pivot_hardware(s):
            out.append((nm, sh, G, STEEL))
        for nm, z in (('upper', ZU), ('lower', ZL)):       # ball cup on the arm, ball stud in the knuckle plate
            out.append(('Rod end %s %d' % (nm, s), ball_cup((s * KX, KY, z), 'y', (-s, 0, 0)), G, STEEL))
            out.append(('Ball stud %s %d' % (nm, s), ball_stud((s * KX, KY, z), (0, 1, 0)), G, STEEL))
        out.append(('Heave shock pin %d' % s, cq.Solid.makeCylinder(1.95, POST_D * 0.88 + 1.0, V(s * ex, ey - POST_D * 0.44 - 0.5, ez), V(0, 1, 0)), G, STEEL))
        out.append(('Axle pin %d' % s, _mir(cyl((WHEEL_IN_X + 5.6, KY, ZC), (166.0, KY, ZC), ROD_D / 2), s), G, STEEL))
        nut = cq.Solid.makeCylinder(5.2, 5.0, V(160.0, KY, ZC), V(1, 0, 0)).cut(cyl((159.0, KY, ZC), (166.5, KY, ZC), ROD_D / 2))
        out.append(('Axle nut %d' % s, _mir(nut, s), G, STEEL))
        hexhub = cq.Workplane('YZ', origin=(148.2, KY, ZC)).polygon(6, 12.0 / math.cos(math.pi / 6)).extrude(9.6).val()
        hexhub = hexhub.cut(cyl((148.0, KY, ZC), (158.5, KY, ZC), ROD_D / 2 + 0.05))
        out.append(('Wheel hex hub %d' % s, _mir(hexhub, s), G, STEEL))
        tube, e1, e2 = tie_rod(s)
        out += [('Tie rod %d' % s, tube, 'Chassis/Steering', STEEL), ('Tie rod end knuckle %d' % s, e1, 'Chassis/Steering', STEEL), ('Tie rod end horn %d' % s, e2, 'Chassis/Steering', STEEL)]
        a, b = tie_rod_ends(s)
        out.append(('Steer stud knuckle %d' % s, ball_stud(tuple(a.toTuple()), (0, 0, -1)), 'Chassis/Steering', STEEL))     # stud up through the steering arm
        out.append(('Steer stud horn %d' % s, ball_stud(tuple(b.toTuple()), (0, 0, 1)), 'Chassis/Steering', STEEL))         # stud hangs down from the horn
    return out


def strip_old_front(ch):
    """Remove the old front suspension and steering nodes from the imported chassis assembly. Returns the removed names."""
    gone = []
    for n in list(ch.objects.values()):
        nm = str(getattr(n, 'name', '')).replace('_', ' ')
        base = nm.rsplit(' ', 1)[0] if nm.rsplit(' ', 1)[-1] in ('1', '-1') else nm
        if base in OLD_FRONT or nm.startswith('b steer') or nm == 'b front pivot axle':
            if getattr(n, 'parent', None) is not None and n in n.parent.children:
                n.parent.children.remove(n)
            gone.append(nm)
    return gone


# ---------------------------------------------------------------------------------------------------------------------------------
# kinematics (shared by the animation and the clearance test)
TYRE_R, TYRE_W = 56.2, 62.0            # tyre outer radius and width (mock tire)
TYRE_RC = TYRE_W / 2                   # profile radius of the tread shoulder used for the contact height of a leaning wheel
SIDE_X = {1: (1.0, 0.0), -1: (-1.0, 0.0)}


def _loc_axis(p, d, deg):
    """Rotation by deg (right hand) about the axis through p with direction d."""
    return cq.Location(V(*p)) * cq.Location(V(0, 0, 0), V(*d), deg) * cq.Location(V(*p) * -1.0)


def roll_loc(phi):
    """Body roll about the ground line (x = 0, z = ground), right hand about +y: +phi lowers the +x side."""
    return _loc_axis((0, 0, GROUND_Z), (0, 1, 0), phi)


def _rot_xz(x, z, phi_deg):
    c, s = math.cos(math.radians(phi_deg)), math.sin(math.radians(phi_deg))
    return c * x + s * z, -s * x + c * z


def arm_ball(s, psi):
    """Head-frame displacement (dx, dz) of the ball joints when the arms of side s turn by psi (right hand about +y)."""
    ox, oz = _rot_xz(s * ARM_L, 0.0, psi)
    return ox - s * ARM_L, oz


def solve_psi(s, phi):
    """Arm angle (deg) at which the wheel of side s still rests on the ground with the body rolled by phi (parallelogram: both arms turn by psi)."""
    h = (TYRE_R - TYRE_RC) * math.cos(math.radians(phi)) + TYRE_RC

    def zc(psi):
        dx, dz = arm_ball(s, psi)
        wx, wz = _rot_xz(s * 144.0 + dx, ZC + dz - GROUND_Z, phi)
        return wz + GROUND_Z
    lo, hi = -60.0, 60.0
    f = lambda a: zc(a) - (GROUND_Z + h)
    for _ in range(80):
        mid = (lo + hi) / 2
        if f(lo) * f(mid) <= 0:
            hi = mid
        else:
            lo = mid
    return (lo + hi) / 2


def steer_solution(s, psi, eta):
    """Knuckle steering angle (deg about the kingpin, + turns the front toward +x... right hand about +z) for horn angle eta (deg, right hand about +z)
    with the arms at psi: the tie rod keeps its length."""
    a0, b0 = tie_rod_ends(s)
    L0 = (a0 - b0).Length
    dx, dz = arm_ball(s, psi)
    k = V(s * KX, KY, 0)
    b = V(*horn_ball(s, eta), ZT)

    def dist(dl):
        c, sn = math.cos(math.radians(dl)), math.sin(math.radians(dl))
        r = V(a0.x - k.x, a0.y - k.y, 0)
        a = V(k.x + c * r.x - sn * r.y + dx, k.y + sn * r.x + c * r.y, ZT + dz)
        return (a - b).Length - L0
    lo, hi = -60.0, 60.0
    if dist(lo) * dist(hi) > 0:
        return float('nan')
    for _ in range(80):
        mid = (lo + hi) / 2
        if dist(lo) * dist(mid) <= 0:
            hi = mid
        else:
            lo = mid
    return (lo + hi) / 2


def tie_rod_loc(s, psi, delta, eta):
    """Head-frame placement of the tie rod body (tube and both rod ends) as a Location, and its two end points."""
    a0, b0 = tie_rod_ends(s)
    dx, dz = arm_ball(s, psi)
    k = V(s * KX, KY, 0)
    c, sn = math.cos(math.radians(delta)), math.sin(math.radians(delta))
    r = V(a0.x - k.x, a0.y - k.y, 0)
    a = V(k.x + c * r.x - sn * r.y + dx, k.y + sn * r.x + c * r.y, ZT + dz)
    b = V(*horn_ball(s, eta), ZT)
    u0, u1 = (b0 - a0).normalized(), (b - a).normalized()
    ax = u0.cross(u1)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, u0.dot(u1)))))
    loc = cq.Location(a - a0)
    if ax.Length > 1e-9 and ang > 1e-9:
        loc = loc * _loc_axis(tuple(a0.toTuple()), tuple(ax.toTuple()), ang)
    return loc, a, b


def shock_locs(psi_l, psi_r):
    """Placement of the heave shock body (on the -x post eye) and shaft (on the +x eye), plus the new eye distance."""
    ex, ez = SHOCK_EYE[0], SHOCK_EYE[2]
    def eye(s, psi):
        px, pz = s * XI, ZU
        ox, oz = _rot_xz(s * ex - px, ez - pz, psi)
        return px + ox, pz + oz
    el, er = eye(-1, psi_l), eye(1, psi_r)
    ang = math.degrees(math.atan2(-(er[1] - el[1]), er[0] - el[0]))      # right hand about +y: +x tips toward -z
    yg = SHOCK_EYE[1]
    lb = cq.Location(V(el[0] + ex, 0, el[1] - ez)) * _loc_axis((-ex, yg, ez), (0, 1, 0), ang)
    ls = cq.Location(V(er[0] - ex, 0, er[1] - ez)) * _loc_axis((ex, yg, ez), (0, 1, 0), ang)
    return lb, ls, math.hypot(er[0] - el[0], er[1] - el[1])
