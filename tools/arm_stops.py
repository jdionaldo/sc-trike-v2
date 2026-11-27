"""Per-arm stops and the steering-servo mount of the version 2 swing arms. Pure math plus 3D checks against the built STEP, no build.
Run: python -I tools/arm_stops.py

Coordinator decision (round 2): there is NO per-arm bump stop. No PETG section in the space can carry the 172 N of a one-wheel landing
(fix round 1, docs/v2/arm_stops.md). Bump travel ends on the printed carrier and beam-yaw stops of tools/beam_kin.py; one wheel then reaches
about +41.9 mm. The per-arm DROOP stop stays: a TPU wedge on the rear face of the upper link's rail lug, hit by the front face of the lever
blade (parts/swingarm_front.py lever_ref) at -32.9.
This tool prints:
- the droop stop (geometry, load, clearances in 3D);
- the reachable (t+x, t-x) region of the carrier, yaw and droop stops, and its corner poses (the pose list for the design thread);
- the forces on the carrier bump pad and the yaw pads for the 65 N landing (analyses/front_arm.py), with hand stresses;
- the servo mount (wedge floor + clip), checked in 3D against the built output/model.step.
World mm (Z up, front = -Y), +x side; mirror x for the -x side.
"""
import math, os, sys
import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _swingarm2_snapshot as sa
import beam_kin as bk

# ---------------------------------------------------------------------------------------------------------------------------------------
# blade (from parts/swingarm_front.py at 7b79b4c): plane, thickness, half widths at the root and at the top
BLADE_X = (sa.LEVER_X - 4.0, sa.LEVER_X + 4.0)          # 62.2 .. 70.2
H_ROOT, H_TOP = 11.0, 5.5

STOP_DESIGN = {
    'droop_travel':     -32.9,
    # SERVO MOUNT (round 2): only holds the servo. PART 1 = wedge floor plate under the servo; PART 2 = clip over its inboard edge, with a thin strap
    # across the servo top under the cage rail. ONE M3 screw from below through the deck and the floor's tongue into part 2's heat-set insert clamps both.
    'SERVO_MOUNT': {
        'floor_x': (32.0, 54.5), 'floor_tmin': 0.8,           # under the servo footprint on the deck; top z 28.2 = servo bottom, underside = deck top
        'tongue': (19.0, -104.9, 31.0 + 1.0, -92.9),          # x0, y0, x1, y1: inboard extension under part 2's leg (the clip screw passes through it)
        'rim':   (32.0, -106.8, 47.0, -105.3, 1.5),            # ends 0.4 mm short of the servo tab zone (x 47.4, parts/swingarm2.py SERVO_TAB_X)           # x0, y0, x1, y1, height over z 28.2: locates the servo's front face
        'leg':   (19.0, -104.9, 31.0, -92.9, 53.0),           # part 2 leg x0, y0, x1, y1, top z (stands on the tongue)
        'arm':   (19.0, -104.9, 39.4, -92.9, 48.9, 53.0),     # part 2 arm over the servo's inboard edge (0.5 mm EVA), 1.2 mm from the rail's inner face x 40.6
        'strap': (39.4, -101.7, 57.0, -95.7, 48.9, 50.4),     # 1.5 mm strap across the servo top under the rail (bottom z 52.6), 2.2 mm short of the horn (x 59.2)
        'inserts': [(25.0, -98.9)],                          # M3 heat-set from below, pocket D4.0 x 8.5, >= 6 mm from every part edge
        'eva_t': 0.5, 'deck_edge_x': 54.5, 'edge_rule': 6.0,
    },
    # DROOP: wedge on the lug's rear face (y -113.4), filling the space up to the blade's front face at droop, over z z0..z1
    'droop_pad_z':      (80.0, 90.0),            # the whole lug height: the lug's lower rear corner lies under the contact face
    'droop_pad_tmin':   2.0,                      # TPU at least 2 mm thick normal to the face: the lug corner is chamfered back to that plane
    'droop_pad_x':      (61.7, 70.7),
    'lug_rear_y':       sa.UPLINK_IN[1] + 6.0,    # -113.4 (see LUG below)
    # loads (from analyses/front_arm.py at 7b79b4c: 65 N up at the tip shaft holes = a landing on one front wheel; side load 30 N gives no moment
    # about the pivot axis). The stop takes the whole moment: the heave spring's share is ignored (conservative).
    'bump_wheel_N':     65.0,                     # one-wheel landing (analyses/front_arm.py): now reacted by the carrier and yaw stops
    'droop_wheel_N':    30.0,                     # ASSUMED: wheel in the air, its spring share (about 8 N) + 0.25 kg unsprung at 8 g (20 N), rounded up
}
# Upper link rail lug and inner clevis (cage), DERIVED from parts/uplink.py rail_lug() and clevis_u() at 7b79b4c with RAIL_REF (45, -142, 100.3 ref
# = world z 85.0), GAP 6.4, PLATE_T 3.0, WIDTH 12 from parts/shock_mount.py and H_IN_W (68.5, -119.4, 97.9):
#   lug box    x RAIL_REF.x - 4 = 41.0 .. H_IN.x + (GAP + 2 PLATE_T) / 2 = 74.7, y H_IN.y +- WIDTH / 2 = -125.4 .. -113.4, z 85.0 +- 5 = 80.0 .. 90.0
#   clevis     x 62.3 .. 74.7, same y, z H_IN.z - 12 = 85.9 .. H_IN.z + 5 = 102.9 (plates at x 62.3..65.3 and 71.7..74.7, floor z 85.9..89.7)
LUG = {'x': (41.0, 74.7), 'y': (sa.UPLINK_IN[1] - 6.0, sa.UPLINK_IN[1] + 6.0), 'z': (80.0, 90.0), 'clevis_z_top': sa.UPLINK_IN[2] + 5.0,
       'clevis_x': (62.3, 74.7), 'plate_t': 3.0}
# the lug's lower rear corner is chamfered over the FULL lug width LUG['x'] (coordinator, fix round 2); the TPU wedge covers only the blade band
# carrier sections for the clearance checks and the hand stresses (fix round 2): arm to the upper yoke 14 (x) x 24, arms to S and to the tail 12 x 10
# fix round 3: the clevis plates move out to |x| 9.4..12.4 (bk.CLEVIS_PLATES_X, gap 18.8): the hub is 16.8 long (1 mm per side, shim washers),
# the tail boss 14.8; the other carrier parts stay <= 10.8 wide;
# the arm to the yoke is 18 deep, its section 2 mm under the PC-yoke line, so it stays >= 1 mm under the spine / clevis bridge (z 108.5)
CARRIER_SECTIONS = {'hub_r': 6.0, 'hub_len': 16.8, 'arm_c': (10.8, 18.0, -2.5), 'arm_s': (10.8, 10.0), 's_fork_r': 6.5, 's_fork_half': 5.4,
                    'pin_boss_r': 7.0, 'pin_boss_up': 4.0, 'clip_r': 4.0, 'clip_h': 2.5, 'yaw_pin_d': 8.0}
TPU_ALLOW = 5.0       # ASSUMED MPa, TPU 95A pad pressure for repeated impacts
YAW_POST = (6.0, 8.0, 5.0)   # ASSUMED yaw pad post on the carrier: width, depth (in the load direction), height (mm)
TPU_E = 26.0          # ASSUMED MPa, TPU 95A in compression (small strain)
PETG_ALLOW = 20.0     # MPa, PETG treated as solid (as in analyses/front_arm.py), safety 2 asked there
# Surroundings: the BUILT model (read-only). The servo, horn and drag link positions come from it; their swing is modelled here.
STEP_PATH = '/Users/johndionaldo/GitHub/sc-trike-v2/output/model.step'
STATIC = ['Front_shell', 'w3_deck_front', 'w3_deck_rear', 'Cage', 'Steer_servo_1', 'Steer_horn_1', 'Drag_link_1', 'Rocker_1', 'Rocker_bracket_1',
          'Rocker_ball_stud_in_1', 'Rocker_ball_stud_out_1', 'Tie_rod_1', 'w3_arm_pivot_block_1', 'b_front_pivot_axle', 'mock_receiver', 'b_motor',
          'w3_motor_plate', 'arm_1', 'uplink_1', 'Lever_ball_stud_1', 'Cup_block_1', 'Balance_beam', 'Beam_carrier', 'Heave_shock_body', 'Droop_stop_pad_1']
SURROUND = {
    'servo': (31.5, -105.3, 28.2, 58.5, -64.5, 48.4),            # worst-case case, shifted 3.3 rearward (NOTES 27); was (32.0, -108.6, 28.2, 58.5, -67.8, 48.4)            # Steer_servo_1 box (verified place; the built body reaches x 59.2 with its shaft)
    'servo_shaft': (-95.1, 38.3),
    'horn': {'x': (59.2, 61.5), 'hub_r': 5.0, 'end_r': 3.5, 'ball': (68.5, -93.70, 26.38), 'ball_r': 3.4, 'stud_r': 1.5,
             'swing_deg': 30.0},                                 # swept model: hub and end radii ASSUMED; servo +-30 deg (steer-kin)
    'drag_link': {'a': (68.5, -93.70, 26.38), 'b': (66.0, -109.70, 24.5), 'r': 4.0},
    'rail': (40.6, -140.0, 52.6, 49.4, -48.0, 61.4),
}
RAIL = {'x': (SURROUND['rail'][0], SURROUND['rail'][3]), 'z': (SURROUND['rail'][2], SURROUND['rail'][5])}
_STEP = {}


def step_bodies(names=STATIC, path=STEP_PATH):
    """Named bodies of the built STEP (XCAF names as in the app), cached for the run."""
    if _STEP:
        return _STEP
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TDF import TDF_LabelSequence, TDF_Label
    from OCP.TDataStd import TDataStd_Name
    from OCP.TopLoc import TopLoc_Location
    from OCP.IFSelect import IFSelect_RetDone
    doc = TDocStd_Document(TCollection_ExtendedString('XmlOcaf'))
    r = STEPCAFControl_Reader(); r.SetNameMode(True)
    assert r.ReadFile(path) == IFSelect_RetDone, 'cannot read ' + path
    r.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    def name(l):
        a = TDataStd_Name()
        return a.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(), a) else '?'
    def walk(l, loc):
        if st.IsReference_s(l):
            ref = TDF_Label(); st.GetReferredShape_s(l, ref)
            walk(ref, loc.Multiplied(st.GetLocation_s(l))); return
        if st.IsAssembly_s(l):
            ch = TDF_LabelSequence(); st.GetComponents_s(l, ch)
            for i in range(1, ch.Length() + 1):
                walk(ch.Value(i), loc)
            return
        nm = name(l).replace(' ', '_')
        if nm in names:
            _STEP[nm] = st.GetShape_s(l).Moved(loc)
    roots = TDF_LabelSequence(); st.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location())
    return _STEP


def ray_z(shape, x, y):
    """z of all surface crossings of a vertical line at (x, y) through a shape, sorted."""
    from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
    from OCP.gp import gp_Lin, gp_Pnt, gp_Dir
    it = IntCurvesFace_ShapeIntersector(); it.Load(shape, 1e-6)
    it.Perform(gp_Lin(gp_Pnt(x, y, 300), gp_Dir(0, 0, -1)), -1e9, 1e9)
    return sorted(it.Pnt(k).Z() for k in range(1, it.NbPnt() + 1))


def dist3(a, b):
    """BRepExtrema distance between two shapes (TopoDS or CadQuery)."""
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    d = BRepExtrema_DistShapeShape(getattr(a, 'wrapped', a), getattr(b, 'wrapped', b)); d.Perform()
    return d.Value()


def overlap3(a, b):
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    c = BRepAlgoAPI_Common(getattr(a, 'wrapped', a), getattr(b, 'wrapped', b)); g = GProp_GProps(); BRepGProp.VolumeProperties_s(c.Shape(), g)
    return g.Mass()
# ---------------------------------------------------------------------------------------------------------------------------------------

D = math.degrees
PY, PZ = sa.PIVOT


def blade_angle(t):
    """Lever angle from vertical toward the rear (rad) at wheel travel t."""
    return math.radians(sa.LEVER_TILT) + sa.arm_swing(t)


def frame(a):
    """Unit radial (pivot -> top) and rear normal of the blade in (y, z)."""
    return np.array([math.sin(a), math.cos(a)]), np.array([math.cos(a), -math.sin(a)])


def half(r):
    return H_ROOT + (H_TOP - H_ROOT) * r / sa.LEVER_R


def blade_outline(t, step=0.25, skip=None):
    """Points on the blade's outline in (y, z) at travel t: the tapered blade plus its root disc (R 11) and top disc (R 5.5).
    skip = (side, r0, r1) leaves out that stretch of one face (the contact face) to measure the clearance of everything else."""
    a = blade_angle(t)
    er, en = frame(a)
    P = np.array([PY, PZ])
    pts = []
    for r in np.arange(0, sa.LEVER_R + 1e-9, step):
        for s in (1, -1):
            if skip and s == skip[0] and skip[1] <= r <= skip[2]:
                continue
            pts.append(P + r * er + s * half(r) * en)
    for c, R in ((P, H_ROOT), (P + sa.LEVER_R * er, H_TOP)):
        for q in np.linspace(0, 2 * math.pi, 120, endpoint=False):
            pp = c + R * np.array([math.cos(q), math.sin(q)])
            if skip and ((pp - P) @ en) * skip[0] > half((pp - P) @ er) - 0.3 and skip[1] <= (pp - P) @ er <= skip[2]:
                continue
            pts.append(pp)
    return np.array(pts)


def face_point(t, r, s):
    """Point on the blade's rear (s = +1) or front (s = -1) face at radius r, travel t."""
    a = blade_angle(t)
    er, en = frame(a)
    return np.array([PY, PZ]) + r * er + s * half(r) * en


def poly_dist(pts, poly):
    """Signed distance of points to a convex polygon (negative inside); returns the minimum."""
    poly = np.asarray(poly)
    n = len(poly)
    best = 1e9
    for p in pts:
        inside, dmin = True, 1e9
        for i in range(n):
            a, b = poly[i], poly[(i + 1) % n]
            e = b - a
            cr = e[0] * (p[1] - a[1]) - e[1] * (p[0] - a[0])
            if cr > 0:
                inside = False
            u = np.clip((p - a) @ e / (e @ e), 0, 1)
            dmin = min(dmin, np.linalg.norm(p - (a + u * e)))
        best = min(best, -dmin if inside else dmin)
    return best


def orient_cw(poly):
    """Make the polygon clockwise in (y, z) (poly_dist treats a left turn as outside)."""
    poly = np.asarray(poly)
    area = sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1] for i in range(len(poly)))
    return poly if area < 0 else poly[::-1]


# ---- servo mount (part 1 carries the bump pad) ----
def _box(b):
    import cadquery as cq
    return cq.Solid.makeBox(b[3] - b[0], b[4] - b[1], b[5] - b[2], cq.Vector(b[0], b[1], b[2]))


def deck_profile():
    """Deck top and underside of w3_deck_front at x 43 (measured on the built STEP) as linear functions of y."""
    sh = step_bodies()['w3_deck_front']
    ys = (-108.6, -67.8)
    zz = [ray_z(sh, 43.0, y) for y in ys]
    top = lambda y: zz[0][-1] + (y - ys[0]) * (zz[1][-1] - zz[0][-1]) / (ys[1] - ys[0])
    bot = lambda y: zz[0][0] + (y - ys[0]) * (zz[1][0] - zz[0][0]) / (ys[1] - ys[0])
    return top, bot


def mount_solids(d):
    """Part 1 (wedge floor plate with its tongue and front rim) and part 2 (clip with strap) as CadQuery solids; deck profile functions."""
    import cadquery as cq
    sm = d['SERVO_MOUNT']
    top, bot = deck_profile()
    zf = SURROUND['servo'][2]
    yf = SURROUND['servo'][1]
    slope = (top(-67.8) - top(yf)) / (-67.8 - yf)
    ye = yf + (zf - sm['floor_tmin'] - top(yf)) / slope                    # the floor ends where it would get thinner than floor_tmin
    sm['floor_y'] = (yf, round(ye, 2)); sm['floor_t'] = (round(zf - top(yf), 2), round(zf - top(ye), 2))
    floor = _prism([(yf, top(yf)), (ye, top(ye)), (ye, zf), (yf, zf)], *sm['floor_x'])
    tx0, ty0, tx1, ty1 = sm['tongue']
    tongue = _prism([(ty0, top(ty0)), (ty1, top(ty1)), (ty1, zf), (ty0, zf)], tx0, tx1)
    rx0, ry0, rx1, ry1, rh = sm['rim']
    rim = _prism([(ry0, top(ry0)), (ry1, top(ry1)), (ry1, zf + rh), (ry0, zf + rh)], rx0, rx1)
    m1 = floor.fuse(tongue).fuse(rim).clean()
    lx0, ly0, lx1, ly1, lz1 = sm['leg']
    m2 = _box((lx0, ly0, zf, lx1, ly1, lz1))
    ax0, ay0, ax1, ay1, az0, az1 = sm['arm']
    m2 = m2.fuse(_box((ax0, ay0, az0, ax1, ay1, az1)))
    sx0, sy0, sx1, sy1, sz0, sz1 = sm['strap']
    m2 = m2.fuse(_box((sx0, sy0, sz0, sx1, sy1, sz1))).clean()
    for ix, iy in sm['inserts']:                                          # clip screw: clearance hole through the tongue, insert pocket in the leg
        m1 = m1.cut(cq.Solid.makeCylinder(1.7, 20.0, cq.Vector(ix, iy, 10.0), cq.Vector(0, 0, 1)))
        m2 = m2.cut(cq.Solid.makeCylinder(2.0, 8.5, cq.Vector(ix, iy, zf - 0.01), cq.Vector(0, 0, 1)))
    assert m1.isValid() and len(m1.Solids()) == 1 and m2.isValid() and len(m2.Solids()) == 1, 'servo mount parts are not single solids'
    return m1.clean(), m2.clean(), top, bot


def horn_solids(eta):
    """Servo horn plate (hub + end + web), ball stud and ball at servo angle eta (deg) about the shaft (x axis)."""
    import cadquery as cq
    H = SURROUND['horn']; sy, sz = SURROUND['servo_shaft']
    bx, by, bz = H['ball']
    c, sn = math.cos(math.radians(eta)), math.sin(math.radians(eta))
    dy, dz = by - sy, bz - sz
    ey, ez = sy + c * dy - sn * dz, sz + sn * dy + c * dz
    x0, x1 = H['x']
    L = math.hypot(ey - sy, ez - sz); ny, nz = -(ez - sz) / L, (ey - sy) / L
    web = _prism([(sy + H['hub_r'] * ny, sz + H['hub_r'] * nz), (ey + H['end_r'] * ny, ez + H['end_r'] * nz),
                  (ey - H['end_r'] * ny, ez - H['end_r'] * nz), (sy - H['hub_r'] * ny, sz - H['hub_r'] * nz)], x0, x1)
    s = web.fuse(cq.Solid.makeCylinder(H['hub_r'], x1 - x0, cq.Vector(x0, sy, sz), cq.Vector(1, 0, 0)))
    s = s.fuse(cq.Solid.makeCylinder(H['end_r'], x1 - x0, cq.Vector(x0, ey, ez), cq.Vector(1, 0, 0)))
    s = s.fuse(cq.Solid.makeCylinder(H['stud_r'], bx - x1, cq.Vector(x1, ey, ez), cq.Vector(1, 0, 0)))
    s = s.fuse(cq.Solid.makeSphere(H['ball_r'], cq.Vector(bx, ey, ez)))
    return s.clean(), (bx, ey, ez)


def capsule(a, b, r):
    import cadquery as cq
    a, b = cq.Vector(*a), cq.Vector(*b)
    s = cq.Solid.makeCylinder(r, (b - a).Length, a, (b - a).normalized())
    return s.fuse(cq.Solid.makeSphere(r, a)).fuse(cq.Solid.makeSphere(r, b)).clean()


def lever_top_solids(t):
    """Lever ball cup (sphere CUP_R) and the stud boss (R 4.2 from the ball to the lever plane) at travel t."""
    import cadquery as cq
    y, z = sa.lever_top(t)
    s = cq.Solid.makeSphere(bk.CUP_R, cq.Vector(sa.LEVER_BALL_X, y, z))
    return s.fuse(cq.Solid.makeCylinder(4.2, sa.LEVER_X - sa.LEVER_BALL_X, cq.Vector(sa.LEVER_BALL_X, y, z), cq.Vector(1, 0, 0))).clean()


def _obox(c, ex, ey, ez, hx, hy, hz):
    """Oriented box: centre c, unit axes ex, ey, ez, half sizes."""
    import cadquery as cq
    pts = [c + a * hx * ex + b * hy * ey + q * hz * ez for a in (-1, 1) for b in (-1, 1) for q in (-1, 1)]
    return cq.Solid.makeBox(1, 1, 1).moved(cq.Location()) if False else _hull_pts(pts)


def _hull_pts(pts):
    """Convex solid of a few points (CadQuery has no hull: a ruled loft between two quads of an 8-point box)."""
    import cadquery as cq
    P = [cq.Vector(*map(float, p)) for p in pts]
    w0 = cq.Wire.makePolygon([P[0], P[1], P[3], P[2]], close=True)
    w1 = cq.Wire.makePolygon([P[4], P[5], P[7], P[6]], close=True)
    return cq.Solid.makeLoft([w0, w1], True)


def beam_solids(r):
    """+x beam-end fork, the tapered beam bar (22 deep at C to 12 at |x| 40, 12 wide) and the carrier's yaw arm with its +x posts, at a state."""
    import cadquery as cq
    p = r['pose']
    ex, em, en = p['beam_dir'], p['M'], p['N']
    fork = _obox(p['Bp'], ex, em, en, *bk.FORK)
    halves = []
    for sgn in (1, -1):
        c0, c1 = p['C'], p['C'] + sgn * bk.BEAM_INNER_X * ex
        q0 = [c0 + bk.bar_m(0)[(b + 1) // 2] * em + q * bk.BEAM_R * en for b, q in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        q1 = [c1 + bk.bar_m(bk.BEAM_INNER_X)[(b + 1) // 2] * em + q * bk.BEAM_R * en for b, q in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        w0 = cq.Wire.makePolygon([cq.Vector(*map(float, v)) for v in q0], close=True)
        w1 = cq.Wire.makePolygon([cq.Vector(*map(float, v)) for v in q1], close=True)
        halves.append(cq.Solid.makeLoft([w0, w1], True))
    bar = halves[0].fuse(halves[1]).clean()
    # carrier frame (turns with the carrier only): rest axes rotated by Rg
    D_ = bk.BEAM_DESIGN; Rg = p['Rg']
    X0 = np.array([1.0, 0, 0]); N0 = np.array(D_['beam_pin_axis']); M0 = np.cross(N0, X0)
    eX, eM, eN = Rg @ X0, Rg @ M0, Rg @ N0
    ya, yp = D_['yaw_arm'], D_['yaw_post']
    arm = _obox(p['C'] + ya['n_off'] * eN, eX, eM, eN, ya['x_half'], ya['m_half'], ya['n_half'])
    posts = []
    for sx in (1, -1):
        for mr in (yp['m_front'], yp['m_rear']):
            xc = sx * (yp['x'][0] + yp['x'][1]) / 2; mc = (mr[0] + mr[1]) / 2; nc = (yp['n'][0] + yp['n'][1]) / 2
            posts.append(_obox(p['C'] + xc * eX + mc * eM + nc * eN, eX, eM, eN, (yp['x'][1] - yp['x'][0]) / 2, (mr[1] - mr[0]) / 2,
                               (yp['n'][1] - yp['n'][0]) / 2))
    return fork, bar, arm, posts


# heave shock envelope (ASSUMED 1/10-class shock). REQUIREMENT for the bought shock: the lower spring retainer (R 8.25) starts >= 14 mm behind the
# front eye centre (a spacer under the retainer if needed): at 12 mm the carrier's tail boss comes within 0.18 mm of it at droop, at 14 mm 1.11 mm
# (NOTES 60) the bought heave shock, RC4WD Ultimate Scale Shock 70 mm RC4Z-D0004 (store.hobbyetc.com/parts/view/29506): internal spring, body D8.8,
# shaft D3.0, 72.5 extended / 55.1 compressed. No external spring or retainer: 'spring_r' is now the body radius 4.4, and the body's front end sits
# 18 mm behind the front eye at ride height (68 - (55.1 - 5): the body is 50.1 mm from the rear eye, 5 mm of shaft and eye left at full compression,
# ASSUMED). Was: shaft r 2.0, spring cup 14 mm behind the front eye, spring r 8.25 (a 1/10 class coil-over envelope).
SHOCK_GEOM = {'eye_r': 4.0, 'eye_w': 6.0, 'shaft_r': 1.5, 'cup_from_front_eye': 18.0, 'spring_r': 4.4}


def shock_solids(k, r):
    """Heave shock as modelled for clearances: front eye at S, shaft to the lower spring cup (ASSUMED 12 mm from the eye), spring R 8.25 to 5 mm
    before the rear eye, rear eye at E."""
    import cadquery as cq
    S, E = r['pose']['S'], k.E
    u = (E - S) / np.linalg.norm(E - S); g = SHOCK_GEOM; Ls = np.linalg.norm(E - S)
    V_ = lambda a: cq.Vector(*map(float, a))
    s = cq.Solid.makeCylinder(g['eye_r'], g['eye_w'], V_(S - np.array([g['eye_w'] / 2, 0, 0])), cq.Vector(1, 0, 0))
    s = s.fuse(cq.Solid.makeCylinder(g['shaft_r'], g['cup_from_front_eye'], V_(S), V_(u)))
    s = s.fuse(cq.Solid.makeCylinder(g['spring_r'], Ls - g['cup_from_front_eye'] - 5.0, V_(S + g['cup_from_front_eye'] * u), V_(u)))
    return s.clean()


def carrier_solids(k, r):
    """The carrier as modelled for the clearance checks (carrier frame turned by Rg about PC). No lower yoke: the beam's yaw pin is cantilevered
    from the upper fork arm (single shear), with a low clip under the bar. Sections from CARRIER_SECTIONS."""
    import cadquery as cq
    p = r['pose']; Rg = p['Rg']; PC = k.PC
    D_ = bk.BEAM_DESIGN
    X0 = np.array([1.0, 0, 0]); N0 = np.array(D_['beam_pin_axis']); M0 = np.cross(N0, X0)
    eX, eM, eN = Rg @ X0, Rg @ M0, Rg @ N0
    cs = CARRIER_SECTIONS
    out = {}
    # hub on the carrier pin
    out['hub'] = cq.Solid.makeCylinder(cs['hub_r'], cs['hub_len'], cq.Vector(-cs['hub_len'] / 2, PC[1], PC[2]), cq.Vector(1, 0, 0))
    def bar_between(a, b, w, h, off=0.0):
        u = (b - a) / np.linalg.norm(b - a); v = np.cross(X0, u); v /= np.linalg.norm(v)
        if v[2] < 0:
            v = -v                                              # v points up
        return _obox((a + b) / 2 + off * v, X0, u, v, w / 2, np.linalg.norm(b - a) / 2, h / 2)
    yoke = p['C'] + D_['yaw_arm']['n_off'] * eN
    out['arm to yoke'] = bar_between(PC, yoke + D_['yaw_arm']['n_half'] * eN, *cs['arm_c'])   # lands on top of the upper yoke
    uS = (p['S'] - PC) / np.linalg.norm(p['S'] - PC)
    out['arm to S'] = bar_between(PC, p['S'] - cs['s_fork_r'] * 1.25 * uS, cs['arm_s'][0], cs['arm_s'][1])   # ends at the S fork's root (the fork straddles the eye)
    out['S fork'] = cq.Solid.makeCylinder(cs['s_fork_r'], 2 * cs['s_fork_half'], cq.Vector(-cs['s_fork_half'], p['S'][1], p['S'][2]), cq.Vector(1, 0, 0))
    tp = D_['tail_pin']
    tail = PC + Rg @ (np.array(D_['heave_stop_tail']) - PC)
    ey, ez = Rg @ np.array([0, 1.0, 0]), Rg @ np.array([0, 0, 1.0])
    # boss: a cylinder about the pin (wall_top all round, so its top never rises above pin + d/2 + wall_top as the carrier turns) plus a block under
    # the pin (side walls wall_side, bottom wall wall_bottom: the bump tear-out section)
    rcyl = tp.get('bore_d', tp['d']) / 2 + tp['wall_top']
    cyl = cq.Solid.makeCylinder(rcyl, tp['boss_len'], cq.Vector(-tp['boss_len'] / 2, tail[1], tail[2]), cq.Vector(1, 0, 0))
    # block under the pin: full depth (d/2 + wall_bottom) under the bore (|dy| <= d/2, the tear-out planes), its rear-bottom corner chamfered up to
    # the side wall so it stays clear of the shock's spring cup behind S
    ws, db = tp['d'] / 2 + tp['wall_side'], tp['d'] / 2 + tp['wall_bottom']
    poly = [(-ws, 0.0), (tp['d'] / 2, 0.0), (tp['d'] / 2, -db), (-ws, -db)]      # front side wall ws, rear face flush with the bore (spring cup behind)
    pts3 = lambda dy, dz: tail + dy * ey + dz * ez
    wp = [cq.Vector(-tp['boss_len'] / 2, *map(float, pts3(a, b)[1:])) for a, b in poly]
    blk = cq.Workplane('YZ').workplane(offset=-tp['boss_len'] / 2).polyline([(v.y, v.z) for v in wp]).close().extrude(tp['boss_len']).val()
    out['tail boss'] = cyl.fuse(blk).clean()
    out['tail to hub'] = bar_between(PC, tail, cs['arm_s'][0], cs['arm_s'][1])
    ya = D_['yaw_arm']
    out['yaw pin boss'] = cq.Solid.makeCylinder(cs['pin_boss_r'], cs['pin_boss_up'], cq.Vector(*map(float, yoke + ya['n_half'] * eN)), cq.Vector(*map(float, eN)))
    out['yaw pin clip'] = cq.Solid.makeCylinder(cs['clip_r'], cs['clip_h'], cq.Vector(*map(float, p['C'] - bk.BEAM_R * eN)), cq.Vector(*map(float, -eN)))
    return out


def section_stress(F, P, centroid, normal, b, h):
    """Hand stress (MPa) on a b x h rectangle section with the given normal (unit axis 0/1/2) for force F at point P: bending about both axes
    + axial, torsion (Roark alpha) + 1.5 x shear; von Mises."""
    F, P, c = np.asarray(F, float), np.asarray(P, float), np.asarray(centroid, float)
    M = np.cross(P - c, F)
    ax = [i for i in range(3) if i != normal]           # in-plane axes (b along ax[0], h along ax[1])
    N, T = F[normal], M[normal]
    V = math.hypot(F[ax[0]], F[ax[1]])
    A = b * h
    sig = abs(M[ax[0]]) / (b * h * h / 6) + abs(M[ax[1]]) / (h * b * b / 6) + abs(N) / A
    lo, hi = min(b, h), max(b, h); q = hi / lo
    alpha = float(np.interp(q, [1, 1.5, 2, 3, 4, 6, 10], [0.208, 0.231, 0.246, 0.267, 0.282, 0.299, 0.312]))
    tau = abs(T) / (alpha * lo * lo * hi) + 1.5 * V / A
    return math.sqrt(sig ** 2 + 3 * tau ** 2), sig, tau


def droop_geometry(d):
    td = d['droop_travel']
    a = blade_angle(td)
    er, en = frame(a)
    # blade front face line at droop: find radius where it reaches each z of the pad
    def r_at_z(z):
        lo, hi = 0.0, sa.LEVER_R
        for _ in range(60):
            m = (lo + hi) / 2
            if face_point(td, m, -1)[1] < z:
                lo = m
            else:
                hi = m
        return (lo + hi) / 2
    z0, z1 = d['droop_pad_z']
    ra, rb = r_at_z(z0), r_at_z(z1)
    fa, fb = face_point(td, ra, -1), face_point(td, rb, -1)
    ly = d['lug_rear_y']
    u = (fb - fa) / np.linalg.norm(fb - fa)
    n = np.array([u[1], -u[0]])
    if n @ (-en) < 0:
        n = -n
    # chamfer plane of the lug: the contact line moved tmin into the lug; E on z0, D on the lug's rear face
    T = d['droop_pad_tmin']
    c0 = fa + T * n
    E = c0 + u * (z0 - c0[1]) / u[1]
    D_ = c0 + u * (ly - c0[0]) / u[0]
    if D_[1] <= z0:            # the lug's rear face is everywhere >= T from the face: no chamfer needed
        pad = orient_cw([np.array([ly, z0]), fa, fb, np.array([ly, z1])])
        lug = orient_cw([np.array([LUG['y'][0], z0]), np.array([ly, z0]), np.array([ly, z1]), np.array([LUG['y'][0], z1])])
        D_ = E = np.array([ly, z0])
    else:
        pad = orient_cw([E, fa, fb, np.array([ly, z1]), D_])
        lug = orient_cw([np.array([LUG['y'][0], z0]), E, D_, np.array([ly, z1]), np.array([LUG['y'][0], z1])])
    clevis = orient_cw([np.array([LUG['y'][0], z1]), np.array([ly, z1]), np.array([ly, LUG['clevis_z_top']]), np.array([LUG['y'][0], LUG['clevis_z_top']])])
    return {'a': a, 'fa': fa, 'fb': fb, 'ra': ra, 'rb': rb, 'pad': pad, 'n': n, 'rc': (ra + rb) / 2, 'len': np.linalg.norm(fb - fa),
            'thick': (T, (fb - np.array([ly, z1])) @ (-n)), 'E': E, 'D': D_, 'lug': lug, 'clevis': clevis}


def blade_solid(t):
    """The lever blade as a CadQuery solid at travel t (as parts/swingarm_front.py lever_ref: tapered blade + root and top discs; stud boss left out)."""
    import cadquery as cq
    a = blade_angle(t)
    er, en = frame(a)
    P = np.array([PY, PZ]); T = P + sa.LEVER_R * er
    pts = [tuple(P + H_ROOT * en), tuple(T + H_TOP * en), tuple(T - H_TOP * en), tuple(P - H_ROOT * en)]
    x0, w = BLADE_X[0], BLADE_X[1] - BLADE_X[0]
    s = cq.Workplane('YZ').workplane(offset=x0).polyline(pts).close().extrude(w).val()
    for c, R in ((P, H_ROOT), (T, H_TOP)):
        s = s.fuse(cq.Solid.makeCylinder(R, w, cq.Vector(x0, c[0], c[1]), cq.Vector(1, 0, 0)))
    return s.clean()


def _prism(poly, x0, x1):
    import cadquery as cq
    return cq.Workplane('YZ').workplane(offset=x0).polyline([tuple(map(float, p)) for p in poly]).close().extrude(x1 - x0).val()


def lug_solids(d, dg):
    """Chamfered lug box, clevis (floor + two plates, same y band, the floor inside the lug height) and the TPU wedge, as CadQuery solids."""
    import cadquery as cq
    (x0, x1), (y0, y1), (z0, z1) = LUG['x'], LUG['y'], LUG['z']
    lug = cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))
    cut = _prism([np.array([y1 + 1.0, z0 - 1.0]), np.array([dg['E'][0], z0 - 1.0]), dg['E'], dg['D'],
                  np.array([y1 + 1.0, dg['D'][1]])], x0 - 1.0, x1 + 1.0)
    lug = lug.cut(cut).clean()
    cx0, cx1 = LUG['clevis_x']; pt = LUG['plate_t']; hz = sa.UPLINK_IN[2]
    clev = cq.Solid.makeBox(cx1 - cx0, y1 - y0, 3.8, cq.Vector(cx0, y0, hz - 12.0))
    for px in (cx0, cx1 - pt):
        clev = clev.fuse(cq.Solid.makeBox(pt, y1 - y0, 17.0, cq.Vector(px, y0, hz - 12.0)))
    wedge = _prism(dg['pad'], d['droop_pad_x'][0], d['droop_pad_x'][1])
    return lug, clev.clean(), wedge


def moment(t, wheel_N):
    """Moment of a vertical wheel force at the tip about the pivot axis (N mm) at travel t."""
    return wheel_N * sa.ARM_L * math.cos(sa._REST + sa.arm_swing(t))


def box_pts(lo, hi, step=1.0):
    """Points on the surface of an axis-aligned box."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    ax = [np.linspace(lo[i], hi[i], max(2, int(math.ceil((hi[i] - lo[i]) / step)) + 1)) for i in range(3)]
    g = np.array(np.meshgrid(*ax, indexing='ij')).reshape(3, -1).T
    on = np.any(np.isclose(g, lo) | np.isclose(g, hi), axis=1)
    return g[on]


def fork_surface_points(k, r, step=1.5):
    """Surface samples of both beam-end forks (oriented boxes FORK half-size) at a beam_kin state."""
    p = r['pose']
    out = []
    hx, hm, hn = bk.FORK
    for B in (p['Bp'], p['Bm']):
        for a in np.arange(-hx, hx + 1e-9, step):
            for b in np.arange(-hm, hm + 1e-9, step):
                for c in np.arange(-hn, hn + 1e-9, step):
                    if abs(abs(a) - hx) < 1e-6 or abs(abs(b) - hm) < 1e-6 or abs(abs(c) - hn) < 1e-6:
                        out.append(B + a * p['beam_dir'] + b * p['M'] + c * p['N'])
    return np.array(out)


def aabb_dist(pts, lo, hi):
    lo, hi = np.asarray(lo), np.asarray(hi)
    d = np.maximum(np.maximum(lo - pts, pts - hi), 0)
    return np.sqrt((d ** 2).sum(axis=1)).min()


def lean_of(tp, tm):
    """Body lean (deg) whose lean pair has the same wheel-travel difference as (tp, tm); heave = what is left."""
    lo, hi = -25.0, 25.0
    f = lambda ph: sa.travel_for_lean(ph, 1) - sa.travel_for_lean(ph, -1) - (tp - tm)
    for _ in range(60):
        m = (lo + hi) / 2
        lo, hi = (m, hi) if f(m) < 0 else (lo, m)
    ph = (lo + hi) / 2
    return ph, (tp + tm) / 2 - (sa.travel_for_lean(ph, 1) + sa.travel_for_lean(ph, -1)) / 2


def region(d, k):
    """Vertices of the reachable (t+x, t-x) region of the carrier stops, beam yaw stops and the per-arm droop stops (no per-arm bump stop)."""
    gb, gd = k.state(sa.BUMP, sa.BUMP)['g'], k.state(-sa.DROOP, -sa.DROOP)['g']
    pm = max(abs(k.state(*bk.lean_pair(10, h))['psi']) for h in (-sa.DROOP, 0, sa.BUMP))
    td = d['droop_travel']
    memo = {}
    def G(t):
        # warm start from the nearest pose solved so far (Newton steps are small), else march from rest
        key = (round(float(t[0]), 4), round(float(t[1]), 4))
        if key in memo:
            return memo[key]
        near = min(memo.items(), key=lambda kv: abs(kv[0][0] - t[0]) + abs(kv[0][1] - t[1]), default=None)
        if near is not None and abs(near[0][0] - t[0]) + abs(near[0][1] - t[1]) < 3.0:
            r = k.state(float(t[0]), float(t[1]), q0=near[1]['q'])
        else:
            r = k.state(float(t[0]), float(t[1]))
        memo[key] = r
        return r
    cons = {'carrier bump': lambda r, t: r['g'] - gb, 'carrier droop': lambda r, t: r['g'] - gd,
            'yaw +': lambda r, t: r['psi'] - pm, 'yaw -': lambda r, t: r['psi'] + pm,
            'droop +x': lambda r, t: t[0] - td, 'droop -x': lambda r, t: t[1] - td}
    starts = [(40, -15), (-15, 40), (15, -33), (-33, 15), (12, 12), (-8, -8), (25, -25), (-25, 25), (0, -33), (-33, 0)]
    verts = []
    names = list(cons)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            c1, c2 = cons[names[i]], cons[names[j]]
            for st in starts:
                t = np.array(st, float)
                ok = False
                for _ in range(25):
                    try:
                        r = G(t)
                    except RuntimeError:
                        break
                    f = np.array([c1(r, t), c2(r, t)])
                    if np.max(np.abs(f)) < 1e-5:
                        ok = True; break
                    J = np.empty((2, 2))
                    for q in range(2):
                        dt = np.zeros(2); dt[q] = 0.05
                        r2 = G(t + dt); J[:, q] = (np.array([c1(r2, t + dt), c2(r2, t + dt)]) - f) / 0.05
                    try:
                        t = t + np.clip(np.linalg.solve(J, -f), -6, 6)
                    except np.linalg.LinAlgError:
                        break
                if not ok:
                    continue
                r = G(t)
                feas = (gd - 1e-4 <= r['g'] <= gb + 1e-4 and abs(r['psi']) <= pm + 1e-4 and t[0] >= td - 1e-4 and t[1] >= td - 1e-4)
                if feas and not any(np.allclose(t, v[0], atol=0.05) for v in verts):
                    verts.append((t.copy(), names[i] + ' + ' + names[j], r))
    return verts, gb, gd, pm


def stop_forces(d, k, tp, tm, wheel):
    """Forces on the carrier bump pad (tail 10 mm behind PC) and on one yaw pad (|x| 20) for a vertical wheel load at the tip of each loaded
    wheel. wheel = (N on +x, N on -x). Spring force ignored (conservative: the stops take it all)."""
    r = k.state(tp, tm); p = r['pose']
    tau_c, tau_y = 0.0, 0.0
    for s_, t_, W, B in ((1, tp, wheel[0], p['Bp']), (-1, tm, wheel[1], p['Bm'])):
        if W == 0:
            continue
        Marm = W * sa.ARM_L * math.cos(sa._REST + sa.arm_swing(t_))
        A_ = k.lever_ball(t_, s_); piv = np.array([s_ * sa.LEVER_BALL_X, *sa.PIVOT])
        arm = abs(np.cross(A_ - piv, p['M'])[0])
        F = (Marm / arm) * p['M']                      # slot force from the lever ball on the beam end
        tau_c += np.cross(A_ - k.PC, F)[0]
        tau_y += np.cross(A_ - p['C'], F) @ p['N']
    tail = np.linalg.norm(np.array(bk.BEAM_DESIGN['heave_stop_tail']) - k.PC)
    stop_forces.last_Fb = Marm / arm if any(wheel) else 0.0
    return abs(tau_c) / tail, abs(tau_y) / (2 * bk.BEAM_DESIGN['yaw_stop_r']), r       # yaw: a pair of pads shares the torque


def main():
    d = STOP_DESIGN
    dg = droop_geometry(d)
    Md = moment(d['droop_travel'], d['droop_wheel_N'])
    Fd = Md / dg['rc']
    k = bk.Kin(bk.BEAM_DESIGN)
    print('ARM STOPS (tools/arm_stops.py), +x side (mirror for -x). Pivot axis y %.1f z %.1f, blade x %.1f .. %.1f' % (PY, PZ, *BLADE_X))
    print('NO per-arm bump stop (coordinator, round 2): bump travel ends on the carrier and yaw stops of tools/beam_kin.py.')
    # ---- reachable region ----
    verts, gb, gd, pm = region(d, k)
    print('REACHABLE REGION with the carrier stops (%+.2f / %+.2f deg), yaw stops (+-%.2f deg) and the per-arm droop stops (%+.1f):' % (gb, gd, pm, d['droop_travel']))
    print('  %-30s %8s %8s %8s %8s %8s %8s %8s %7s' % ('vertex (stops engaged)', 't+x', 't-x', 'lean', 'heave', 'shock', 'box', 'rail', 'cond'))
    vs = sorted(verts, key=lambda v: math.atan2(v[0][1] - 5, v[0][0] - 5))
    for t, nm, r in vs:
        ph, hv = lean_of(*t)
        print('  %-30s %+8.2f %+8.2f %+8.2f %+8.2f %8.2f %+8.1f %+8.1f %7.3f' % (nm, *t, ph, hv, r['shock'], r['keepout'], r['beam_over_rail'], r['cond']))
    tmax = max(max(v[0]) for v in verts)
    print('  largest one-wheel bump: %+.2f mm (the layout joints were sized for +37.9: the design thread must re-check the arm, upper link, tie rod and'
          ' tyre at this travel); lowest: %+.2f' % (tmax, min(min(v[0]) for v in verts)))
    print('  POSE LIST for the design thread\'s pose check (both sides mirror): ' + '; '.join('(%+.1f, %+.1f)' % tuple(v[0]) for v in vs))
    # ---- forces on the stops ----
    W = d['bump_wheel_N']
    print('STOP LOADS (65 N vertical at the tip of a landing wheel, analyses/front_arm.py; heave spring ignored, the stops take it all):')
    worst = None
    for t, nm, r in vs:
        if 'carrier bump' in nm or 'yaw' in nm:
            fc, fy, _ = stop_forces(d, k, t[0], t[1], (W if t[0] >= t[1] else 0.0, W if t[1] > t[0] else 0.0))
            print('  one-wheel landing at vertex %-30s (%+.1f, %+.1f): carrier bump pad %.0f N, yaw pad %.0f N' % (nm, *t, fc, fy))
            if worst is None or max(fc, fy) > max(worst[0], worst[1]):
                worst = (fc, fy, nm, t)
    fc2, fy2, _ = stop_forces(d, k, sa.BUMP, sa.BUMP, (W, W))
    print('  two-wheel landing at the carrier bump stop (+12, +12): carrier bump pad %.0f N, yaw pads %.0f N' % (fc2, fy2))
    FC = max(worst[0], fc2); FY = worst[1]
    # ---- sizing (stop sizing round 1): sections, loads, stresses, safety ----
    D_ = bk.BEAM_DESIGN
    tp_ = D_['tail_pin']; ya, yp = D_['yaw_arm'], D_['yaw_post']
    stop_forces(d, k, worst[3][0], worst[3][1], (W, 0.0) if worst[3][0] >= worst[3][1] else (0.0, W)); Fb1 = stop_forces.last_Fb
    stop_forces(d, k, sa.BUMP, sa.BUMP, (W, W)); Fb2 = stop_forces.last_Fb
    ry = D_['yaw_stop_r']
    rows = []
    def row(part, sec, load, num, den, allow):
        st = num / den
        rows.append((part, sec, load, st, allow / st))
    Zc = 2 * bk.BEAM_R * (2 * bk.BAR_HM) ** 2 / 6
    row('beam bar at C (two-wheel)', '%.0f x %.0f mm (N x M)' % (2 * bk.BEAM_R, 2 * bk.BAR_HM), '%.0f N at %.0f mm' % (Fb2, sa.LEVER_BALL_X),
        Fb2 * sa.LEVER_BALL_X, Zc, PETG_ALLOW)
    row('beam bar at C (one-wheel)', '%.0f x %.0f mm' % (2 * bk.BEAM_R, 2 * bk.BAR_HM), '%.0f N ball, %.0f N pad' % (Fb1, FY),
        abs(Fb1 * sa.LEVER_BALL_X - FY * ry), Zc, PETG_ALLOW)
    Ze = 2 * bk.BEAM_R * (2 * bk.bar_hm(ry)) ** 2 / 6          # section depth at the pad (centroid offset does not change Z of a rectangle)
    row('beam bar at |x| %.0f (one-wheel)' % ry, '%.0f x %.1f mm' % (2 * bk.BEAM_R, 2 * bk.bar_hm(ry)), '%.0f N at %.0f mm' % (Fb1, sa.LEVER_BALL_X - ry),
        Fb1 * (sa.LEVER_BALL_X - ry), Ze, PETG_ALLOW)
    tpl = bk.FORK[1] - 3.5
    row('fork plate at the ball', '%.0f x %.1f mm, 12 fixed-fixed' % (2 * bk.FORK[2], tpl), '%.0f N' % Fb1, Fb1 * 12.0 / 8.0, 2 * bk.FORK[2] * tpl ** 2 / 6,
        PETG_ALLOW)
    row('puck on the slot face', 'D9 puck face', '%.0f N' % Fb1, Fb1, math.pi * 4.5 ** 2, PETG_ALLOW)
    post_l = ya['n_off'] - ya['n_half']
    pm_ = yp['m_rear'][1] - yp['m_rear'][0]
    row('yaw pad post', '%.0f x %.0f mm, %.0f mm lever' % (yp['x'][1] - yp['x'][0], pm_, post_l), '%.0f N' % FY,
        FY * post_l, (yp['x'][1] - yp['x'][0]) * pm_ ** 2 / 6, PETG_ALLOW)
    # yaw arm root: bending of the pad force at r plus torsion of its offset along N (pad at the bar mid-plane, arm centre n_off above)
    bN, hM = 2 * ya['n_half'], 2 * ya['m_half']
    sgb = FY * ry / (bN * hM ** 2 / 6)
    alpha = float(np.interp(max(bN, hM) / min(bN, hM), [1, 1.5, 2, 3], [0.208, 0.231, 0.246, 0.267]))
    tau_t = FY * ya['n_off'] / (alpha * min(bN, hM) ** 2 * max(bN, hM)) + 1.5 * FY / (bN * hM)
    vm = math.sqrt(sgb ** 2 + 3 * tau_t ** 2)
    rows.append(('yaw arm root at C (bend + torsion)', '%.0f x %.0f mm (N x M)' % (bN, hM), '%.0f N at %.1f, offset %.1f' % (FY, ry, ya['n_off']), vm, PETG_ALLOW / vm))
    # beam yaw pin, cantilevered from the upper yoke (no lower yoke: it would hit the motor), two-wheel landing: both slot forces on it
    cs = CARRIER_SECTIONS; Fpin = 2 * Fb2; lev = ya['n_off'] - ya['n_half']
    row('yaw pin, steel D%.0f cantilever (350 ASSUMED)' % cs['yaw_pin_d'], 'D%.0f, %.1f mm lever' % (cs['yaw_pin_d'], lev), '%.0f N' % Fpin, Fpin * lev,
        math.pi * cs['yaw_pin_d'] ** 3 / 32, 350.0)
    Lb = 2 * ya['n_half'] + cs['pin_boss_up']
    # boss: bending of the cantilever pin (6 M / (d L^2)) plus its direct bearing (F / (d L)), added
    row('yaw pin boss in the upper yoke (bend + bearing)', 'D%.0f x %.0f mm long' % (cs['yaw_pin_d'], Lb), '%.0f N, %.0f N mm' % (Fpin, Fpin * lev),
        6 * Fpin * lev / (cs['yaw_pin_d'] * Lb ** 2) + Fpin / (cs['yaw_pin_d'] * Lb), 1.0, PETG_ALLOW)
    row('beam bar on the yaw pin (bearing)', 'D%.0f x %.0f mm' % (cs['yaw_pin_d'], 2 * bk.BEAM_R), '%.0f N' % Fpin, Fpin, cs['yaw_pin_d'] * 2 * bk.BEAM_R,
        PETG_ALLOW)
    yoke0 = np.array(D_['beam_centre']) + (ya['n_off'] + ya['n_half']) * np.array(D_['beam_pin_axis'])
    Fvec = Fpin * np.cross(np.array(D_['beam_pin_axis']), np.array([1.0, 0, 0]))
    Mh = abs(np.cross(yoke0 - k.PC, Fvec)[0])
    w_, h_, _ = cs['arm_c']
    row('carrier arm PC to yoke, at the hub', '%.1f x %.0f mm' % (w_, h_), '%.0f N, %.0f N mm' % (Fpin, Mh), Mh, w_ * h_ ** 2 / 6, PETG_ALLOW)
    row('yaw pad TPU', '%.0f x 12 mm face' % (yp['x'][1] - yp['x'][0]), '%.0f N' % FY, FY, (yp['x'][1] - yp['x'][0]) * 12.0, TPU_ALLOW)
    Fc = fc2; Fe = Fc / 2
    L_pad = tp_['pad_x'][1] - tp_['pad_x'][0]
    lever = (tp_['pad_x'][0] + tp_['pad_x'][1]) / 2 - tp_['boss_len'] / 2
    row('tail boss on the pin (bearing)', 'D%.0f bore x %.1f mm' % (tp_['d'], tp_['boss_len']), '%.0f N' % Fc, Fc, tp_['d'] * tp_['boss_len'], PETG_ALLOW)
    SH = PETG_ALLOW / math.sqrt(3)                               # shear allowable on the von Mises basis of the other rows
    row('tail boss tear-out, bump (wall below, shear)', '2 x %.1f x %.1f mm' % (tp_['wall_bottom'], tp_['boss_len']), '%.0f N' % Fc, Fc,
        2 * tp_['wall_bottom'] * tp_['boss_len'], SH)
    Fdroop = Fc * d['droop_wheel_N'] / d['bump_wheel_N']          # wheels in the air, 30 N each (ASSUMED), on the droop saddles
    row('tail boss tear-out, droop (wall above, shear)', '2 x %.1f x %.1f mm' % (tp_['wall_top'], tp_['boss_len']), '%.0f N' % Fdroop, Fdroop,
        2 * tp_['wall_top'] * tp_['boss_len'], SH)
    row('tail boss side walls (net tension)', '(%.1f + %.1f) x %.1f mm' % (tp_['wall_side'], tp_['wall_top'], tp_['boss_len']), '%.0f N' % Fc, Fc,
        (tp_['wall_side'] + tp_['wall_top']) * tp_['boss_len'], PETG_ALLOW)
    row('tail pin (steel, 350 ASSUMED)', 'D%.0f, %.1f mm lever' % (tp_['d'], lever), '%.0f N per end' % Fe, Fe * lever, math.pi * tp_['d'] ** 3 / 32, 350.0)
    row('pin-end TPU saddle', 'D%.0f x %.0f mm projected' % (tp_['d'], L_pad), '%.0f N' % Fe, Fe, tp_['d'] * L_pad, TPU_ALLOW)
    blk_h = 6.0                       # ASSUMED pad block wall under the saddle, supported at both ends (spine side and outer end)
    row('pad block wall under the saddle', '%.0f x %.0f mm, span %.0f' % (L_pad, blk_h, L_pad), '%.0f N' % Fe, Fe * L_pad / 8, L_pad * blk_h ** 2 / 6, PETG_ALLOW)
    print('SIZING (hand calc, FEM later; PETG %g MPa (shear %.1f), TPU %g MPa ASSUMED, steel 350 MPa ASSUMED):' % (PETG_ALLOW, PETG_ALLOW / math.sqrt(3), TPU_ALLOW))
    print('  %-34s %-32s %-22s %8s %7s' % ('part', 'section', 'load', 'MPa', 'safety'))
    for r_ in rows:
        print('  %-34s %-32s %-22s %8.1f %7.1f%s' % (*r_, '' if r_[4] >= 2.5 else '   <-- below 2.5'))
    print('  saddle faces square to the tail radius at contact: bump face normal %.2f deg from vertical (the pin moves along it), droop %.2f deg;'
          ' a horizontal face would raise the loads 3.7 %% (verify-2)' % (gb, -gd))
    print('  INTO THE CAGE (cage owner): pad blocks under the front cross tube take %.0f N up (bump) per end at |x| %.0f..%.0f; on the 8.8 mm cross tube'
          ' alone (rail to spine) that is about %.0f MPa, so the blocks must tie into the spine and the carrier clevis (one printed clevis block),'
          ' or the spine is raised locally' % (Fe, *tp_['pad_x'], Fe * 19 * 26 / 45 / (8.8 ** 3 / 6)))
    k_spring, Wf = 3.0, 15.3
    Fs = Wf / 0.381 + k_spring * 4.68
    print('  heave spring at the bump stop (information only; ASSUMED 2.6 kg, 60 %% front, %.1f N/mm spring): about %.0f N in the shock, ignored above' % (
        k_spring, Fs))
    guide_3d_report(k, [v[0] for v in vs])
    # ---- droop stop (unchanged) ----
    print('DROOP at t %+.1f: blade %.1f deg; TPU wedge on the lug rear face y %.1f over z %.1f .. %.1f; contact face from (y %.2f, z %.2f) to (y %.2f, z %.2f);'
          ' x %.1f .. %.1f; thickness normal to the face %.1f .. %.1f mm' % (d['droop_travel'], D(dg['a']), d['lug_rear_y'], *d['droop_pad_z'], *dg['fa'],
                                                                             *dg['fb'], *d['droop_pad_x'], *dg['thick']))
    print('  lug corner chamfer (cage, full lug width): the lower rear corner (y %.1f, z %.1f) is cut back to a plane %.1f mm behind the contact face, from'
          ' (y %.2f, z %.2f) on the lug bottom to (y %.2f, z %.2f) on its rear face; the wedge fills it' % (d['lug_rear_y'], LUG['z'][0], d['droop_pad_tmin'],
                                                                                                        *dg['E'], *dg['D']))
    print('  force on the wedge (y, z) = (%+.3f, %+.3f): %.1f deg from horizontal; flat-on-flat' % (dg['n'][0], dg['n'][1], D(math.atan2(dg['n'][1], abs(dg['n'][0])))))
    print('  load: %.0f N at the tip (ASSUMED) -> %.0f N mm; contact radius %.1f mm -> %.0f N' % (d['droop_wheel_N'], Md, dg['rc'], Fd))
    lug_arm = (d['droop_pad_x'][0] + d['droop_pad_x'][1]) / 2 - 45.0
    sig = Fd * abs(dg['n'][0]) * lug_arm / (10 * 12 ** 2 / 6) + Fd * abs(dg['n'][1]) * lug_arm / (12 * 10 ** 2 / 6)
    print('  load path: wedge -> rail lug (10 x 12 mm, %.1f mm from the stub bar at x 45) -> stub bar -> front foot post y -140 -> deck. Lug bending %.1f MPa'
          ' (hand calc; safety %.1f)' % (lug_arm, sig, PETG_ALLOW / sig))
    Ad = dg['len'] * (BLADE_X[1] - BLADE_X[0])
    print('  wedge: TPU 95A (ASSUMED) glued to the lug, contact %.0f mm2 -> %.2f MPa' % (Ad, Fd / Ad))
    sq_d = Fd / Ad / TPU_E * dg['thick'][0]
    gap = ', '.join('%+.2f: %.2f' % (d['droop_travel'] + dt, poly_dist(blade_outline(d['droop_travel'] + dt), dg['pad'])) for dt in (0.0, 0.5, 1.0, 2.0, 5.0))
    print('  contact face gap (closes to 0 at contact, by design) at t %s mm' % gap)
    skip = (-1, dg['ra'] - 2.0, dg['rb'] + 2.0)
    ts = np.linspace(d['droop_travel'], tmax, 72)
    print('  wedge to the rest of the blade over the whole travel (to %+.1f): min %.2f mm' % (tmax, min(poly_dist(blade_outline(t, skip=skip), dg['pad']) for t in ts)))
    lug3, clev3, wedge3 = lug_solids(d, dg)
    ts3 = [d['droop_travel'], -32.7, -32.4, -31.9, -30.9, -28.9, -24.9, -16.9, -4.9, 10.0, 25.0, 37.9, tmax]
    rows = [(t, blade_solid(t).distance(lug3), blade_solid(t).distance(clev3), blade_solid(t).distance(wedge3)) for t in ts3]
    print('  3D (BRepExtrema), blade vs lug (chamfered over x %.1f..%.1f), clevis and wedge: ' % tuple(LUG['x']) +
          '; '.join('t %+.1f: %.2f / %.2f / %.2f' % r for r in rows))
    print('  3D minimum: blade to the bare lug %.2f mm, minus the droop TPU squeeze %.2f = %.2f mm; to the clevis %.2f mm (owner rule >= 1.0)' % (
        min(r[1] for r in rows), sq_d, min(r[1] for r in rows) - sq_d, min(r[2] for r in rows)))
    servo_mount_report(d, tmax)


def guide_3d_report(k, extra):
    """Beam bar, +x fork, carrier yaw arm and posts in 3D (BRepExtrema) against the built model and each other, over the poses inside the stops."""
    bodies = step_bodies()
    names = ['Cage', 'b_motor', 'Steer_servo_1', 'mock_receiver', 'Front_shell', 'w3_deck_front', 'w3_deck_rear', 'w3_arm_pivot_block_1']
    gb, gd = k.state(sa.BUMP, sa.BUMP)['g'], k.state(-sa.DROOP, -sa.DROOP)['g']
    pm = max(abs(k.state(*bk.lean_pair(10, h))['psi']) for h in (-sa.DROOP, 0, sa.BUMP))
    poses = [tuple(e) for e in extra]
    for tp in np.linspace(-32.9, bk.T_MAX, 7):
        for tm in np.linspace(-32.9, bk.T_MAX, 7):
            poses.append((tp, tm))
    best = {n: (1e9, None) for n in ['fork to posts', 'bar to posts (2 = on the 2 mm TPU)', 'arm to bar', 'arm to fork']}
    n = 0
    for tp, tm in poses:
        r = k.state(tp, tm)
        if not (gd - 1e-4 <= r['g'] <= gb + 1e-4 and abs(r['psi']) <= pm + 1e-4):
            continue
        n += 1
        fork, bar, arm, posts = beam_solids(r)
        guide = bar.fuse(arm)
        for p_ in posts:
            guide = guide.fuse(p_)
        for nm in names:
            for part, sol in (('bar+arm+posts', guide), ('fork', fork)):
                key = '%s to %s' % (part, nm)
                dd = dist3(sol, bodies[nm])
                if dd < 1e-6:
                    dd = -overlap3(sol, bodies[nm])
                if key not in best or dd < best[key][0]:
                    best[key] = (dd, (round(tp, 1), round(tm, 1)))
        cs_ = carrier_solids(k, r)
        shock = shock_solids(k, r)
        import cadquery as cq
        for sx_ in (1, -1):
            x0p, x1p = (bk.CLEVIS_PLATES_X if sx_ > 0 else (-bk.CLEVIS_PLATES_X[1], -bk.CLEVIS_PLATES_X[0]))
            plate = _box((x0p, -56.6, 89.9, x1p, -33.6, bk.SPINE_UNDER))
            for nm2, sol2 in (('heave shock', shock), ('beam bar', bar), ('beam fork', fork)):
                key = 'new clevis plate to the %s' % nm2
                dd = plate.distance(sol2)
                if key not in best or dd < best[key][0]:
                    best[key] = (dd, (round(tp, 1), round(tm, 1)))
            for cn2 in ('hub', 'arm to yoke', 'arm to S', 'S fork', 'tail to hub', 'tail boss'):
                key = 'new clevis plate to the carrier %s' % cn2
                dd = plate.distance(carrier_solids(k, r)[cn2]) if cn2 != 'hub' else plate.distance(carrier_solids(k, r)[cn2])
                if key not in best or dd < best[key][0]:
                    best[key] = (dd, (round(tp, 1), round(tm, 1)))
        for cn, csol in cs_.items():
            if cn not in ('S fork',):
                key = 'carrier %s to the heave shock' % cn
                dd = csol.distance(shock)
                if key not in best or dd < best[key][0]:
                    best[key] = (dd, (round(tp, 1), round(tm, 1)))
            for nm in names:
                key = 'carrier %s to %s' % (cn, nm)
                dd = dist3(csol, bodies[nm])
                if dd < 1e-6:
                    dd = -overlap3(csol, bodies[nm])
                if key not in best or dd < best[key][0]:
                    best[key] = (dd, (round(tp, 1), round(tm, 1)))
            for nm2, sol2 in (('bar', bar), ('fork', fork)):
                if cn in ('yaw pin clip',):
                    continue
                key = 'carrier %s to the beam %s' % (cn, nm2)
                dd = csol.distance(sol2)
                if key not in best or dd < best[key][0]:
                    best[key] = (dd, (round(tp, 1), round(tm, 1)))
        for nm, a_, b_ in (('fork to posts', fork, posts), ('bar to posts (2 = on the 2 mm TPU)', bar, posts), ('arm to bar', arm, [bar]), ('arm to fork', arm, [fork])):
            dd = min(a_.distance(q) for q in b_)
            if dd < best[nm][0]:
                best[nm] = (dd, (round(tp, 1), round(tm, 1)))
    print('BEAM GUIDE 3D (bar 12 wide x 22..13 deep, fork, yaw arm and posts) against the BUILT model and itself, %d poses inside the stops:' % n)
    for nm, (dd, at) in best.items():
        print('  %-34s %7.2f mm at %s%s' % (nm, dd, at, '' if dd >= 0.995 or nm.startswith('bar to posts') else '   <-- below 1.0'))
    print('  (negative = overlap in mm3)')


def servo_mount_report(d, tmax):
    """Servo mount (wedge floor + clip): geometry, rules, and every clearance in 3D (BRepExtrema) against the BUILT model."""
    import cadquery as cq
    sm = d['SERVO_MOUNT']
    m1, m2, top, bot = mount_solids(d)
    bb1, bb2 = m1.BoundingBox(), m2.BoundingBox()
    print('SERVO MOUNT (holds the servo only; checked against the built %s):' % STEP_PATH)
    print('  deck (w3_deck_front, measured at x 43): top z %.2f at y -108.6 .. %.2f at y -67.8, underside %.2f .. %.2f; edge x %.1f' % (
        top(-108.6), top(-67.8), bot(-108.6), bot(-67.8), sm['deck_edge_x']))
    print('  part 1, wedge floor plate (PETG, printed on its flat top face): x %.1f..%.1f, y %.1f..%.2f under the servo, top z 28.2, %.2f..%.2f mm thick; it ends'
          ' where it would get thinner than %.1f mm (the servo\'s last %.1f mm stand over a %.1f..%.1f mm gap); tongue %s under the clip; front rim %s;'
          ' bbox z %.2f..%.2f, %.1f cm3' % (*sm['floor_x'], *sm['floor_y'], *sm['floor_t'], sm['floor_tmin'], -67.8 - sm['floor_y'][1],
                                           28.2 - top(sm['floor_y'][1]), 28.2 - top(-67.8), sm['tongue'], sm['rim'], bb1.zmin, bb1.zmax, m1.Volume() / 1000))
    print('  part 2, clip (PETG): leg %s on the tongue, arm %s over the servo\'s inboard edge, strap %s across the servo top under the rail; bbox x %.1f..%.1f'
          ' z %.1f..%.1f' % (sm['leg'], sm['arm'], sm['strap'], bb2.xmin, bb2.xmax, bb2.zmin, bb2.zmax))
    print('  WHY two parts: the floor must lie under the servo while the servo slides in and out; the clip is the only part that has to come off')
    ix, iy = sm['inserts'][0]
    lg = sm['leg']
    print('  insert (%.1f, %.1f): to the leg edges %.1f / %.1f / %.1f / %.1f mm (rule %.0f); deck hole to the deck edges x %.1f, y %.1f mm; off the lap'
          ' joint (y -62.3..-61.9); screw M3 x 16 from below through the deck (6 mm) and the tongue (%.1f mm)' % (
              ix, iy, ix - lg[0], lg[2] - ix, iy - lg[1], lg[3] - iy, sm['edge_rule'], sm['deck_edge_x'] - ix, -61.9 - iy, 28.2 - top(iy)))
    print('  servo: located by the front rim and the floor, held down by the clip and its strap on %.1f mm EVA (ASSUMED). Out: horn off, one screw out from'
          ' below, clip off, servo slides 23.5 mm inboard over the tongue (1 mm short of the receiver); lifting it out from there needs the beam assembly'
          ' off (18.6 mm under the beam bar for a 20.2 mm servo)' % sm['eva_t'])
    print('CLEARANCE 3D (BRepExtrema) against the BUILT model (owner rule >= 1.0 mm):')
    bodies = step_bodies()
    by_design = {'w3_deck_front': 'floor and tongue lie on the deck', 'Steer_servo_1': 'servo on the floor, clip and strap on 0.5 mm EVA'}
    print('  %-24s %9s %9s   %s' % ('body', 'part 1', 'part 2', 'note'))
    for nm in STATIC:
        if nm not in bodies:
            print('  %-24s not in the STEP' % nm); continue
        row = []
        for q in (m1, m2):
            dd = dist3(q, bodies[nm]); row.append(dd if dd > 1e-6 else -overlap3(q, bodies[nm]))
        note = by_design.get(nm, '')
        print('  %-24s %9.2f %9.2f   %s%s' % (nm, *row, note, '' if min(row) >= 1.0 or note else '  <-- below 1.0'))
    print('  (negative = overlap volume in mm3); lowest point z %.2f: on the deck top' % min(bb1.zmin, bb2.zmin))
    print('CLEARANCE 3D to the moving parts (swept):')
    sw = SURROUND['horn']['swing_deg']
    hm, dm = [1e9, 1e9], [1e9, 1e9]
    for eta in np.linspace(-sw, sw, 7):
        hs, ball = horn_solids(eta)
        dl = capsule(ball, SURROUND['drag_link']['b'], SURROUND['drag_link']['r'])
        for i, q in enumerate((m1, m2)):
            hm[i] = min(hm[i], q.distance(hs)); dm[i] = min(dm[i], q.distance(dl))
    print('  %-56s part 1 %6.2f  part 2 %6.2f' % ('horn plate + ball stud, servo -%g..+%g deg' % (sw, sw), *hm))
    print('  %-56s part 1 %6.2f  part 2 %6.2f' % ('drag link (r 4.0 capsule), servo -%g..+%g deg' % (sw, sw), *dm))
    bl, lc = [1e9, 1e9], [1e9, 1e9]
    for t in (d['droop_travel'], -20.0, -5.0, 10.0, 25.0, 37.9, tmax):
        b3, lt = blade_solid(t), lever_top_solids(t)
        for i, q in enumerate((m1, m2)):
            bl[i] = min(bl[i], q.distance(b3)); lc[i] = min(lc[i], q.distance(lt))
    print('  %-56s part 1 %6.2f  part 2 %6.2f' % ('blade, travel %+.1f..%+.1f' % (d['droop_travel'], tmax), *bl))
    print('  %-56s part 1 %6.2f  part 2 %6.2f' % ('lever ball cup + stud boss, same travel', *lc))
    k = bk.Kin(bk.BEAM_DESIGN)
    gb, gd = k.state(sa.BUMP, sa.BUMP)['g'], k.state(-sa.DROOP, -sa.DROOP)['g']
    pm = max(abs(k.state(*bk.lean_pair(10, h))['psi']) for h in (-sa.DROOP, 0, sa.BUMP))
    fk, br, n = [1e9, 1e9], [1e9, 1e9], 0
    for tp in np.linspace(-32.9, tmax, 13):
        for tm in np.linspace(-32.9, tmax, 13):
            r = k.state(tp, tm)
            if not (gd - 1e-6 <= r['g'] <= gb + 1e-6 and abs(r['psi']) <= pm + 1e-6):
                continue
            n += 1
            f3, b3, _, _ = beam_solids(r)
            for i, q in enumerate((m1, m2)):
                fk[i] = min(fk[i], q.distance(f3)); br[i] = min(br[i], q.distance(b3))
    print('  %-56s part 1 %6.2f  part 2 %6.2f' % ('beam-end fork (FORK box, %d poses inside the beam stops)' % n, *fk))
    print('  %-56s part 1 %6.2f  part 2 %6.2f' % ('beam bar', *br))


if __name__ == '__main__':
    main()
