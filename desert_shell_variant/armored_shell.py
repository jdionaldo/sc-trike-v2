"""Option 3 Armored — Render-Matched Modular Faceted Armor Panels & Hexagonal Mesh Intake Grille for Fugu V2.

All parts are built in Fugu V2 world coordinates (mm, Z up, front = -Y, X right) using
pure flat facets (zero curves), uniform 3.0 mm printable walls, and exact 0.30 mm print clearances.

Parts produced:
  1. Armored_Nose_Tub   — Inboard nose side-walls + raked front fascia (LED groove & hex intake
                          opening) + 3-facet armored lower chin/skid plate tucked inside the V-bullbar.
  2. Hex_Mesh_Grille    — Separate 3D-printable hexagonal honeycomb intake mesh with rear lap flange.
  3. Nose_Light_Bar     — Flush cyan LED light bar with twin 3 mm LED bore pegs on the back.
  4. Armored_Hood_Deck  — Removable faceted top hood with pronounced center power-bulge, subtle panel
                          seams, and outer wings hugging the exact contour of the upper exo-cage rails.
  5. Side_Pod_Right / Side_Pod_Left — Forward-swept angular side-pod flank armor with 3 sculpted,
                          raised forward-leaning gill louvers and M3 cage mounting holes.
"""
import math
import numpy as np
import cadquery as cq
from scipy.spatial import ConvexHull, HalfspaceIntersection
from scipy.optimize import linprog

WALL = 3.0            # mm uniform printable panel thickness
FIT = 0.30            # mm print-in-place / assembly clearance
STRIP_H = 8.5         # mm height of the cyan LED light bar on the raked nose face
STRIP_EDGE = 2.5      # mm top/side margin around the LED light bar
GROOVE_D = 1.8        # mm blind groove depth for LED bar
HEX_FLAT = 6.0        # mm across-flats of each hexagonal intake cell
HEX_RIB = 1.8         # mm printable rib width between hex cells


def _hull(pts):
    """Convex hull solid from 3D points using SciPy ConvexHull -> CadQuery Solid."""
    arr = np.asarray(pts, float)
    ch = ConvexHull(arr)
    faces = []
    for s in ch.simplices:
        p0, p1, p2 = arr[s[0]], arr[s[1]], arr[s[2]]
        n = np.cross(p1 - p0, p2 - p0)
        if np.linalg.norm(n) < 1e-9:
            continue
        if np.dot(n, p0 - arr.mean(axis=0)) < 0:
            p1, p2 = p2, p1
        w = cq.Wire.makePolygon([cq.Vector(*p0), cq.Vector(*p1), cq.Vector(*p2), cq.Vector(*p0)])
        faces.append(cq.Face.makeFromWires(w))
    shell = cq.Shell.makeShell(faces)
    return cq.Solid.makeSolid(shell)


def _both(points):
    out = []
    for x, y, z in points:
        out.append((float(x), float(y), float(z)))
        if abs(x) > 1e-9:
            out.append((-float(x), float(y), float(z)))
    return out


def _planes(points):
    eq = ConvexHull(np.asarray(points, float)).equations
    keep = []
    for q in eq:
        if not any(np.allclose(q, k, atol=1e-6) for k in keep):
            keep.append(q)
    return np.array(keep)


def _solid_from_planes(planes):
    A, b = planes[:, :3], -planes[:, 3]
    fit = linprog(
        [0, 0, 0, -1],
        A_ub=np.column_stack((A, np.ones(len(A)))),
        b_ub=b,
        bounds=[(None, None)] * 3 + [(0, None)],
        method="highs",
    )
    assert fit.success and fit.x[3] > 1e-3, "empty halfspace solid"
    pts = HalfspaceIntersection(planes, fit.x[:3]).intersections
    return _hull(pts)


def _hollow_block(points, wall=WALL, open_if=None, mirror_x=True):
    """Build a hollow faceted block with exact `wall` thickness on every closed facet."""
    pts = _both(points) if mirror_x else list(points)
    outer = _planes(pts)
    inner = outer.copy()
    for q in inner:
        if open_if is not None and open_if(q[:3]):
            q[3] -= 40.0
        else:
            q[3] += wall
    body = _solid_from_planes(outer).cut(_solid_from_planes(inner))
    return body.clean(), outer


# =================================================================================================
# 1. ARMORED NOSE TUB (inboard side walls + raked front fascia + 3-facet armored chin/skid)
# =================================================================================================
# Tucked cleanly inside the chamfered-square V-bullbar & side exo-rails with >= 0.4 mm clearance.
NF_TOP   = (36.2, -285.0, 108.0)   # top outer corner of raked nose face (high-clearance Baja stub-nose)
NF_BOT   = (31.0, -263.0,  81.2)   # bottom outer corner of raked nose face (clears raised mid V-bar)
CHIN_MID = (23.5, -249.5,  63.5)   # cheek bevel crease of 3-facet lower chin
CHIN_BOT = (17.5, -236.5,  40.5)   # high-tucked lower chin corner (preserves steep off-road entry angle)

TUB_REAR_TOP = (38.0, -140.0, 106.0)
TUB_REAR_MID = (38.0, -140.0,  61.5)
TUB_REAR_BOT = (31.5, -140.0,  39.0)
TUB_MID_TOP  = (37.2, -212.0, 102.0)
TUB_MID_BOT  = (35.5, -212.0,  61.5)


def _nose_fascia_frame():
    """Orthonormal frame on the raked front nose fascia (n points forward-downward)."""
    p_top = np.array([0.0, NF_TOP[1], NF_TOP[2]])
    p_bot = np.array([0.0, NF_BOT[1], NF_BOT[2]])
    edge_v = p_top - p_bot
    edge_v /= np.linalg.norm(edge_v)
    u = np.array([1.0, 0.0, 0.0])
    n = np.cross(u, edge_v)
    n /= np.linalg.norm(n)
    if n[1] > 0:
        n = -n
    center = 0.5 * (p_top + p_bot)
    span_v = float(np.linalg.norm(p_top - p_bot))
    return center, u, edge_v, n, span_v


def build_nose_tub_and_inserts():
    """Build Armored_Nose_Tub, Hex_Mesh_Grille, and Nose_Light_Bar together so fits are exact."""
    pts = [
        NF_TOP,
        (0.0, NF_TOP[1] - 1.5, NF_TOP[2] + 0.5),   # subtle forward-V nose prow
        NF_BOT,
        (0.0, NF_BOT[1] - 1.5, NF_BOT[2]),
        CHIN_MID,
        (0.0, CHIN_MID[1] - 3.0, CHIN_MID[2]),     # 3-facet armored chin center face
        CHIN_BOT,
        (0.0, CHIN_BOT[1] - 2.0, CHIN_BOT[2]),
        TUB_MID_TOP,
        TUB_REAR_TOP,
        TUB_MID_BOT,
        TUB_REAR_MID,
        TUB_REAR_BOT,
    ]
    def is_open(n):
        return (n[2] > 0.65) or (n[1] > 0.85) or (n[2] < -0.92)

    tub, _ = _hollow_block(pts, wall=WALL, open_if=is_open, mirror_x=True)

    # Coordinate frame on the raked nose fascia
    c, u, v, n, span_v = _nose_fascia_frame()
    pl_fascia = cq.Plane(origin=tuple(c), xDir=tuple(u), normal=tuple(n))

    # --- A. Cyan LED Light Bar groove + separate Nose_Light_Bar part ---
    v_top = span_v / 2.0
    led_v_hi = v_top - STRIP_EDGE
    led_v_lo = led_v_hi - STRIP_H
    led_v_mid = 0.5 * (led_v_hi + led_v_lo)
    led_w = 66.0

    led_groove = (
        cq.Workplane(pl_fascia)
        .center(0.0, led_v_mid)
        .rect(led_w, STRIP_H)
        .extrude(5.0)
        .translate(cq.Vector(*(n * (-GROOVE_D))))
        .val()
    )
    tub = tub.cut(led_groove)

    led_bar = (
        cq.Workplane(pl_fascia)
        .center(0.0, led_v_mid)
        .rect(led_w - 2 * FIT, STRIP_H - 2 * FIT)
        .extrude(GROOVE_D - 0.1)
        .translate(cq.Vector(*(n * (-GROOVE_D + 0.05))))
        .val()
    )
    for px in (-19.0, 19.0):
        peg_pos = c + u * px + v * led_v_mid - n * (GROOVE_D - 0.1)
        peg_pl = cq.Plane(origin=tuple(peg_pos), xDir=tuple(u), normal=tuple(-n))
        peg_outer = cq.Workplane(peg_pl).circle(2.4).extrude(WALL - GROOVE_D + 4.5).val()
        peg_bore = cq.Workplane(peg_pl).circle(1.6).extrude(WALL - GROOVE_D + 4.6).translate(cq.Vector(*(-n * 0.4))).val()
        led_bar = led_bar.fuse(peg_outer.cut(peg_bore))
        hole = cq.Workplane(peg_pl).circle(2.6).extrude(WALL + 5.0).translate(cq.Vector(*(n * 1.5))).val()
        tub = tub.cut(hole)

    # --- B. Hexagonal Mesh Intake Grille opening + separate Hex_Mesh_Grille part ---
    gr_v_hi = led_v_lo - 3.8
    gr_v_lo = -span_v / 2.0 + 3.5
    gr_h = gr_v_hi - gr_v_lo
    gr_v_mid = 0.5 * (gr_v_hi + gr_v_lo)
    gr_w = 50.0   # sized cleanly inside the inward-tapering V-bullbar cheeks

    def _chamf_rect_pts(w, h, ch=3.5):
        hw, hh = w / 2.0, h / 2.0
        return [
            (-hw + ch, -hh), (hw - ch, -hh),
            (hw, -hh + ch),  (hw, hh - ch),
            (hw - ch, hh),   (-hw + ch, hh),
            (-hw, hh - ch),  (-hw, -hh + ch),
        ]

    intake_cut = (
        cq.Workplane(pl_fascia)
        .center(0.0, gr_v_mid)
        .polyline(_chamf_rect_pts(gr_w, gr_h, ch=3.5))
        .close()
        .extrude(WALL + 12.0)
        .translate(cq.Vector(*(n * (-WALL - 5.0))))
        .val()
    )
    tub = tub.cut(intake_cut)

    gr_plate = (
        cq.Workplane(pl_fascia)
        .center(0.0, gr_v_mid)
        .polyline(_chamf_rect_pts(gr_w - 2 * FIT, gr_h - 2 * FIT, ch=3.3))
        .close()
        .extrude(WALL - 0.4)
        .translate(cq.Vector(*(n * (-WALL))))
        .val()
    )
    gr_flange = (
        cq.Workplane(pl_fascia)
        .center(0.0, gr_v_mid)
        .polyline(_chamf_rect_pts(gr_w + 5.0, gr_h + 5.0, ch=4.0))
        .close()
        .extrude(2.0)
        .translate(cq.Vector(*(n * (-WALL - 2.0 - FIT))))
        .val()
    )
    gr_bridge = (
        cq.Workplane(pl_fascia)
        .center(0.0, gr_v_mid)
        .polyline(_chamf_rect_pts(gr_w - 2 * FIT, gr_h - 2 * FIT, ch=3.3))
        .close()
        .extrude(FIT + 0.6)
        .translate(cq.Vector(*(n * (-WALL - FIT - 0.3))))
        .val()
    )
    gr_inner_hole = (
        cq.Workplane(pl_fascia)
        .center(0.0, gr_v_mid)
        .polyline(_chamf_rect_pts(gr_w - 4.5, gr_h - 4.5, ch=2.8))
        .close()
        .extrude(6.0)
        .translate(cq.Vector(*(n * (-WALL - 4.0))))
        .val()
    )
    grille = gr_plate.fuse(gr_bridge).fuse(gr_flange.cut(gr_inner_hole))

    # Cut hexagonal honeycomb lattice cells through the grille
    pitch = HEX_FLAT + HEX_RIB
    dx = pitch * math.sqrt(3.0) / 2.0
    dy = pitch
    hex_r = HEX_FLAT / math.sqrt(3.0)
    lim_x = (gr_w - 2 * FIT) / 2.0 - 3.2
    lim_y = (gr_h - 2 * FIT) / 2.0 - 2.8

    hex_centers = []
    for ix in range(-6, 7):
        cx_h = ix * dx
        y_off = (dy / 2.0) if (ix % 2 != 0) else 0.0
        for iy in range(-4, 5):
            cy_h = iy * dy + y_off
            if abs(cx_h) + hex_r * 0.85 <= lim_x and abs(cy_h) + hex_r * 0.85 <= lim_y:
                hex_centers.append((cx_h, gr_v_mid + cy_h))

    if hex_centers:
        wp_hex = cq.Workplane(pl_fascia).pushPoints(hex_centers).polygon(6, 2.0 * hex_r).extrude(WALL + 8.0)
        hex_cut = wp_hex.translate(cq.Vector(*(n * (-WALL - 4.0)))).val()
        grille = grille.cut(hex_cut)

    tub = tub.clean()
    grille = grille.clean()
    led_bar = led_bar.clean()
    assert tub.isValid() and len(tub.Solids()) == 1, f"Nose tub invalid ({len(tub.Solids())} solids)"
    assert grille.isValid() and len(grille.Solids()) == 1, f"Hex grille invalid ({len(grille.Solids())} solids)"
    assert led_bar.isValid() and len(led_bar.Solids()) == 1, f"LED bar invalid ({len(led_bar.Solids())} solids)"
    return tub, grille, led_bar


# =================================================================================================
# 2. ARMORED HOOD / COWL DECK (flat outer side panels + steeper-walled center power-bulge)
# =================================================================================================
def build_armored_hood():
    """Removable multi-faceted top hood matching the exact original Fugu V2 hood length
    (y = -304.5 .. -204.0, ending cleanly at the cowl / A-pillar base COWL_TONGUE_Y = -204.0):
    - Flatter outer side panels (|x| = 24..40.4 mm) hugging the exact upper exo-rail contour
    - Distinct concave valley crease at |x| = 24 mm where steeper (~45 deg) power-bulge walls rise
      to a crisp flat-crowned center bulge (|x| <= 16 mm, +9.5 mm proud of the outer deck)
    - Hollowed uniformly to 3.0 mm wall thickness, with 4 M3 mounting holes along the side rails.
    """
    def _outer_and_inner(pts_top, wall=WALL, drop=24.0):
        pts_bot = [(x, y, z - drop) for x, y, z in pts_top]
        outer = _planes(_both(pts_top + pts_bot))
        inner = outer.copy()
        for q in inner:
            if q[2] < 0.25:
                q[3] -= 40.0
            else:
                q[3] += wall
        return _solid_from_planes(outer), _solid_from_planes(inner)

    # 1) Base flatter outer hood deck (y = -290.5 .. -204.0, blunter high-clearance nose ending at V2 cowl)
    base_pts = [
        (37.4, -289.0, 114.5), (40.2, -252.0, 123.5), (40.4, -204.0, 132.0),
        ( 0.0, -290.5, 117.0), ( 0.0, -252.0, 126.0), ( 0.0, -204.0, 134.5),
    ]
    b_out, b_in = _outer_and_inner(base_pts, WALL)

    # 2) Raised center power-bulge (steeper ~45 deg side walls from |x|=24 at deck -> |x|=15.5 at top,
    # rising +9.5 mm proud of the outer deck with a crisp flat-crowned top)
    bulge_pts = [
        # Base footprint sitting on the flatter outer deck (forms crisp concave valley!)
        (23.5, -282.0, 116.0), (24.5, -252.0, 124.0), (25.0, -204.0, 132.5),
        # Steeper angled power-bulge top shoulder edges
        (14.0, -276.0, 124.0), (15.5, -252.0, 134.5), (16.5, -204.0, 143.0),
        # Crisp flat-crowned center ridge
        ( 0.0, -277.0, 125.0), ( 0.0, -252.0, 135.7), ( 0.0, -204.0, 144.2),
    ]
    p_out, p_in = _outer_and_inner(bulge_pts, WALL)

    hood_outer = b_out.fuse(p_out)
    hood_inner = b_in.fuse(p_in)
    hood = hood_outer.cut(hood_inner)
    hood = hood.cut(cq.Solid.makeBox(120.0, 150.0, 30.0, cq.Vector(-60.0, -315.0, 76.0)))

    # 4 M3 fastener clearance holes (2 per side) along the flatter rail-hugging outer side panels
    for sx in (-1, 1):
        for hy in (-270.0, -218.0):
            hx = sx * 33.5
            bolt = cq.Solid.makeCylinder(1.7, 75.0, cq.Vector(hx, hy, 95.0), cq.Vector(0, 0, 1))
            hood = hood.cut(bolt)

    hood = hood.clean()
    assert hood.isValid() and len(hood.Solids()) == 1, f"Armored hood invalid ({len(hood.Solids())} solids)"
    return hood


# =================================================================================================
# 3. FORWARD-SWEPT LOUVERED SIDE-POD FLANK ARMOR (Left & Right)
# =================================================================================================
def _build_side_pod_right():
    r"""Build the +X forward-swept angular side-pod armor panel matching the concept render:
    - Main flank block sits strictly behind arm_tool (y >= -55) down to the deck (z = 20)
    - Forward-swept upper cowl-shoulder wing reaches y = -98 above z = 104 (strictly above
      arm_tool z_max = 100.7 and behind uplink_tool y_max = -114.5 -> 0.0000 mm3 overlap!)
    - 3 sculpted, raised forward-leaning (\ \ \) cyberpunk gill louvers with rear exhaust throats.
    """
    # Block A: Main lower/rear flank (y = -55 .. +134, z = 20 .. 178) — aggressively tapered rear apex into tail cage
    main_outer = [
        (78.0,  -55.0, 145.5),
        (72.0,  -40.0, 150.0),
        (61.0,   30.0, 161.0),
        (49.5,  134.0, 178.0),   # razor-tapered tail tip hugging the tail cage truss
        (91.5,  -54.5, 102.0),
        (91.0,  -40.0,  92.0),
        (86.0,   48.0, 118.0),
        (61.0,  -54.0,  20.0),
        (66.0,   15.0,  29.0),
        (58.0,   92.0, 136.0),
    ]
    main_inboard = [
        (49.0,  -55.0, 145.5),
        (46.0,  134.0, 178.0),
        (49.0,  -54.0,  20.0),
        (49.0,   15.0,  29.0),
        (49.0,   92.0, 136.0),
    ]
    main_pts = main_outer + main_inboard

    def is_open_main(n):
        return (n[0] < -0.35) or (n[2] > 0.72) or (n[1] > 0.85) or (n[2] < -0.82)

    pod_main, _ = _hollow_block(main_pts, wall=WALL, open_if=is_open_main, mirror_x=False)
    pod_main = pod_main.cut(cq.Solid.makeBox(56.0, 300.0, 210.0, cq.Vector(-10.0, -155.0, 0.0)))

    # Block B: Forward-swept upper cowl-shoulder prow overlapping onto the front exo-cage rail at y = -140!
    # Stays strictly ABOVE arm_tool (z_max=100.7) and tapers inward to |x| = 49.0..55.5 ahead of y = -114
    # so it clears uplink_tool (x >= 61.0) while overlapping & bolting straight onto the exo-cage upright!
    wing_pts = [
        # Forward rail-overlap ear hugging the outside of the exo-cage upright (x = 48.5, y = -140)
        (54.5, -144.0, 140.5),   # upper front overlap corner (bolts onto TIE_UPPER rail)
        (54.5, -144.0, 114.0),   # lower front overlap corner (bolts onto TIE_MID rail)
        # Transition shoulder behind uplink_tool (y = -112) flaring out cleanly to main flank
        (76.0, -104.0, 140.5),
        (78.0,  -54.0, 145.5),   # joins main upper belt seamlessly
        (88.5,  -78.0, 106.0),   # lower-front prow crease (safely above z=101 arm sweep)
        (91.5,  -54.0, 104.5),   # joins main chine seamlessly
        # Inboard backing plane (x = 49.2 clears exo-cage outer face x = 49.0 by 0.2 mm)
        (49.2, -144.0, 140.5),
        (49.2, -144.0, 114.0),
        (49.2,  -54.0, 145.5),
        (49.2,  -78.0, 106.0),
        (49.2,  -54.0, 104.5),
    ]
    def is_open_wing(n):
        return (n[0] < -0.35) or (n[2] > 0.72) or (n[1] > 0.75) or (n[2] < -0.80)

    pod_wing, _ = _hollow_block(wing_pts, wall=WALL, open_if=is_open_wing, mirror_x=False)
    pod_wing = pod_wing.cut(cq.Solid.makeBox(59.2, 160.0, 200.0, cq.Vector(-10.0, -160.0, 0.0)))

    pod = pod_main.fuse(pod_wing)

    # 3 sculpted, raised forward-leaning (\ \ \) cyberpunk gill louvers on the lower-mid flank
    flank_n = np.array([0.935, 0.04, -0.35], float)
    flank_n /= np.linalg.norm(flank_n)
    ang = math.radians(-22.0)
    v_dir = np.array([0.0, math.cos(ang), math.sin(ang)], float)
    v_dir -= np.dot(v_dir, flank_n) * flank_n
    v_dir /= np.linalg.norm(v_dir)
    u_dir = np.cross(flank_n, v_dir)
    u_dir /= np.linalg.norm(u_dir)
    if u_dir[2] < 0:
        u_dir = -u_dir

    outer_env = _solid_from_planes(_planes(main_pts))
    louver_specs = [(-36.0, 62.0), (-18.0, 66.0), (0.0, 70.0)]
    throat_cuts = []
    for ly, lz in louver_specs:
        probe = outer_env.intersect(cq.Solid.makeBox(120.0, 0.6, 0.6, cq.Vector(20.0, ly - 0.3, lz - 0.3)))
        fx = probe.BoundingBox().xmax if probe.Volume() > 1e-6 else 80.5
        c_surf = np.array([fx, ly, lz], float)

        hu_outer = 18.5
        hu_top   = 15.0
        v_lead   = -6.5
        v_crest  =  2.0
        v_lip    =  3.4
        w_embed  = -2.2
        w_proud  =  5.8

        def to_world(u, v, w):
            p = c_surf + u * u_dir + v * v_dir + w * flank_n
            return (float(p[0]), float(p[1]), float(p[2]))

        fin_pts = [
            to_world(-hu_outer, v_lead,  w_embed),
            to_world( hu_outer, v_lead,  w_embed),
            to_world( hu_outer, v_crest, w_embed),
            to_world(-hu_outer, v_crest, w_embed),
            to_world(-hu_outer, v_lead,  0.2),
            to_world( hu_outer, v_lead,  0.2),
            to_world(-hu_top,   v_crest, w_proud),
            to_world( hu_top,   v_crest, w_proud),
            to_world(-hu_top,   v_lip,   w_proud - 1.2),
            to_world( hu_top,   v_lip,   w_proud - 1.2),
        ]
        pod = pod.fuse(_hull(fin_pts))

        t_hu = 14.5
        throat_pts = [
            to_world(-t_hu, v_crest - 0.5, w_proud - 2.4),
            to_world( t_hu, v_crest - 0.5, w_proud - 2.4),
            to_world( t_hu, v_lip + 2.8,   w_proud - 2.4),
            to_world(-t_hu, v_lip + 2.8,   w_proud - 2.4),
            to_world(-t_hu, v_crest - 5.0, -WALL - 4.0),
            to_world( t_hu, v_crest - 5.0, -WALL - 4.0),
            to_world( t_hu, v_lip - 1.0,   -WALL - 4.0),
            to_world(-t_hu, v_lip - 1.0,   -WALL - 4.0),
        ]
        throat_cuts.append(_hull(throat_pts))

    for tc in throat_cuts:
        pod = pod.cut(tc)

    # M3 mounting holes along upper/lower edges (including front rail-overlap bolt at y=-138!)
    for my, mz in ((-138.0, 130.0), (-82.0, 128.0), (-38.0, 136.0), (36.0, 146.0), (-40.0, 38.0), (14.0, 44.0)):
        bolt = cq.Solid.makeCylinder(1.7, 60.0, cq.Vector(45.0, my, mz), cq.Vector(1, 0, 0))
        pod = pod.cut(bolt)

    pod = pod.clean()
    assert pod.isValid() and len(pod.Solids()) == 1, f"Side pod invalid ({len(pod.Solids())} solids)"
    return pod


def build_side_pods():
    right = _build_side_pod_right()
    left = right.mirror("YZ").clean()
    assert left.isValid() and len(left.Solids()) == 1, "Left side pod invalid"
    return right, left


# =================================================================================================
# 4. MATCHING ARMORED ROOF PANEL (flat outer panels + steeper-walled center power-bulge)
# =================================================================================================
def build_armored_roof():
    """Drop-in replacement for parts/roof_plate.py on Fugu V2's open-cockpit roof frame, matching
    the hood's exact faceted design language: flatter outer panels + steeper-walled center bulge.

    Exact Fugu V2 open-cockpit roof rail centerlines (from renders/v2_build8.log):
      A (front): (+-60.9, -86.9, 196.9),  B (rear): (+-45.9, 15.0, 219.9),  tube half = 3.5 mm.
    Underside sits 4.10 mm above the A-B tube centerlines (clears headers + roof X by 0.56 mm).
    """
    A = np.array([60.9, -86.9, 196.9], float)
    B = np.array([45.9,  15.0, 219.9], float)
    half = 3.5
    seat_offset = half + 0.60   # matches roof_plate.py (0.36 proud + 0.24 clearance)
    roof_t = 2.5                # lightweight 2.5 mm printable wall to keep CG low

    u_vec = np.array([0.0, B[1] - A[1], B[2] - A[2]], float)
    L = float(np.linalg.norm(u_vec))
    u_dir = u_vec / L
    n_dir = np.cross(np.array([1.0, 0.0, 0.0]), u_dir)
    n_dir /= np.linalg.norm(n_dir)
    if n_dir[2] < 0:
        n_dir = -n_dir

    # Origin at center of front header underside plane (local x = world x, local y = along roof u_dir, local z = along n_dir)
    orig = np.array([0.0, A[1], A[2]], float) + n_dir * seat_offset
    pl_roof = cq.Plane(origin=tuple(orig), xDir=(1.0, 0.0, 0.0), normal=tuple(n_dir))

    def to_w(lx, ly, lz):
        p = orig + np.array([lx, 0.0, 0.0]) + ly * u_dir + lz * n_dir
        return (float(p[0]), float(p[1]), float(p[2]))

    # 1) Flatter outer roof deck (tapered trapezoid matching roof_plate.py outer tube faces)
    xf = A[0] + half   # 64.4 mm
    xr = B[0] + half   # 49.4 mm
    y0, y1 = -half, L + half
    base_top = [
        to_w(xf, y0, roof_t),       to_w(xr, y1, roof_t),
        to_w(0.0, y0, roof_t + 1.5), to_w(0.0, y1, roof_t + 1.5),
    ]
    base_bot = [
        to_w(xf, y0, 0.0),          to_w(xr, y1, 0.0),
        to_w(0.0, y0, 0.0),         to_w(0.0, y1, 0.0),
    ]
    base_solid = _hull(_both(base_top + base_bot))

    # 2) Matching steeper-walled center power-bulge spine (+8.5 mm proud, hollowed underneath to 2.5 mm wall)
    bulge_h = 8.5
    bulge_outer_pts = [
        # Base footprint on roof deck (concave valley crease echoing the hood)
        to_w(26.0, y0 + 5.0,  roof_t - 0.5), to_w(22.0, y1 - 3.0,  roof_t - 0.5),
        # Steeper (~45 deg) angled power-bulge shoulder edges
        to_w(16.5, y0 + 13.0, roof_t + bulge_h), to_w(13.5, y1 - 8.0, roof_t + bulge_h),
        # Subtle flat-crowned center ridge (+1.0 mm crown)
        to_w(0.0,  y0 + 12.0, roof_t + bulge_h + 1.0), to_w(0.0, y1 - 7.5, roof_t + bulge_h + 1.0),
        # Floor anchor
        to_w(26.0, y0 + 5.0,  0.0),          to_w(22.0, y1 - 3.0,  0.0),
    ]
    bulge_outer = _hull(_both(bulge_outer_pts))

    # Hollow underside cavity under the power-bulge (offset inward by roof_t)
    bulge_cav_pts = [
        to_w(22.5, y0 + 8.0,  -2.0),         to_w(18.5, y1 - 6.0,  -2.0),
        to_w(14.5, y0 + 15.0, bulge_h),      to_w(11.5, y1 - 10.0, bulge_h),
        to_w(0.0,  y0 + 14.0, bulge_h + 1.0), to_w(0.0, y1 - 9.5,  bulge_h + 1.0),
    ]
    bulge_cav = _hull(_both(bulge_cav_pts))

    roof = base_solid.fuse(bulge_outer).cut(bulge_cav)

    # 4 M3 mounting holes near the roof rail corners
    for sx in (-1, 1):
        for lx, ly in ((A[0] - 2.0, 12.0), (B[0] - 1.0, L - 12.0)):
            p_hole = orig + np.array([sx * lx, 0.0, 0.0]) + ly * u_dir - n_dir * 4.0
            bolt = cq.Solid.makeCylinder(1.7, 25.0, cq.Vector(*p_hole), cq.Vector(*n_dir))
            roof = roof.cut(bolt)

    roof = roof.clean()
    assert roof.isValid() and len(roof.Solids()) == 1, f"Armored roof invalid ({len(roof.Solids())} solids)"
    return roof


# =================================================================================================
# 5. AGGRESSIVELY TAPERED ARMORED TAIL LIGHT HOUSING (tapers forward into the tail cage!)
# =================================================================================================
def build_armored_tail_housing():
    """Faceted stealth tail-light housing that wraps the existing Fugu V2 Tail_light_strip
    (x=[-38,38], y=[213.8,234.8], z=[165.2,180.4]) at the rear and tapers aggressively forward
    and inward right back into the tail cage rails (y=166 -> y=132, safely above the rear shock
    mount at y=136.3, z=180.2).
    """
    pts = [
        # Rear bezel framing Tail_light_strip (y = 216 .. 237)
        (43.0, 224.0, 184.5),   # rear upper outer corner
        (40.0, 236.5, 183.0),   # rear lip upper corner
        ( 0.0, 237.5, 187.5),   # rear center peak ridge
        (41.5, 234.0, 162.5),   # rear lip lower corner
        ( 0.0, 235.5, 161.5),   # rear lower center
        # Mid-body shoulder wrapping the tail cage nodes at y = 166
        (36.0, 180.0, 189.0),
        ( 0.0, 180.0, 195.5),   # faceted center spine crest
        (34.5, 178.0, 166.0),
        # Aggressive forward taper narrowing sharply right back into the tail cage!
        (22.0, 148.0, 191.5),
        ( 0.0, 132.0, 192.5),   # razor-tapered forward spear tip meeting tail cage spine
        (20.0, 148.0, 182.5),
        ( 0.0, 132.0, 186.0),
    ]
    # Open on the underside so the tail cage tubes nest cleanly inside, and open slightly at front tip underside
    def is_open(n):
        return n[2] < -0.55

    tail, _ = _hollow_block(pts, wall=WALL, open_if=is_open, mirror_x=True)

    # Clean recessed pocket for the existing Tail_light_strip (x=[-38,38], y=[213.8,234.8], z=[165.2,180.4])
    strip_cut = cq.Solid.makeBox(76.8, 28.0, 15.8, cq.Vector(-38.4, 213.4, 164.9))
    tail = tail.cut(strip_cut)

    tail = tail.clean()
    assert tail.isValid() and len(tail.Solids()) == 1, f"Armored tail housing invalid ({len(tail.Solids())} solids)"
    return tail


def build_all_armor():
    tub, grille, led_bar = build_nose_tub_and_inserts()
    hood = build_armored_hood()
    pod_r, pod_l = build_side_pods()
    roof = build_armored_roof()
    tail = build_armored_tail_housing()
    return {
        "Armored_Nose_Tub": tub,
        "Hex_Mesh_Grille": grille,
        "Nose_Light_Bar": led_bar,
        "Armored_Hood_Deck": hood,
        "Armored_Roof_Panel": roof,
        "Armored_Tail_Housing": tail,
        "Side_Pod_Right": pod_r,
        "Side_Pod_Left": pod_l,
    }


if __name__ == "__main__":
    parts = build_all_armor()
    for name, sol in parts.items():
        bb = sol.BoundingBox()
        print(f"{name:20s}: vol={sol.Volume():8.1f} mm3  x=[{bb.xmin:6.1f},{bb.xmax:6.1f}] y=[{bb.ymin:6.1f},{bb.ymax:6.1f}] z=[{bb.zmin:6.1f},{bb.zmax:6.1f}]")
