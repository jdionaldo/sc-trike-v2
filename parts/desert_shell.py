"""Desert armored shell (owner's design, desert_shell_variant/armored_shell.py, ported November 24).

Nose tub with hex grille and light bar, hood deck, side pods with gill louvers, roof panel. World coordinates (mm, Z up, front = -Y).
The owner's folder stays as it is; this module is the copy the build uses. Changes against the source are listed in docs/v2/louvers.md
(louvers) and NOTES.md (fit cuts, mounts). The tail light housing of the source is left out on the owner's word.
"""
import math
import numpy as np
import cadquery as cq
from scipy.spatial import ConvexHull, HalfspaceIntersection
from scipy.optimize import linprog
from soma import param

from parts.params import WALL_PRINT, PANEL_GAP

WALL = WALL_PRINT     # mm panel thickness (parameter "Printed panel thickness", 3.0)
FIT = PANEL_GAP       # mm clearance between a panel and its insert (parameter "Panel seam gap", 0.3)
FIT_CUT = 0.4         # mm a panel stays clear of the cage, deck and skid it is trimmed against (@3d-printing: mating faces never touch)
SCREW_PILOT = 2.5     # mm pilot in the cage bosses and hood tabs for M3 self-tapping screws for plastic (ASSUMED screw type; the cage's other bosses are M2 pilots)
BOSS_MAX_REACH = 40.0 # mm longest cage boss towards a panel (the pod skin stands up to 35 mm off the uprights)
POD_BOSS_FRONT = (-138.0, 130.0)   # mm (y, z): pod front ear screw on the front upright (the source's hole)
POD_BOSS_FRONT_D = 6.0             # mm: its boss is slim, the ear's front wall stands 3 mm ahead of the screw
POD_BOSS_DIAG_Y = -30.0            # mm: pod upper middle screw, on the cage's door diagonal (source hole (-38, 136) moved 8 mm back and 6 mm down onto the tube)
POD_BOSS_TAIL_Y = 112.0       # mm: pod tail tip screw on the middle tail rail
POD_LOWER_Y = (-35.0, -10.0) # mm: two screws on each pod's lower edge, into brackets on the rear deck's side (below the gills: fin ends at z 46 and up)
POD_LOWER_Z = 30.0            # mm: at the height where the deck plate is solid out to its side face (MEASURED: x 20 .. 54.5 at z 26 .. 30)
POD_LOWER_D = 8.0             # mm bracket diameter
POD_LOWER_EMBED = 1.5         # mm the bracket starts inside the deck side
POD_LOWER_PILOT_DEPTH = 8.0   # mm the pilot runs on into the deck plate
POD_SCREWS = 4                # per pod: front ear, door diagonal, the old front shell boss on the rear upright, tail tip. No screw on the lower edge (OPEN, NOTES.md)
TUB_BOSS_Y = (-160.0, -240.0) # mm: nose tub screws, on the lower side rail and on the swept side bar
HOOD_SCREWS = ((33.5, -270.0), (33.5, -218.0))   # (x, y) of the hood screws per side (source positions)
HOOD_TAB_W, HOOD_TAB_T = 9.0, 6.0                # mm: tab under each hood screw, from the exo rail inward
COLOR_ARMOR, COLOR_ROOF, COLOR_GRILLE, COLOR_LED = (0.28, 0.31, 0.27), (0.16, 0.17, 0.19), (0.18, 0.20, 0.22), (0.05, 0.88, 0.95)   # source colours
SCREW_D = 3.4         # mm clearance hole for the M3 screws into the cage bosses (@3d-printing: 0.2 over the nominal 3.2)
STRIP_H = 8.5         # mm height of the cyan LED light bar on the raked nose face
STRIP_EDGE = 2.5      # mm top/side margin around the LED light bar
GROOVE_D = 1.8        # mm blind groove depth for LED bar (the fascia behind it is 1.2 mm, the minimum wall; the bar is glued in and backs it)
LED_FLOOR = 1.6       # mm of wall left behind the light bar groove (backing pad inside the tub)
LED_BACK_RIM = 1.0    # mm the backing pad reaches past the groove all round (the grille flange starts 1.3 mm under the groove)
LED_BORE = 3.2        # mm bore of the two LED pegs behind the light bar (3 mm LED)
LED_PEG_WALL = 1.2    # mm wall of the pegs (source: 0.8, under the minimum wall of 1.2; @3d-printing)
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
CHIN_BOT = (17.5, -236.5,  46.5)   # lower chin corner. Source: z 40.5, which stood 1596 mm3 inside the skid's upturned nose (skid top z 45 at this y, MEASURED);
                                   # trimming it there left feather edges of 0.01 mm, so the corner is 6 mm higher and passes over the skid nose

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
    # Backing behind the groove: the fascia is a shallow V (prow 1.5 mm ahead at the centre) but the groove is cut flat, so at the centre the
    # groove's floor ran out to 0.04 mm (MEASURED, soma analyze thickness; the source has the same). A flat pad inside the tub keeps LED_FLOOR.
    led_back = (
        cq.Workplane(pl_fascia)
        .center(0.0, led_v_mid)
        .rect(led_w + 2 * LED_BACK_RIM, STRIP_H + 2 * LED_BACK_RIM)
        .extrude(LED_FLOOR + GROOVE_D - 0.5)
        .translate(cq.Vector(*(n * (-GROOVE_D - LED_FLOOR))))
        .val()
    )
    tub = tub.fuse(led_back)
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
        peg_outer = cq.Workplane(peg_pl).circle(LED_BORE / 2.0 + LED_PEG_WALL).extrude(WALL - GROOVE_D + 4.5).val()
        peg_bore = cq.Workplane(peg_pl).circle(LED_BORE / 2.0).extrude(WALL - GROOVE_D + 4.6).translate(cq.Vector(*(-n * 0.4))).val()
        led_bar = led_bar.fuse(peg_outer.cut(peg_bore))
        hole = cq.Workplane(peg_pl).circle(LED_BORE / 2.0 + LED_PEG_WALL + 0.2).extrude(WALL + 5.0).translate(cq.Vector(*(n * 1.5))).val()
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
        for hx0, hy in HOOD_SCREWS:
            hx = sx * hx0
            bolt = cq.Solid.makeCylinder(SCREW_D / 2.0, 75.0, cq.Vector(hx, hy, 95.0), cq.Vector(0, 0, 1))
            hood = hood.cut(bolt)

    hood = hood.clean()
    assert hood.isValid() and len(hood.Solids()) == 1, f"Armored hood invalid ({len(hood.Solids())} solids)"
    return hood


# =================================================================================================
# 3. FORWARD-SWEPT LOUVERED SIDE-POD FLANK ARMOR (Left & Right)
# =================================================================================================
# --- gill louvers on the pod flank (docs/v2/louvers.md) ---
LOUVER_COUNT = param(3, 'Gill louvers per side pod', min=2, max=4, group='Desert shell')
LOUVER_FIRST = (-36.0, 62.0)      # mm (y, z) of the front gill's centre on the flank
LOUVER_PITCH = (18.0, 4.0)        # mm (y, z) from one gill to the next: the row climbs to the rear
LOUVER_FLANK_N = (0.935, 0.04, -0.35)   # outward normal of the flank frame the gills are built in (the lower flank facet leans 20 deg inward at the bottom)
LOUVER_LEAN = -22.0               # deg: the gill's cross direction v in the yz plane before projection onto the flank (rearward and down)
LOUVER_FALLBACK_X = 80.5          # mm flank x if the surface probe misses
FIN_HALF_BASE = 18.5              # mm half length of the fin at its base, along u (up the flank)
FIN_HALF_CREST = 15.0             # mm half length at the crest: the ends slope 3.5 mm in
FIN_LEAD = -6.5                   # mm v of the leading (front) edge, where the ramp starts
FIN_CREST = 2.0                   # mm v of the crest
FIN_LIP = 3.4                     # mm v of the rear lip that hangs over the throat
FIN_EMBED = -2.2                  # mm w of the fin's base, inside the skin, so fin and skin fuse over a face and not along an edge
FIN_LEAD_PROUD = 0.2              # mm w of the leading edge above the skin
FIN_PROUD = 5.8                   # mm w of the crest above the skin
FIN_LIP_DROP = 1.2                # mm the lip's edge lies under the crest
THROAT_HALF = 14.5                # mm half length of the opening along u
THROAT_ROOF = 2.4                 # mm of fin left above the throat (crest skin thickness)
THROAT_V = (-0.5, 2.8)            # mm outer mouth of the throat, from the crest (v) to behind the lip
THROAT_V_IN = (-5.0, -1.0)        # mm inner mouth, shifted forward: the throat runs forward and inward under the fin, so it opens to the rear
THROAT_DEPTH = 4.0                # mm the throat cutter runs past the inner face of the wall
BEAM_WINDOW_X = 61.0              # mm: notch in the pod's front wall for the balance beam's fork tip (tip reaches |x| 57.7)
BEAM_WINDOW_Y = (-57.0, -49.0)    # mm: through the 3 mm front wall (y -55 .. -52)
BEAM_WINDOW_Z = (76.0, 92.0)      # mm: the tip passes at z 79.8 .. 87.1 (lowest with one wheel at +41.9)
LOUVER_INFO = []                  # filled by the build: one dict per gill (centre, open area), read by model.py for the notes
POD_ENV = []                      # filled by the build: the right pod's outer envelope (main flank + wing), to clip parts fused into its wall
WING_EDGE_TIP = ((-96.4, 73.0), (-55.6, 74.6))   # (y, x) of the wing top edge's feather tip at the ends of the cut (MEASURED sections)
WING_EDGE_BACK = 0.5              # mm outboard of the tip line: there the edge is 1.4 .. 1.5 mm tall (a flat land)
WING_EDGE_Z = (136.0, 148.0)      # z band of the cut (the wing's top edge is at z 141.9 .. 145.3 there)


def louver_frame():
    """Unit vectors of the flank frame: u up along the fin, v across it (to the rear), n outward."""
    n = np.array(LOUVER_FLANK_N, float); n /= np.linalg.norm(n)
    ang = math.radians(LOUVER_LEAN)
    v = np.array([0.0, math.cos(ang), math.sin(ang)], float)
    v -= np.dot(v, n) * n; v /= np.linalg.norm(v)
    u = np.cross(n, v); u /= np.linalg.norm(u)
    if u[2] < 0:
        u = -u
    return u, v, n


def louvers(pod, outer_env):
    """Fuses the raised gill fins to the right pod and cuts their rear-facing throats. Returns (pod, info)."""
    u_dir, v_dir, flank_n = louver_frame()
    fins, throats, info = [], [], []
    for i in range(int(LOUVER_COUNT)):
        ly, lz = LOUVER_FIRST[0] + i * LOUVER_PITCH[0], LOUVER_FIRST[1] + i * LOUVER_PITCH[1]
        probe = outer_env.intersect(cq.Solid.makeBox(120.0, 0.6, 0.6, cq.Vector(20.0, ly - 0.3, lz - 0.3)))   # where the skin is at this (y, z)
        fx = probe.BoundingBox().xmax if probe.Volume() > 1e-6 else LOUVER_FALLBACK_X
        c_surf = np.array([fx, ly, lz], float)

        def to_world(u, v, w, c=c_surf):
            p = c + u * u_dir + v * v_dir + w * flank_n
            return (float(p[0]), float(p[1]), float(p[2]))

        fins.append(_hull([
            to_world(-FIN_HALF_BASE, FIN_LEAD, FIN_EMBED), to_world(FIN_HALF_BASE, FIN_LEAD, FIN_EMBED),
            to_world(FIN_HALF_BASE, FIN_CREST, FIN_EMBED), to_world(-FIN_HALF_BASE, FIN_CREST, FIN_EMBED),
            to_world(-FIN_HALF_BASE, FIN_LEAD, FIN_LEAD_PROUD), to_world(FIN_HALF_BASE, FIN_LEAD, FIN_LEAD_PROUD),
            to_world(-FIN_HALF_CREST, FIN_CREST, FIN_PROUD), to_world(FIN_HALF_CREST, FIN_CREST, FIN_PROUD),
            to_world(-FIN_HALF_CREST, FIN_LIP, FIN_PROUD - FIN_LIP_DROP), to_world(FIN_HALF_CREST, FIN_LIP, FIN_PROUD - FIN_LIP_DROP),
        ]))
        w_out, w_in = FIN_PROUD - THROAT_ROOF, -WALL - THROAT_DEPTH
        throats.append(_hull([
            to_world(-THROAT_HALF, FIN_CREST + THROAT_V[0], w_out), to_world(THROAT_HALF, FIN_CREST + THROAT_V[0], w_out),
            to_world(THROAT_HALF, FIN_LIP + THROAT_V[1], w_out), to_world(-THROAT_HALF, FIN_LIP + THROAT_V[1], w_out),
            to_world(-THROAT_HALF, FIN_CREST + THROAT_V_IN[0], w_in), to_world(THROAT_HALF, FIN_CREST + THROAT_V_IN[0], w_in),
            to_world(THROAT_HALF, FIN_LIP + THROAT_V_IN[1], w_in), to_world(-THROAT_HALF, FIN_LIP + THROAT_V_IN[1], w_in),
        ]))
        info.append({'centre': tuple(round(float(q), 1) for q in c_surf)})
    for f in fins:
        pod = pod.fuse(f)
    for t, d in zip(throats, info):
        before = pod.Volume()
        pod = pod.cut(t)
        d['cut_mm3'] = round(before - pod.Volume(), 1)
        # open area: the throat's section in the plane of the wall's inner face (w = -WALL), measured on the cutter
        c = np.array(d['centre'], float) - (WALL + 0.01) * flank_n
        pl = cq.Face.makeFromWires(cq.Wire.makePolygon([cq.Vector(*(c + a * u_dir + b * v_dir)) for a, b in ((-40, -40), (40, -40), (40, 40), (-40, 40))], close=True))
        d['open_mm2'] = round(pl.intersect(t).Area(), 1)
    return pod, info


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

    pod, _info = louvers(pod, _solid_from_planes(_planes(main_pts)))
    env = _solid_from_planes(_planes(main_pts)).fuse(_solid_from_planes(_planes(wing_pts))).clean()
    # Wing top edge (orchestrator, November 25): between y -96 and -56 the top edge ended in a feather (0.15 mm at the tip, 1.2 mm reached
    # 0.4 .. 0.45 mm outboard; sections MEASURED at y -96 .. -58). Cut back to a flat land WING_EDGE_BACK outboard of the tip line.
    (_y0, _x0), (_y1, _x1) = WING_EDGE_TIP
    _cutp = [(40.0, _y0), (_x0 + WING_EDGE_BACK, _y0), (_x1 + WING_EDGE_BACK, _y1), (40.0, _y1)]
    _wc = cq.Workplane("XY").workplane(offset=WING_EDGE_Z[0]).polyline(_cutp).close().extrude(WING_EDGE_Z[1] - WING_EDGE_Z[0]).val()
    _v0 = pod.Volume(); pod = pod.cut(_wc).clean()
    print("wing top edge cut back %.1f mm outboard of the feather tip (a land about 1.4 mm tall): %.1f mm3" % (WING_EDGE_BACK, _v0 - pod.Volume()))
    assert 0.5 < _v0 - pod.Volume() < 60.0, "wing edge cut removed %.1f mm3 (expected the feather only)" % (_v0 - pod.Volume())
    POD_ENV[:] = [env]
    # (_fill_top_flakes is not used: thickness 57 showed its 1.5 mm curtain under a 0.5 mm face is itself a fin; the y -40 .. 29 'ledge' is a
    # 1.5 mm shelf whose 68 deg inner edge reads low, MEASURED in a section, a named exception in NOTES)
    # Window for the balance beam's fork end: at 10 deg lean with 12 mm bump the fork tip swings back into the pod's front wall
    # (3.42 mm3 at x 52.1..57.7, y -54.5..-53.4, z 83.8..87.1, MEASURED in the pose run). A square notch from the wall's inboard edge.
    pod = pod.cut(cq.Solid.makeBox(BEAM_WINDOW_X - 40.0, BEAM_WINDOW_Y[1] - BEAM_WINDOW_Y[0], BEAM_WINDOW_Z[1] - BEAM_WINDOW_Z[0],
                                   cq.Vector(40.0, BEAM_WINDOW_Y[0], BEAM_WINDOW_Z[0])))
    LOUVER_INFO[:] = _info
    pod = pod.clean()
    assert pod.isValid() and len(pod.Solids()) == 1, f"Side pod invalid ({len(pod.Solids())} solids)"
    return pod


def _fill_top_flakes(pod, env, t_min=0.6, drop=1.5):
    """Thickness 54: where an open top facet of the hollow block meets a closed one at a shallow angle, the top edge ends in a flake
    (0.03 mm over the inner skin at x 61 .. 72, y -40 .. 29). Every small upward face with less than t_min of material under it is filled
    down by `drop` (its prism along -z, clipped to the pod's outer envelope): the flake becomes part of the wall."""
    out, n = pod, 0
    for f in pod.Faces():
        try:
            c = f.Center(); nz = f.normalAt(c).z
        except Exception:
            continue
        if nz < 0.9 or f.Area() > 100.0:
            continue
        ray = cq.Solid.makeCylinder(0.02, 5.0, c + cq.Vector(0, 0, 0.01), cq.Vector(0, 0, -1))
        hit = pod.intersect(ray)
        th = max((q.BoundingBox().zlen for q in hit.Solids()), default=0.0)
        if th >= t_min:
            continue
        from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
        from OCP.gp import gp_Vec
        fill = cq.Shape.cast(BRepPrimAPI_MakePrism(f.wrapped, gp_Vec(0, 0, -drop)).Shape()).intersect(env)
        r = out.fuse(fill).clean()
        if r.isValid() and len(r.Solids()) == 1:
            out, n = r, n + 1
            print('pod top flake filled: face %.1f mm2 at (%.1f, %.1f, %.1f), %.2f mm under it' % (f.Area(), c.x, c.y, c.z, th))
    print('pod top flakes filled:', n)
    return out


def build_side_pods():
    right = _build_side_pod_right()
    left = right.mirror("YZ").clean()
    assert left.isValid() and len(left.Solids()) == 1, "Left side pod invalid"
    return right, left


# =================================================================================================
# 4. MATCHING ARMORED ROOF PANEL (flat outer panels + steeper-walled center power-bulge)
# =================================================================================================
def build_armored_roof(A=(60.9, -86.9, 196.9), B=(45.9, 15.0, 219.9), half=3.5):
    """Drop-in replacement for parts/roof_plate.py on Fugu V2's open-cockpit roof frame, matching
    the hood's exact faceted design language: flatter outer panels + steeper-walled center bulge.

    Exact Fugu V2 open-cockpit roof rail centerlines (from renders/v2_build8.log):
      A (front): (+-60.9, -86.9, 196.9),  B (rear): (+-45.9, 15.0, 219.9),  tube half = 3.5 mm.
    Underside sits 4.10 mm above the A-B tube centerlines (clears headers + roof X by 0.56 mm).
    """
    A = np.array(A, float)      # roof rail nodes from the cage (parts/cage.py cabin_nodes); the defaults are the source's numbers
    B = np.array(B, float)
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
            bolt = cq.Solid.makeCylinder(SCREW_D / 2.0, 25.0, cq.Vector(*p_hole), cq.Vector(*n_dir))
            roof = roof.cut(bolt)

    roof = roof.clean()
    assert roof.isValid() and len(roof.Solids()) == 1, f"Armored roof invalid ({len(roof.Solids())} solids)"
    return roof




# =================================================================================================
# Fit to the built model: trim against neighbours, screw holes at the cage bosses
# =================================================================================================
def fit_cut(part, neighbours, shifts, min_volume=20.0):
    """Cuts every neighbour out of the part, also shifted by each vector in shifts (the clearance, towards the part).
    Loose crumbs under min_volume are dropped. The result must stay one valid solid."""
    out = part
    for nb in neighbours:
        for d in [(0.0, 0.0, 0.0)] + list(shifts):
            out = out.cut(nb.translate(cq.Vector(*d)) if any(d) else nb)
    solids = sorted(out.Solids(), key=lambda q: -q.Volume())
    for q in solids[1:]:
        b = q.BoundingBox()
        print('fit cut: loose piece dropped, mm3 %.1f at x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % (q.Volume(), b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax))
    lost = sum(q.Volume() for q in solids[1:])
    assert lost < max(min_volume, 0.01 * solids[0].Volume()), 'fit cut split the part: %.1f mm3 in %d loose pieces' % (lost, len(solids) - 1)
    out = solids[0].clean()
    assert out.isValid(), 'fit cut left an invalid solid'
    return out


def screw_hole(part, p, axis, depth=60.0, d=SCREW_D):
    """Clearance hole from p along axis through the panel."""
    return part.cut(cq.Solid.makeCylinder(d / 2.0, depth, cq.Vector(*p), cq.Vector(*axis)))


def skin_x(part, y, z, sx=1, r=0.5):
    """Inner and outer x of the panel's skin at (y, z) on side sx, or None."""
    h = part.intersect(cq.Solid.makeBox(120.0, 2 * r, 2 * r, cq.Vector(0.0 if sx > 0 else -120.0, y - r, z - r)))
    if h.Volume() < 1e-6:
        return None
    b = h.BoundingBox()
    return (b.xmin, b.xmax) if sx > 0 else (b.xmax, b.xmin)


def underside_z(part, x, y, r=0.5):
    """Lowest z of the panel above (x, y), or None."""
    h = part.intersect(cq.Solid.makeBox(2 * r, 2 * r, 200.0, cq.Vector(x - r, y - r, 0.0)))
    return h.BoundingBox().zmin if h.Volume() > 1e-6 else None


# =================================================================================================
# Side covers over the strip between the pods' top edge and the cage (owner, November 24: "add coverage where the yellow squiggle lines
# are. make the cover meet up to the cage"). Front (y -140 .. 46, up to 29 mm wide): a separate cover per side on the upper belt rail.
# Rear (y 48 .. tail, 5 .. 15 mm): the pod's top edge carried inward as a flange with a 45 deg underside. NOTES.md decision 25.
# =================================================================================================
COVER_Y = (-140.0, -128.0, -104.0, -80.0, -56.0, -40.0, -16.0, 8.0, 32.0, 40.0, 46.0)   # stations of the cover (the pod's top edge kinks at -104, -56, -40)
COVER_NODE_Y = 38.0            # behind the rear upright the cover goes over the node's top (probed), ahead of it over the rail (analytic)
COVER_RAIL_LAP = 3.5           # mm: the cover's inner edge lies this far inboard of the rail's centre line (it covers the rail's outer half)
COVER_SCREWS_Y = (-132.0, -100.0, 44.0)   # mm: M3 down into the rail / node top. The only spots with a free vertical tool path (MEASURED: the roof
                                          # panel covers y -92 .. 17, the B pillar and tail tubes the rest)
COVER_PILOT_DEPTH = 8.0        # mm pilot into the tube (no longer used: option (c), NOTES 31)
COVER_POD_SCREWS_Y = (-100.0, -20.0, 30.0)   # mm: cover screws from above into bosses inside the pod's top edge (tool path checked in the build)
COVER_BOSS = (7.0, 10.0, 3.0)  # mm: boss diameter, depth under the pod top, inset inboard of the skin's inner face
COVER_LIFT = 3                # mm: side cover path (orchestrator, November 25): the pods stay on; the cover goes up COVER_LIFT, then outward
                              # (removal; install is the reverse). 3 mm: ten times the 0.3 mm fit gap; at 6 mm and more the outward slide hits
                              # the cage at the front upright (x 66.5..67.7, y -138.4..-133.5, z 153.9..155.6, MEASURED), at 0 .. 5 it is clear
COVER_OUT = 40                # mm outward in x (tools/assembly_path.py). Members the cover meets on the lift get a through-notch (2D outline)
# Order: pods on first, then the side covers (a pod slid in under a fitted cover hits it after 6 mm); the pods come off only after the covers.
COVER_BOSS_DROP = 1.0         # mm: not used (thickness 59: a top under the land left a 1.0 mm lip; the boss is now clipped to the land plane)
COVER_BOSS_REACH = 8.0        # mm: each cover screw boss is stretched this far outward into the pod wall (clipped to the pod's outside): a plain
                              # cylinder touched the leaning inner skin only at its top (0.13 .. 4.5 mm gap below)
POD_SLIDE = 30                 # mm: the pods' sideways install path (tools/assembly_path.py)
FLANGE_Y = (48.0, 56.0, 64.0, 72.0, 80.0, 88.0, 96.0, 104.0, 112.0, 120.0, 128.0)
FLANGE_EMBED = 1.0             # mm the flange starts this far inside the pod skin's top edge
FLANGE_BAND = 25.0             # mm below the pod's top edge in which the flange looks for the cage tube it meets (the plate reaches 22 mm down)
FLANGE_DROP = 0.15             # mm the flange's top runs under the pod's top land: at the land's height it poked through the sloped land (sliver
                               # strip, thickness 54); 0.5 mm lower left the land's inner edge as a 0.62 mm lip over a V-gap (section y 80, thickness 57)
FLANGE_SEAT = 2.5              # mm the flange's flat top runs outward under the land from the land's inner edge (fills the lip's underside)
FLANGE_SAMPLE = 1.0            # mm: the cage's reach is sampled this often along each span; the span's inner edge is straight at the maximum + FIT_CUT
# The flange is a 3 mm plate at 45 deg (prints upright without support), not a solid wedge: the wedge weighed 12.7 g per pod (CG +0.3 mm).
FLANGE_DEFAULT_X = 41.0        # mm inner edge where no cage tube is found next to it


def top_slab(part, y, x0, x1, z0=100.0, z1=250.0, dz=1.0):
    """(xmin, xmax, zmax) of the topmost dz of the part in the slice at y between x0 and x1, or None."""
    h = part.intersect(cq.Solid.makeBox(x1 - x0, 0.6, z1 - z0, cq.Vector(x0, y - 0.3, z0)))
    if h.Volume() < 1e-6:
        return None
    zt = h.BoundingBox().zmax
    t = h.intersect(cq.Solid.makeBox(x1 - x0, 0.6, dz, cq.Vector(x0, y - 0.3, zt - dz))).BoundingBox()
    return t.xmin, t.xmax, zt


def band_xmax(part, y, x0, x1, z0, z1):
    h = part.intersect(cq.Solid.makeBox(x1 - x0, 0.6, z1 - z0, cq.Vector(x0, y - 0.3, z0)))
    return h.BoundingBox().xmax if h.Volume() > 1e-6 else None


def strip_solid(sections, thickness=None):
    """Faceted plate or flange from sections [(y, [(x, z), ...]), ...]. With thickness: the points are the top line, the plate is that
    thick below it (vertical offset); without: each section is a closed polygon. Neighbouring sections are joined facet by facet (convex hulls)."""
    out = None
    for (y0, p0), (y1, p1) in zip(sections, sections[1:]):
        pieces = []
        if thickness:
            for j in range(len(p0) - 1):
                q = [(x, y0, z) for x, z in (p0[j], p0[j + 1])] + [(x, y1, z) for x, z in (p1[j], p1[j + 1])]
                pieces.append(q + [(x, y, z - thickness) for x, y, z in q])
        else:
            pieces.append([(x, y0, z) for x, z in p0] + [(x, y1, z) for x, z in p1])
        for q in pieces:
            h = _hull(q)
            out = h if out is None else out.fuse(h)
    return out.clean()
