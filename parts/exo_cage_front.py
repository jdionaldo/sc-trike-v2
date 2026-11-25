"""Front exo frame and V bull bar of the desert shell (owner's design, desert_shell_variant/exo_cage.py, ported November 24).

Not a part of its own: the members go into the cage's node graph (parts/cage.py make_cage, exo=...), so every joint is one of the cage's
convex hull nodes and the frame is printed with the cage (@tube-cage-frame: the frame is a graph, tubes that meet share the node).
The source tied into cage nodes that this cage does not have (front hoop corners at z 128; the hoop is at z 101 here) and doubled the front
upright and the hoop-to-belt tube. Here the hoop corner is raised to the source's height and the doubled members are left out.

Lower front (owner, November 24: "line up the lower front cage with the skid"): the lower members follow the skid plate. A rail runs beside each
side edge of the skid, parallel to the plate, its bottom face in the plate's bottom plane and SKID_GAP outboard of the edge; the V cross bar is
the skid's front edge (the skid is shortened to end SKID_GAP behind it); a post from each rail's rear end, just ahead of the deck, goes up into
the lower side rail. Every node comes from the skid's outline (skid_outline), measured on the built skid, none is typed.
"""
import math
from parts.cage import CABIN_BEAM, CAGE_Z, CAGE_RAIL_X, RAIL_START_Y, FRONT_LEG_X, _half

EXO_D = CABIN_BEAM            # mm: the cage's second tube size (7); the source had 6.4 and 7.0 (@tube-cage-frame: two sizes, no more)
HOOP_Z = 128.0                # mm: front hoop corner in the desert style (source: 128; 101 with the stealth shell)
NOSE_TOP = (43.5, -290.0, 114.0)     # blunt prow: front end of the upper rails
V_MID = (38.5, -268.0, 75.0)         # chin corner of the V bull bar, under the grille (kept: moving it onto the skid line puts its cross bar into the tub's chin)
SKID_GAP = 0.4                       # mm between the skid and the rails beside it, and between the V bar and the skid's new front end
SIDE_KNEE = (44.0, -212.0, 96.0)     # knee of the swept side bars
CROSS_Z = 112.0                      # mm: cross ties under the hood, at the height of the cage's brace landing on the front upright
MID_BAR = False                      # the source's mid-hood cross tie at y -212: off, the hoop node is 16 mm above it and the tub's top edge under it


def _on(a, b, z):
    t = (z - a[2]) / (b[2] - a[2])
    return tuple(round(a[i] + t * (b[i] - a[i]), 1) for i in range(3))


def skid_outline(skid):
    """Half width and the four corners (y, z) of the skid plate's side edge, from its vertices at x = +W (world frame):
    bottom rear B_r (lowest), top rear T_r (rearmost), bottom front B_f (foremost), top front T_f (highest)."""
    W = skid.BoundingBox().xmax
    vs = [(v.Y, v.Z) for v in skid.Vertices() if abs(v.X - W) < 0.05]
    return dict(W=W, B_r=min(vs, key=lambda p: p[1]), T_r=max(vs, key=lambda p: p[0]),
                B_f=min(vs, key=lambda p: p[0]), T_f=max(vs, key=lambda p: p[1]))


def skid_nodes(sk, deck_front_y):
    """Lower front nodes (+x side) from the skid outline: C front corner (V bar end), R rail rear end (post foot), T post top on the lower
    side rail. Also the unit vectors d (along the plate, forward and up) and n (normal, to the top side), all in (y, z)."""
    (yr, zr), (yf, zf) = sk['T_r'], sk['T_f']
    ln = math.hypot(yf - yr, zf - zr)
    d = ((yf - yr) / ln, (zf - zr) / ln)
    n = (d[1], -d[0])
    if n[0] * (sk['T_r'][0] - sk['B_r'][0]) + n[1] * (sk['T_r'][1] - sk['B_r'][1]) < 0:
        n = (-n[0], -n[1])
    h = _half(EXO_D)
    x = round(sk['W'] + SKID_GAP + h, 2)                          # rail beside the side edge, SKID_GAP off it
    p0 = (sk['B_r'][0] + h * n[0], sk['B_r'][1] + h * n[1])      # rail centre line: bottom face in the plate's bottom plane
    at_y = lambda y: (round(y, 2), round(p0[1] + (y - p0[0]) / d[0] * d[1], 2))
    C = (x,) + at_y(sk['B_f'][0] + h)                            # V bar: front face level with the skid's old front edge
    R = (x,) + at_y(deck_front_y - SKID_GAP - h)                 # rail rear end: post SKID_GAP ahead of the deck's front edge
    L = (CAGE_RAIL_X, RAIL_START_Y, CAGE_Z)
    t = (R[1] - L[1]) / (V_MID[1] - L[1])
    T = tuple(round(L[i] + t * (V_MID[i] - L[i]), 2) for i in range(3))   # on the lower side rail, above R
    return dict(C=C, R=R, T=T, d=d, n=n, slope_deg=round(math.degrees(math.atan2(d[1], -d[0])), 1))


def members(front_y, belt_start_z, skid=None):
    """Centre lines (a, b, d) of the +x half and of the cross bars, for the cage frame. front_y: y of the hoop. skid: skid_nodes()."""
    H = (FRONT_LEG_X, front_y, HOOP_Z)                       # hoop corner (the cage builds hoop -> belt start, -> cowl corner and the brace itself)
    U = (CAGE_RAIL_X, RAIL_START_Y, belt_start_z)            # upper belt start
    L = (CAGE_RAIL_X, RAIL_START_Y, CAGE_Z)                  # lower rail start
    C, R, T = skid['C'], skid['R'], skid['T']
    side = [(H, NOSE_TOP), (NOSE_TOP, V_MID), (V_MID, C),              # upper rail and V bull bar down to the skid's front corner
            (U, SIDE_KNEE), (SIDE_KNEE, V_MID), (H, SIDE_KNEE),        # swept side bars
            (L, T), (T, V_MID),                                        # lower side rail (split at the post top)
            (C, R), (R, T)]                                            # skid rail beside the skid's side edge, post up into the side rail
    m = lambda p: (-p[0], p[1], p[2])
    cross = [(m(C), C), (m(V_MID), V_MID),
             (m((CAGE_RAIL_X, RAIL_START_Y, CROSS_Z)), (CAGE_RAIL_X, RAIL_START_Y, CROSS_Z))]    # firewall tie between the front uprights
    if MID_BAR:
        k = _on(H, SIDE_KNEE, CROSS_Z)
        cross.append((m(k), k))
    return [(a, b, EXO_D) for a, b in side], [(a, b, EXO_D) for a, b in cross]


def rail_point(front_y, y):
    """Centre of the upper exo rail (hoop corner to nose top) at station y."""
    H = (FRONT_LEG_X, front_y, HOOP_Z)
    t = (y - H[1]) / (NOSE_TOP[1] - H[1])
    return tuple(H[i] + t * (NOSE_TOP[i] - H[i]) for i in range(3))
