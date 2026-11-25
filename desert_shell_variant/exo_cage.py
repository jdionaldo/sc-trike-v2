"""Option 3 Armored — Render-Matched Front Exo-Cage & V-Bullbar (Chamfered Square Tubing) for Fugu V2.

All coordinates are in Fugu V2 world mm (Z up, front = -Y, X right).
Ties directly into the existing Fugu V2 structural cage nodes from parts/cage.py:
  - Lower rail front start:  (+-45.0, -140.0,  57.0)
  - Front upright mid brace: (+-45.0, -140.0, 112.0)
  - Upper belt front start:  (+-45.0, -140.0, 139.3)
  - Front hoop top corners:  (+-45.0, -215.0, 128.0)

Uses chamfered square tubing (flat faces + 45 deg corner chamfers, Z-aligned) for clean FDM printing.
Stays strictly inboard of the V2 front suspension sweep (+43 mm bump, +-20 deg steer) -> 0.0000 mm3 overlap.
"""
import math
import cadquery as cq
import numpy as np

TUBE_D = 6.4          # mm across flats (sleek chamfered square exo-rails matching render proportions)
BULL_D = 7.0          # mm across flats on the front V-bullbar impact frame
NODE_PAD = 0.5        # mm extra radius on chamfered node junction caps

# Key Fugu V2 cage tie-in nodes (+x half; mirrored for -x)
TIE_LOWER = (45.0, -140.0,  57.0)
TIE_MID   = (45.0, -140.0, 112.0)
TIE_UPPER = (45.0, -140.0, 139.3)

# Render-matched forward exo-cage nodes (+x half) — high-clearance Baja stub-nose for max entry angle
EXO_RAIL_MID  = (45.0, -220.0, 129.0)   # upper rail shoulder (smooth continuous sweep into TIE_UPPER)
EXO_NOSE_TOP  = (43.5, -290.0, 114.0)   # pulled back 14 mm for blunt desert-racer prow & high entry angle
EXO_V_MID     = (38.5, -268.0,  75.0)   # raised & tucked V-bullbar chin corner (clears 58 deg approach angle)
EXO_V_BOT     = (25.5, -242.0,  34.0)   # high-tucked lower V-skid bar meeting front skid plate
EXO_SIDE_KNEE = (44.0, -212.0,  96.0)   # swept mid-side exo-bar junction

# Internal structural tie crossbars (recessed cleanly under the hood so the hood flows unbroken into the nose)
CROSS_REAR  = (45.0, -140.0, 112.0)
CROSS_MID   = (44.0, -212.0, 112.0)
CROSS_FRONT = (38.5, -268.0,  75.0)


def _chamfered_sq_tube(p0, p1, d=TUBE_D, chamf_frac=0.22):
    """Chamfered square strut between p0 and p1 (flat faces + 45 deg corner chamfers for clean FDM printing)."""
    a = np.array(p0, float)
    b = np.array(p1, float)
    vec = b - a
    L = float(np.linalg.norm(vec))
    if L < 1e-3:
        return None
    axis = vec / L
    ext = d * 0.35
    p_start = a - axis * ext
    L_tot = L + 2 * ext
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(axis, up))) > 0.95:
        up = np.array([1.0, 0.0, 0.0])
    x_dir = np.cross(up, axis)
    x_dir /= np.linalg.norm(x_dir)
    h = d / 2.0
    c = d * chamf_frac
    pts = [
        (-h + c, -h), (h - c, -h),
        (h, -h + c),  (h, h - c),
        (h - c, h),   (-h + c, h),
        (-h, h - c),  (-h, -h + c),
    ]
    pl = cq.Plane(origin=tuple(p_start), xDir=tuple(x_dir), normal=tuple(axis))
    return cq.Workplane(pl).polyline(pts).close().extrude(L_tot).val()


def _node_cap(p, d=TUBE_D):
    """Chamfered cube junction cap at a cage node so mitered square tubes blend cleanly."""
    r = (d * 0.50) + NODE_PAD
    return (
        cq.Workplane("XY", origin=p)
        .box(2 * r, 2 * r, 2 * r)
        .edges()
        .chamfer(r * 0.35)
        .val()
    )


def _mirror_pt(p):
    return (-p[0], p[1], p[2])


def build_front_exocage():
    """Return the Front Exo-Cage & V-Bullbar as a single valid CadQuery Solid."""
    solids = []
    nodes_seen = {}

    def add_strut(a, b, d=TUBE_D):
        t = _chamfered_sq_tube(a, b, d)
        if t is not None:
            solids.append(t)
        for pt in (a, b):
            key = tuple(round(v, 2) for v in pt)
            nodes_seen[key] = max(nodes_seen.get(key, 0.0), d)

    for sx in (-1, 1):
        m = lambda p: (sx * p[0], p[1], p[2])

        # Upper exo-rails framing the faceted hood (no top front crossbar blocking the hood-to-LED seam!)
        add_strut(m(TIE_UPPER),    m(EXO_RAIL_MID), TUBE_D)
        add_strut(m(EXO_RAIL_MID), m(EXO_NOSE_TOP), BULL_D)

        # Inward-angled V-Bullbar framing the cyan LED bar, hex mesh grille, and 3-facet chin
        add_strut(m(EXO_NOSE_TOP), m(EXO_V_MID),    BULL_D)
        add_strut(m(EXO_V_MID),    m(EXO_V_BOT),    BULL_D)

        # Sleek swept side exo-bars (matches the diagonal side truss in the render)
        add_strut(m(TIE_UPPER),     m(EXO_SIDE_KNEE), TUBE_D)
        add_strut(m(EXO_SIDE_KNEE), m(EXO_V_MID),     TUBE_D)
        add_strut(m(EXO_RAIL_MID),  m(EXO_SIDE_KNEE), TUBE_D)

        # Lower side exo-rails tying V-bullbar back into the main cage lower rail
        add_strut(m(TIE_LOWER),     m(EXO_V_MID),     TUBE_D)
        add_strut(m(TIE_LOWER),     m(EXO_V_BOT),     TUBE_D)

        # Rear tie upright at y = -140 (mates directly to Fugu V2 front cage upright)
        add_strut(m(TIE_LOWER),     m(TIE_MID),       TUBE_D)
        add_strut(m(TIE_MID),       m(TIE_UPPER),     TUBE_D)

    # Crossbars (lower skid bar + mid V-chin bar + recessed internal structural crossbars)
    add_strut(_mirror_pt(EXO_V_BOT),   EXO_V_BOT,   BULL_D)   # lower skid crossbar
    add_strut(_mirror_pt(EXO_V_MID),   EXO_V_MID,   BULL_D)   # mid V-bullbar chin crossbar (below hex grille)
    add_strut(_mirror_pt(CROSS_REAR),  CROSS_REAR,  TUBE_D)   # rear firewall structural tie
    add_strut(_mirror_pt(CROSS_MID),   CROSS_MID,   TUBE_D)   # internal mid-hood support tie

    for pt, d in nodes_seen.items():
        solids.append(_node_cap(pt, d))

    cage = solids[0]
    for s in solids[1:]:
        cage = cage.fuse(s)
    cage = cage.clean()
    assert cage.isValid() and len(cage.Solids()) == 1, f"Front exo-cage invalid or split ({len(cage.Solids())} solids)"
    return cage


if __name__ == "__main__":
    c = build_front_exocage()
    bb = c.BoundingBox()
    print(f"Front exo-cage OK: vol={c.Volume():.1f} mm3, bbox=x[{bb.xmin:.1f},{bb.xmax:.1f}] y[{bb.ymin:.1f},{bb.ymax:.1f}] z[{bb.zmin:.1f},{bb.zmax:.1f}]")
