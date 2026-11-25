"""Edge chamfers on the printed moving parts (owner, November 24: "make sure all mechanically moving parts like tie rods, knuckles, pivots, etc,
please make sure to apply chamfers to them. all except gears of course"). NOTES decision 30.

One size for every edge: CHAMFER = 0.5 mm. It lies in both ranges asked for (outside edges 0.5 .. 1.0, bore / pocket / insert entries
0.3 .. 0.5) and takes the bed-face edges' elephant foot. A chamfer only cuts an edge: no fit diameter changes (arc slot 8.3, hub bore 6.4,
insert holes), and parts/swap_interface.py is untouched. Gear and pulley teeth are never chamfered (the pulleys are left out whole).
If OpenCascade refuses the whole set, the size steps down (FALLBACK), then edge by edge; an edge no size works on stays sharp and is counted.
"""
import cadquery as cq

CHAMFER = 0.5                  # mm, every edge
FALLBACK = (0.4, 0.3)          # smaller sizes tried when the full set fails
MIN_EDGE = 1.2                 # mm: edges shorter than this are left sharp (a 0.5 chamfer would eat them)
SHARP_DEG = 50.0               # only real corners: the faces turn by this angle or more (the truss tubes are octagons whose 45 deg faces are chamfers already; hull facet lines and seams are not corners)


def corners(s):
    """Convex edges between two different faces that turn by SHARP_DEG or more (normals at the edge's middle). Concave edges are left:
    a chamfer there adds material (it would close a pocket floor or a bore bottom)."""
    from OCP.TopExp import TopExp
    from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
    import math
    m = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(s.wrapped, TopAbs_EDGE, TopAbs_FACE, m)
    out = []
    for i in range(1, m.Extent() + 1):
        e = cq.Edge(m.FindKey(i)); fl = list(m.FindFromIndex(i))
        fs = []
        for f in fl:
            ff = cq.Face(f)
            if not any(ff.isSame(g) for g in fs):
                fs.append(ff)
        if len(fs) != 2:
            continue                                   # seam of a closed surface: no corner
        p = e.positionAt(0.5)
        try:
            n1, n2 = fs[0].normalAt(p), fs[1].normalAt(p)
        except Exception:
            continue
        ang = math.degrees(math.acos(max(-1.0, min(1.0, n1.dot(n2)))))
        if ang < SHARP_DEG:
            continue
        # convexity by geometry (no OCC chamfer call here: a trial chamfer per edge crashed the build): d = the direction in face A away
        # from the edge; the corner is convex if face B's normal points against d (a cube edge), concave if along it (a pocket floor)
        try:
            t = e.tangentAt(0.5)
            d = n1.cross(t)
            if d.Length < 1e-9:
                continue
            d = d.normalized()
            v1, v2 = cq.Vertex.makeVertex(*(p + d * 0.05).toTuple()), cq.Vertex.makeVertex(*(p - d * 0.05).toTuple())
            if fs[0].distance(v2) < fs[0].distance(v1):
                d = d * -1.0
            if n2.dot(d) >= 0:
                continue                               # concave: a chamfer there would ADD material
        except Exception:
            continue
        out.append(e)
    return out


def _ok(s, v0):
    return s is not None and s.isValid() and len(s.Solids()) == 1 and 0.9 * v0 < s.Volume() <= v0 + 1e-6


def _try(s, edges, size):
    try:
        r = s.chamfer(size, None, edges)
        return r if _ok(r, s.Volume()) else None
    except Exception:
        return None


def radii(s):
    """Set of cylinder radii (0.01 mm) of the part: a chamfer may shorten a cylinder but never removes or resizes one."""
    out = set()
    for f in s.Faces():
        if f.geomType() == 'CYLINDER':
            try:
                out.add(round(f._geomAdaptor().Radius(), 2))
            except Exception:
                try:
                    from OCP.BRepAdaptor import BRepAdaptor_Surface
                    out.add(round(BRepAdaptor_Surface(f.wrapped).Cylinder().Radius(), 2))
                except Exception:
                    pass
    return out


def boss(x0, x1, yz, od, c=CHAMFER):
    """Cylinder along x with both end edges chamfered (source-level chamfer for bosses)."""
    cyl = cq.Solid.makeCylinder(od / 2, x1 - x0, cq.Vector(x0, yz[0], yz[1]), cq.Vector(1, 0, 0))
    try:
        r = cyl.chamfer(c, None, [e for e in cyl.Edges() if e.geomType() == 'CIRCLE'])
        return r if r.isValid() else cyl
    except Exception:
        return cyl


def entry_cones(xs, yz, d, c=CHAMFER):
    """Cutters that chamfer a bore of diameter d along x where it meets a face at each x in xs (both directions)."""
    out = []
    for x in xs:
        for sd in (1, -1):
            out.append(cq.Solid.makeCone(d / 2 + c, d / 2, c, cq.Vector(x - sd * 0.0, yz[0], yz[1]), cq.Vector(sd, 0, 0)))
    return out


def chamfer_part(s, name='', size=CHAMFER, skip=None):
    """-> (chamfered solid, report dict). Never returns a broken solid: on total failure the part comes back unchanged."""
    v0 = s.Volume()
    edges = [e for e in corners(s) if e.Length() >= MIN_EDGE and not (skip and skip(e))]
    r0 = radii(s)
    for sz in (size,) + FALLBACK:
        r = _try(s, edges, sz)
        if r is not None:
            return r, dict(name=name, size=sz, edges=len(edges), skipped=len(s.Edges()) - len(edges), sharp=0, dvol=round(v0 - r.Volume(), 1), fits_kept=r0 <= radii(r))
    # recursive halving: a refused set is split and each half that works is applied, down to single edges; the refused ones get the
    # smaller sizes the same way. Edges are found again by position after each cut (neighbouring chamfers shorten them).
    keys = [(e.Center(), e.Length()) for e in edges]
    cur, sizes = s, []
    for sz in (size,) + FALLBACK:
        cur, keys, done = _halve(cur, keys, sz)
        if done:
            sizes.append(sz)
        if not keys:
            break
    return cur, dict(name=name, size=min(sizes) if sizes else 0.0, edges=len(edges), skipped=len(s.Edges()) - len(edges), sharp=len(keys),
                     dvol=round(v0 - cur.Volume(), 1), fits_kept=r0 <= radii(cur))


def _match(cur, keys):
    out = []
    for c0, l0 in keys:
        cand = [f for f in cur.Edges() if (f.Center() - c0).Length < 0.8 and abs(f.Length() - l0) < 1.2]
        if cand:
            out.append(min(cand, key=lambda q: (q.Center() - c0).Length))
    return out


def _halve(cur, keys, sz):
    """-> (solid, keys that failed, number chamfered)."""
    edges = _match(cur, keys)
    if not edges:
        return cur, [], 0
    r = _try(cur, edges, sz)
    if r is not None:
        return r, [], len(edges)
    if len(keys) == 1:
        return cur, keys, 0
    m = len(keys) // 2
    cur, f1, n1 = _halve(cur, keys[:m], sz)
    cur, f2, n2 = _halve(cur, keys[m:], sz)
    return cur, f1 + f2, n1 + n2
