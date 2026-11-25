import numpy as np
import cadquery as cq
from scipy.spatial import ConvexHull


def hull(pts):
    """Low-poly solid: convex hull of points, coplanar triangles merged into planar facets."""
    P = np.array(pts, float)
    h = ConvexHull(P)
    groups = {}
    for eq, simp in zip(h.equations, h.simplices):
        # Only merge truly coplanar triangles. Coarse rounding merged shallow
        # folds into nonplanar wires and could also distort clearance cutters.
        groups.setdefault(tuple(np.round(eq, 8)), set()).update(int(i) for i in simp)
    faces = []
    for key, idx in groups.items():
        n = np.array(key[:3]); n /= np.linalg.norm(n)
        Q = P[sorted(idx)]
        c = Q.mean(0)
        u = np.cross(n, [0, 0, 1.0] if abs(n[2]) < 0.9 else [1.0, 0, 0]); u /= np.linalg.norm(u)
        v = np.cross(n, u)
        order = np.argsort(np.arctan2((Q - c) @ v, (Q - c) @ u))
        w = cq.Wire.makePolygon([cq.Vector(*q) for q in Q[order]], close=True)
        faces.append(cq.Face.makeFromWires(w))
    solid = cq.Solid.makeSolid(cq.Shell.makeShell(faces))
    if solid.Volume() < 0:
        solid = cq.Solid(solid.wrapped.Reversed())
    return solid.clean()


def beam(a, b, r):
    a, b = cq.Vector(*a), cq.Vector(*b)
    d = b - a
    return cq.Solid.makeCylinder(r, d.Length, a, d.normalized())


def ball(p, r):
    return cq.Workplane("XY").sphere(r).val().translate(p)


def fuse(shapes):
    shapes = list(shapes)
    return shapes[0].fuse(*shapes[1:]).clean() if len(shapes) > 1 else shapes[0]


def tubes(path_pairs, r, joints=True):
    """Tubes between point pairs, with ball joints at the ends."""
    parts = [beam(a, b, r) for a, b in path_pairs]
    if joints:
        seen = set()
        for a, b in path_pairs:
            for p in (a, b):
                if tuple(p) not in seen:
                    seen.add(tuple(p)); parts.append(ball(p, r * 1.15))
    return fuse(parts)


def mx(p):
    return (-p[0], p[1], p[2])


def mirror_x(shape):
    return shape.mirror("YZ")


# Numeric weld tolerance only: intentional armor folds stay separate planes.
WELD_LINEAR = 1e-5
WELD_ANGLE = 1e-6

def weld(s, name):
    """Unify contiguous coplanar faces and verify one closed material body."""
    from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
    volume=s.Volume()
    unify=ShapeUpgrade_UnifySameDomain(s.copy().wrapped,True,True,True)
    unify.SetLinearTolerance(WELD_LINEAR)
    unify.SetAngularTolerance(WELD_ANGLE)
    unify.Build()
    result=cq.Shape.cast(unify.Shape())
    # OCC can reshape narrow folds while unifying transformed faces. Retain
    # the already fused/cleaned original when that alters the material volume.
    if (not result.isValid() or len(result.Solids())!=len(s.Solids())
            or len(result.Shells())!=len(s.Shells())
            or abs(result.Volume()-volume)>=max(0.001,volume*1e-7)):
        result=s
    assert result.isValid() and len(result.Solids())==1, name+': weld must remain one solid'
    from OCP.BRep import BRep_Tool
    assert len(result.Shells())==len(s.Shells()), name+': weld changed internal voids'
    assert all(BRep_Tool.IsClosed_s(sh.wrapped) for sh in result.Shells()), name+': open boundary shell'
    assert abs(result.Volume()-volume)<max(0.001,volume*1e-7), name+': weld changed material volume'
    return result


def clean_registered(obj):
    """Remove coplanar subdivision seams throughout a part or nested assembly.

    Keep separate bodies and designed folds. Never accept a cleanup that changes
    volume, validity, or body count. Imported hardware keeps its original fit.
    """
    if isinstance(obj,cq.Assembly):
        seen=set()
        for node in obj.objects.values():
            if node.obj is None or id(node) in seen:
                continue
            seen.add(id(node))
            node.obj=clean_registered(node.obj)
        return obj
    shape=obj.val() if isinstance(obj,cq.Workplane) else obj
    if not isinstance(shape,cq.Shape) or not shape.isValid():
        return obj
    try: candidate=shape.clean()
    except Exception: return obj   # OCC can raise on near-tangent faces: register the part uncleaned
    if (candidate.isValid() and len(candidate.Solids())==len(shape.Solids())
            and len(candidate.Shells())==len(shape.Shells())
            and abs(candidate.Volume()-shape.Volume())<max(0.001,shape.Volume()*1e-7)):
        return candidate
    return obj


CHAMFER = 0.5   # mm: uniform edge break on every external convex edge of the printed parts


def chamfer_convex(solid, d=CHAMFER, min_turn=50.0, min_len=3.0, label='', chunk=40):
    """Chamfer every outside (convex) edge of a printed part by d mm.
    Skips edges that turn less than min_turn degrees (soft facet creases and the 45 degree flats of the square tubes, which are already
    chamfered), edges shorter than min_len, and edges on curved faces (holes, bosses, bores). Never touches concave edges.
    Tries all edges together, then in chunks, then one by one; whatever the kernel rejects is left sharp and reported. Returns (solid, n_done, n_total)."""
    import math
    import cadquery as cq
    faces_of = {}
    for f in solid.Faces():
        for e in f.Edges(): faces_of.setdefault(e.hashCode(), []).append(f)
    picked = []
    for e in solid.Edges():
        fs = faces_of.get(e.hashCode(), [])
        if len(fs) != 2 or e.Length() < min_len: continue
        if any(f.geomType() not in ('PLANE',) for f in fs): continue
        mid = e.positionAt(0.5)
        n1, n2 = fs[0].normalAt(mid), fs[1].normalAt(mid)
        turn = math.degrees(math.acos(max(-1.0, min(1.0, n1.dot(n2)))))
        if turn < min_turn: continue
        eps = 0.05
        if solid.isInside(mid + (n1 - n2) * eps) or solid.isInside(mid + (n2 - n1) * eps): continue     # not convex
        picked.append(e)
    total = len(picked); done = 0

    def ok(s): return s is not None and s.isValid() and len(s.Solids()) == 1

    def apply(s, edges):
        try:
            r = s.chamfer(d, None, edges)
            return r if ok(r) else None
        except Exception:
            return None
    # edges of a solid change identity after each chamfer, so re-pick by position when working in chunks
    r = apply(solid, picked)
    if r is not None:
        print('chamfer %s: %d/%d edges at once' % (label, total, total)); return r, total, total
    cur = solid; keys = [(tuple(round(c, 2) for c in e.positionAt(0.5).toTuple()), e.Length()) for e in picked]
    def find(s, key):
        best = None
        for e in s.Edges():
            if abs(e.Length() - key[1]) < 0.01:
                p = e.positionAt(0.5)
                if (abs(p.x - key[0][0]) + abs(p.y - key[0][1]) + abs(p.z - key[0][2])) < 0.03: return e
        return best
    for i in range(0, len(keys), chunk):
        es = [find(cur, k) for k in keys[i:i + chunk]]; es = [e for e in es if e is not None]
        r = apply(cur, es)
        if r is not None: cur = r; done += len(es); continue
        for k in keys[i:i + chunk]:
            e = find(cur, k)
            if e is None: continue
            r = apply(cur, [e])
            if r is not None: cur = r; done += 1
    print('chamfer %s: %d/%d edges (rest left sharp)' % (label, done, total))
    return cur, done, total
