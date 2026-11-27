"""Seam and facet probe: how two skins meet across a joint, in numbers (owner, November 25: "make sure all joints and facets are seamlessly
blended like the shoulder to door skin of the shell"; NOTES 43).

Casts parallel rays (direction d) at the tessellated outer surface of the given parts along a line of start points, and reports for each
neighbouring pair of hits: the step (jump of the hit point along d, a ledge or gap shows here), the crease angle between the surface normals
(a facet edge shows here), which part each hit belongs to, and where the ray found nothing (an opening). Reads output/model.step.

    from seam_probe import probe
    probe(['Side_pod_right'], start=(120, -70, 125), end=(120, -40, 125), d=(-1, 0, 0), step=0.25)

Run as a script for the reference transition (pod shoulder to door skin): python -I -B tools/seam_probe.py
"""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'tools'))
import numpy as np

_CACHE = {}


def _parts():
    if 'P' not in _CACHE:
        import front2_pose as fp
        P = fp.load(ROOT / 'output' / 'model.step')
        _CACHE['P'] = {fp.leaf(k): s for k, s in P.items()}
    return _CACHE['P']


def _mesh(name, tol=0.05):
    key = ('m', name, tol)
    if key not in _CACHE:
        s = _parts()[name]
        vs, ts = s.tessellate(tol, 0.1)
        V = np.array([(v.x, v.y, v.z) for v in vs]); T = np.array(ts)
        _CACHE[key] = (V[T[:, 0]], V[T[:, 1]], V[T[:, 2]])
    return _CACHE[key]


def _hit(o, d, tri):
    """First hit of the ray o + t d (t > 0) on the triangles (Moller-Trumbore): t, unit normal (facing the ray), or None."""
    A, B, C = tri
    e1, e2 = B - A, C - A
    p = np.cross(d, e2); det = (e1 * p).sum(1)
    ok = np.abs(det) > 1e-12
    inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
    s = o - A
    u = (s * p).sum(1) * inv
    q = np.cross(s, e1)
    v = (q * d).sum(1) * inv
    t = (q * e2).sum(1) * inv
    m = ok & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (t > 1e-6)
    if not m.any():
        return None
    i = np.argmin(np.where(m, t, np.inf))
    n = np.cross(e1[i], e2[i]); n /= np.linalg.norm(n)
    if np.dot(n, d) > 0:
        n = -n
    return t[i], n


def probe(names, start, end, d, step=0.25, quiet=False, crease_min=1.0, step_min=0.05):
    """Rays along the line start..end in direction d against the parts `names`. Returns the list of (point index, s along the line, part, hit
    point, normal) and prints the creases, steps and openings found."""
    d = np.asarray(d, float); d /= np.linalg.norm(d)
    a, b = np.asarray(start, float), np.asarray(end, float)
    L = np.linalg.norm(b - a); n = max(2, int(L / step) + 1)
    meshes = {nm: _mesh(nm) for nm in names}
    rows = []
    for i in range(n):
        o = a + (b - a) * i / (n - 1)
        best = None
        for nm, tri in meshes.items():
            h = _hit(o, d, tri)
            if h and (best is None or h[0] < best[1]):
                best = (nm, h[0], h[1])
        rows.append((i, L * i / (n - 1), best[0] if best else None, (o + d * best[1]) if best else None, best[2] if best else None))
    out = []
    for (i0, s0, p0, h0, n0), (i1, s1, p1, h1, n1) in zip(rows, rows[1:]):
        if h0 is None or h1 is None:
            if (h0 is None) != (h1 is None):
                out.append('opening edge at s %.2f (%s -> %s)' % (s1, p0, p1))
            continue
        jump = float(np.dot(h1 - h0, d))
        ang = float(np.degrees(np.arccos(np.clip(np.dot(n0, n1), -1, 1))))
        if abs(jump) > step_min * 4 or ang > crease_min or p0 != p1:
            out.append('s %6.2f -> %6.2f  %-16s -> %-16s  step %+6.2f mm  crease %5.1f deg  at (%.1f, %.1f, %.1f)'
                       % (s0, s1, p0, p1, jump, ang, *h1))
    if not quiet:
        print('probe %s from %s to %s, rays %s, %d rays:' % (','.join(names), tuple(start), tuple(end), tuple(np.round(d, 2)), n))
        for l in out:
            print('   ' + l)
        if not out:
            print('   smooth: no step over %.2f mm, no crease over %.1f deg' % (step_min * 4, crease_min))
    return rows, out


if __name__ == '__main__':
    # reference: right side pod, shoulder (wing, ahead of y -54) to door skin (main flank), rays from outboard at several heights
    for z in (108.0, 115.0, 125.0, 135.0, 142.0):
        probe(['Side_pod_right'], start=(130.0, -75.0, z), end=(130.0, -35.0, z), d=(-1, 0, 0), step=0.25)
