"""Plain front deck: one flat slab in the plane of the old front deck, without its mounting holes, pockets, sockets, ears and recesses.

The slab keeps the old deck's top and bottom planes (found as the planes carrying the most face area with a normal close to +-z) and its plan outline
(sampled along y, so the bevelled front corners stay). Its rear end is a butt seam on the plane of the rear deck's front face.
The skid is screwed under its front edge with a stepped lap (lap_skid). Frame: whatever frame the given deck solid is in.
"""
import cadquery as cq

V = cq.Vector

SKID_SCREWS = ((26.0, 6.05), (-26.0, 6.05))   # M3 self-tapping screws from below through the flange (x, y behind the deck's front edge)
SKID_SCREW_D, SKID_PILOT_D = 3.4, 2.6


def _half_space(n, c, big=400.0):
    """A thick slab on the material side of the plane through c whose outward normal is n."""
    return cq.Workplane(cq.Plane(origin=c, xDir=V(1, 0, 0), normal=n * -1.0)).rect(big, big).extrude(big).val()


def _main_plane(deck, sign):
    """(normal, point) of the plane with the largest total face area whose normal points to sign * z."""
    bins = {}
    for f in deck.Faces():
        c = f.Center()
        try:
            n = f.normalAt(c)
        except Exception:
            continue
        if n.z * sign < 0.98: continue
        key = (round(n.x, 2), round(n.y, 2), round(n.dot(c), 0))
        a, _, _ = bins.get(key, (0.0, None, None))
        bins[key] = (a + f.Area(), n, c)
    assert bins, 'front deck has no flat face toward %+d z' % sign
    a, n, c = max(bins.values(), key=lambda q: q[0])
    return n, c


def make_flat_deck(deck, rear=None):
    """rear: the rear deck solid. The slab then ends on the plane of its front face (a butt seam; cutting the rear deck out of a slab with a
    vertical end left a 0.4 mm lip over the seam, because the front deck lies 0.9 mm higher)."""
    bb = deck.BoundingBox()
    nt, ct = _main_plane(deck, 1)
    nb, cb = _main_plane(deck, -1)
    # plan outline: half width at stations along y (1 mm steps over the first and last 20 mm, where the corners are bevelled)
    ys = [bb.ymin + 0.05] + [bb.ymin + k for k in range(1, 21)] + [bb.ymax - k for k in range(20, 0, -1)] + [bb.ymax - 0.05]
    half = []
    for y in ys:
        sec = deck.intersect(cq.Solid.makeBox(bb.xlen + 2, 0.1, bb.zlen + 2, V(bb.xmin - 1, y - 0.05, bb.zmin - 1)))
        if sec.Volume() > 1e-6:
            sb = sec.BoundingBox(); half.append((min(max(abs(sb.xmin), abs(sb.xmax)), 54.5), y))
    if rear is not None: half.append((half[-1][0], bb.ymax + 3.0))       # runs past the seam plane, which cuts the end face
    pts = half + [(-x, y) for x, y in reversed(half)]
    prism = cq.Workplane('XY', origin=(0, 0, bb.zmin - 20)).polyline(pts).close().extrude(bb.zlen + 40).val()
    slab = prism.intersect(_half_space(nt, ct)).intersect(_half_space(nb, cb)).clean()
    if rear is not None:
        ff = max((f for f in rear.Faces() if f.geomType() == 'PLANE' and f.normalAt(f.Center()).y < -0.9 and f.Center().y < rear.BoundingBox().ymin + 2.0),
                 key=lambda f: f.Area())
        slab = slab.intersect(_half_space(ff.normalAt(ff.Center()) * -1.0, ff.Center())).clean()
        assert slab.intersect(rear).Volume() < 0.01, 'flat front deck overlaps the rear deck'
    assert slab.isValid() and len(slab.Solids()) == 1, 'flat front deck is not one valid solid'
    t = (ct - cb).dot(nt)
    print('flat front deck: thickness %.2f mm, top normal (%.3f, %.3f, %.3f), volume %.0f -> %.0f mm3' % (t, nt.x, nt.y, nt.z, deck.Volume(), slab.Volume()))
    assert 2.0 < t < 20.0, 'front deck planes not found (thickness %.2f)' % t
    return slab


def lap_skid(deck, skid):
    """Stepped lap between the skid and the flat deck. The skid stand-in has a 3 mm mounting flange that runs under the deck's front edge. The deck
    lies 0.9 mm above the flange's underside, so cutting the plain slab out of the skid left a 0.9 mm sheet with four old holes under the deck.
    The flange is rebuilt as a plain plate with the old footprint, underside and top, let into the underside of the deck by its upper 2.1 mm, with
    two M3 screws from below (clearance holes in the flange, pilot holes through the deck). Returns (deck, skid)."""
    nb, cb = _main_plane(deck, -1)
    yf = deck.BoundingBox().ymin
    sb = skid.BoundingBox()
    behind = cq.Solid.makeBox(sb.xlen + 2, sb.ymax - yf + 1.0, sb.zlen + 2, V(sb.xmin - 1, yf, sb.zmin - 1))
    old = skid.intersect(cq.Solid.makeBox(sb.xlen + 2, sb.ymax - yf, sb.zlen + 2, V(sb.xmin - 1, yf + 0.5, sb.zmin - 1)))
    assert old.Volume() > 10.0, 'skid has no flange behind the front edge of the deck'
    ob = old.BoundingBox(); hx = min(abs(ob.xmin), abs(ob.xmax))
    ns, cs = _main_plane(old, -1)                                  # underside and top of the flange
    nu, cu = _main_plane(old, 1)
    body = max(skid.cut(behind).Solids(), key=lambda q: q.Volume())
    # from 2.5 mm ahead of the deck's front edge (inside the root of the nose plate; also closes what is left there of two old holes on the edge)
    flange = cq.Solid.makeBox(2 * hx, ob.ymax - yf + 2.5, sb.zlen + 40, V(-hx, yf - 2.5, sb.zmin - 20)).intersect(_half_space(ns, cs)).intersect(_half_space(nu, cu))
    skid2 = body.fuse(flange).clean()
    deck2 = deck.cut(skid2).clean()                                # recess for the plain flange, before the holes are drilled
    for x, dy in SKID_SCREWS:
        skid2 = skid2.cut(cq.Solid.makeCylinder(SKID_SCREW_D / 2, sb.zlen + 40, V(x, yf + dy, sb.zmin - 20), V(0, 0, 1)))
    assert skid2.isValid() and len(skid2.Solids()) == 1, 'skid flange is not one valid solid'
    for x, dy in SKID_SCREWS:
        deck2 = deck2.cut(cq.Solid.makeCylinder(SKID_PILOT_D / 2, sb.zlen + 40, V(x, yf + dy, sb.zmin - 20), V(0, 0, 1)))
    assert deck2.isValid() and len(deck2.Solids()) == 1, 'skid lap broke the flat deck'
    t, lap = (cs - cu).dot(ns), (cb - cu).dot(nb)
    assert t > 2.0 and 1.0 < lap < t and (deck.Volume() - deck2.Volume()) > 500.0, 'skid flange does not lap the deck (flange %.2f, lap %.2f)' % (t, lap)
    print('skid lap: flange %.1f x %.1f mm, %.2f mm thick, %.2f mm of it in the deck; skid mm3 %.0f -> %.0f' % (2 * hx, ob.ymax - yf, t, lap, skid.Volume(), skid2.Volume()))
    return deck2, skid2
