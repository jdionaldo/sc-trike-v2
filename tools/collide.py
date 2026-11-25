"""Check the built STEP assembly, including every nested component.

Run after `soma build`: python -u tools/collide.py
No model rerun, fake parameters, hidden frame hull, or skipped boolean failures.
"""
from pathlib import Path
import itertools
import json
import sys
import cadquery as cq
from soma import import_assembly

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
SOURCE = ROOT / 'output/model.step'
REPORT = ROOT / 'renders/interference-report.json'
MIN_VOLUME = 0.01  # mm3: reject numerical contact dust only
OWN_GROUPS = ('Print_shell/', 'Lights/', 'Chassis/b_steer', 'Rear_suspension/Rear_shock_bridge', 'Cage_')


def boxes_overlap(a, b):
    return all(getattr(a, axis+'max') >= getattr(b, axis+'min') and
               getattr(b, axis+'max') >= getattr(a, axis+'min') for axis in 'xyz')


def main():
    asm = import_assembly(str(SOURCE))
    parts = []
    for name, node in asm.objects.items():
        if node.obj is None:
            continue
        shape = node.obj.val() if isinstance(node.obj, cq.Workplane) else node.obj
        shape = shape.moved(node.loc)
        parts.append((name, shape, shape.BoundingBox(), name.startswith(OWN_GROUPS)))
    print(f'Checking {len(parts)} components in the built assembly', flush=True)
    hits, errors = [], []
    checked = 0
    for (an, a, ab, own_a), (bn, b, bb, own_b) in itertools.combinations(parts, 2):
        if not (own_a or own_b) or not boxes_overlap(ab, bb):
            continue
        checked += 1
        try:
            overlap = a.intersect(b)
            volume = overlap.Volume()
            if volume <= MIN_VOLUME:
                continue
            bounds = overlap.BoundingBox()
            hit = dict(a=an, b=bn, volume_mm3=round(volume, 4),
                       bounds_mm=[round(getattr(bounds, k), 3) for k in
                                  ('xmin','xmax','ymin','ymax','zmin','zmax')])
            hits.append(hit)
            print(f'{volume:.3f} mm3: {an} <> {bn}', flush=True)
        except Exception as exc:
            errors.append(dict(a=an, b=bn, error=str(exc)))
            print(f'FAILED: {an} <> {bn}: {exc}', flush=True)
    hits.sort(key=lambda h: h['volume_mm3'], reverse=True)
    report = dict(step=str(SOURCE.relative_to(ROOT)), source_mtime=SOURCE.stat().st_mtime,
                  components=len(parts), boolean_pairs=checked,
                  overlap_threshold_mm3=MIN_VOLUME, overlaps=hits, errors=errors,
                  limits='Static ride-position geometry; intended mounting overlaps require review. '
                         'Imported mock wheels and other low-detail donor components remain approximations.')
    REPORT.parent.mkdir(exist_ok=True)
    REPORT.write_text(json.dumps(report, indent=2)+'\n')
    print(f'{len(hits)} overlapping pairs; {len(errors)} failed checks. Report: {REPORT}', flush=True)
    if errors:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
