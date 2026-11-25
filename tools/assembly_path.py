"""Assembly-path test for the removable and separately fitted shell parts (V2 orchestrator's standing check, November 25).

Each part moves along its install direction in 1 mm steps and must clear the cage and every other part at every step
(0 mm3 overlap, above 0.01 mm3 counts). Lift-off parts go straight up 30 mm; the pods first slide outward along x (their screws run along x)
30 mm, then up 30. The side covers (orchestrator, November 25): the pods stay on; the cover goes up COVER_LIFT (3 mm) and outward in x
by COVER_OUT (40 mm). Install is the reverse: pods on first, then each cover in from outside and down, then its screws from above into the
pods' bosses. The pods come off only after the covers.
Reads output/model.step. Run: python -I -B tools/assembly_path.py [part ...]   (default: every part in PATHS that is in the build)
Exit 1 if any part is blocked.
"""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'tools'))
import cadquery as cq
import front2_pose as fp
from parts import desert_shell as _ds

STEPS = 30
UP = [(0.0, 0.0, float(k)) for k in range(1, STEPS + 1)]
def _side_then_up(sx):
    return [(sx * float(k), 0.0, 0.0) for k in range(1, STEPS + 1)] + [(sx * STEPS, 0.0, float(k)) for k in range(1, STEPS + 1)]
def _up_then_side(sx, h=_ds.COVER_LIFT, out=_ds.COVER_OUT):
    return [(0.0, 0.0, float(k)) for k in range(1, h + 1)] + [(sx * float(k), 0.0, float(h)) for k in range(1, out + 1)]
PATHS = {
    'Side_cover_right': _up_then_side(1.0), 'Side_cover_left': _up_then_side(-1.0),
    'Side_cover_front_right': UP, 'Side_cover_front_left': UP, 'Side_cover_rear_right': UP, 'Side_cover_rear_left': UP,
    'Hood_deck': UP, 'Cowl_panel': UP, 'Dash_panel': UP, 'Roof_panel': UP,
    'Rear_cover_right': UP, 'Rear_cover_left': UP,
    'Side_pod_right': _side_then_up(1.0), 'Side_pod_left': _side_then_up(-1.0),
}
# install order: a part is tested against everything except parts that go on after it (they are off while it moves)
ORDER = {'Side_pod_right': 1, 'Side_pod_left': 1, 'Side_cover_right': 2, 'Side_cover_left': 2, 'Rear_cover_right': 2, 'Rear_cover_left': 2,
         'Side_cover_front_right': 2, 'Side_cover_front_left': 2, 'Side_cover_rear_right': 2, 'Side_cover_rear_left': 2,
         'Hood_deck': 3, 'Cowl_panel': 3, 'Dash_panel': 3, 'Roof_panel': 3}
SKIP = ('mock_tire', 'Rear_tire')   # tyres are not in the way of shell parts above the cage rail; wheels are kept


def main():
    parts = fp.load(ROOT / 'output' / 'model.step')
    by = {fp.leaf(k): k for k in parts}
    names = sys.argv[1:] or [n for n in PATHS if n in by]
    others_all = {k: s for k, s in parts.items() if not fp.leaf(k).startswith(SKIP)}
    bad = 0
    for n in names:
        s = parts[by[n]]
        others = [(k, o) for k, o in others_all.items() if k != by[n] and ORDER.get(fp.leaf(k), 0) <= ORDER.get(n, 0)]
        first = None
        for d in PATHS[n]:
            m = s.translate(cq.Vector(*d)); mb = fp.bb(m)
            for k, o in others:
                if not fp.bb_hit(mb, fp.bb(o)):
                    continue
                v = m.intersect(o).Volume()
                if v > 0.01:
                    first = (d, fp.short(k), v); break
            if first:
                break
        if first:
            bad += 1
            print('BLOCKED %-24s at step %s: %.2f mm3 into %s' % (n, tuple(round(c, 1) for c in first[0]), first[2], first[1]), flush=True)
        else:
            d = PATHS[n][-1]
            print('clear   %-24s %d steps (%s, ends at %s)' % (n, len(PATHS[n]), 'up 30 mm' if PATHS[n] is UP else 'two legs', tuple(round(c, 1) for c in d)), flush=True)
    print('assembly path: %d of %d parts blocked' % (bad, len(names)))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
