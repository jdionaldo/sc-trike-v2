"""Fit of the owner's desert shell parts (desert_shell_variant/, read only) against the built model.

Reads output/model.step, leaves out the parts the desert shell replaces, and reports for each desert part every
overlap above 0.01 mm3 and every distance under 1 mm to another part. Rest pose only (the pose run is tools/front2_pose.py).
Run: python -I -B tools/desert_fit.py [own]     own = take the parts from parts/desert_shell.py instead of the owner's folder
"""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'tools'))
import front2_pose as fp

REPLACED = ('Front_shell', 'Grille_insert', 'Nose_light_strip', 'Roof_plate')
OWN = len(sys.argv) > 1 and sys.argv[1] == 'own'


def desert_parts():
    if OWN:
        from parts import desert_shell as ds
        return ds.build_all()
    from desert_shell_variant.exo_cage import build_front_exocage
    from desert_shell_variant import armored_shell as a
    tub, grille, led = a.build_nose_tub_and_inserts()
    r, l = a.build_side_pods()
    return {'Front_Exo_Cage': build_front_exocage(), 'Armored_Nose_Tub': tub, 'Hex_Mesh_Grille': grille, 'Nose_Light_Bar': led,
            'Armored_Hood_Deck': a.build_armored_hood(), 'Armored_Roof_Panel': a.build_armored_roof(), 'Side_Pod_Right': r, 'Side_Pod_Left': l}


def main():
    model = {k: v for k, v in fp.load(ROOT / 'output' / 'model.step').items() if fp.leaf(k) not in REPLACED}
    new = desert_parts()
    print('model parts', len(model), 'desert parts', len(new))
    for n, s in new.items():
        b = s.BoundingBox()
        print('%-20s solids %d valid %s mm3 %9.1f x %6.1f..%6.1f y %7.1f..%7.1f z %6.1f..%6.1f' % (
            n, len(s.Solids()), s.isValid(), s.Volume(), b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax))
    names = list(new)
    for i, n in enumerate(names):
        if n.endswith('Left'):
            continue                      # mirror of the right pod
        s = new[n]; sb = fp.bb(s)
        others = list(model.items()) + [(m, new[m]) for m in names[i + 1:]]
        for k, o in others:
            if not fp.bb_hit(sb, fp.bb(o)):
                continue
            v, where = fp.overlap(s, o)
            if v > 0.01:
                print('OVERLAP %-18s <> %-34s %9.2f mm3  %s' % (n, fp.short(k), v, where))
            else:
                d = s.distance(o)
                if d < 1.0:
                    print('near    %-18s <> %-34s %6.3f mm' % (n, fp.short(k), d))
    print('done')


if __name__ == '__main__':
    main()
