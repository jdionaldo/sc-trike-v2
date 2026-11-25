"""Hidden-line pictures of the desert shell on the cage (no app needed): renders/desert_<view>.svg and .png.
Run: python -I -B tools/desert_view.py"""
import subprocess, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'tools'))
import cadquery as cq
import front2_pose as fp

SHOW = ('Cage', 'Nose_tub', 'Hood_deck', 'Side_pod_right', 'Side_pod_left', 'Hex_grille', 'Nose_light_bar', 'Roof_panel', 'Tail_light_housing',
        'mock_tire_1', 'mock_tire_-1', 'Rear_tire', 'w3_skid')
P = fp.load(ROOT / 'output' / 'model.step')
shapes = [v for k, v in P.items() if fp.leaf(k) in SHOW]
print('shown', len(shapes), 'of', len(SHOW))
comp = cq.Compound.makeCompound(shapes)
for name, d in (('iso', (-1.2, -2.0, 0.9)), ('side', (1.0, 0.0, 0.05)), ('front', (-0.25, -1.0, 0.25)), ('rear', (0.9, 1.6, 0.8))):
    out = ROOT / 'renders' / ('desert_%s.svg' % name)
    cq.exporters.export(comp, str(out), opt={'width': 1400, 'height': 900, 'marginLeft': 20, 'marginTop': 20, 'projectionDir': d,
                                            'showAxes': False, 'showHidden': False, 'strokeWidth': 0.35})
    subprocess.run(['qlmanage', '-t', '-s', '1400', '-o', str(ROOT / 'renders'), str(out)], capture_output=True)
    print('written', out)
