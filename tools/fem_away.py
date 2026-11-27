"""Read an FEM result away from its supports and loads (V2 copy of the method, written here; nothing imported from V1).

A rigid support or a load spread over a face makes a stress peak at its edge that grows with every finer mesh. This reads the highest
von Mises stress over each part's result surface only at points farther than D mm from every held or loaded face, so a peak that sits on a
support is judged a little away from it (the FEM skill's rule). Read it at two meshes: settled when the two maxima differ by 10 % or less.

  python -I -B tools/fem_away.py .soma/results/<study>.glb D [strength_MPa] [part substring]

The result GLB of `soma analyze fem`: part meshes carry _STRESS (MPa) per vertex; meshes named bc<i> are the held and loaded faces.
Prints per part: the maximum away from the bc faces, where, its safety (strength / stress), and the global maximum with its distance to a bc face.
"""
import json, struct, sys
import numpy as np
from scipy.spatial import cKDTree


def read_glb(path):
    raw = open(path, 'rb').read()
    jlen = struct.unpack('<I', raw[12:16])[0]
    doc = json.loads(raw[20:20 + jlen])
    data0 = 20 + jlen + 8

    def accessor(i):
        a = doc['accessors'][i]
        view = doc['bufferViews'][a['bufferView']]
        start = data0 + view.get('byteOffset', 0) + a.get('byteOffset', 0)
        width = {'SCALAR': 1, 'VEC3': 3}[a['type']]
        arr = np.frombuffer(raw, dtype=np.float32, count=a['count'] * width, offset=start)
        return arr.reshape(-1, 3) if width == 3 else arr

    meshes, bc = [], []
    for m in doc['meshes']:
        name = m.get('name', '')
        for prim in m['primitives']:
            att = prim['attributes']
            if name.startswith('bc'):
                bc.append(accessor(att['POSITION']))
            elif '_STRESS' in att:
                meshes.append((name, accessor(att['POSITION']), accessor(att['_STRESS'])))
    return meshes, bc


def main():
    path, away = sys.argv[1], float(sys.argv[2])
    strength = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
    only = sys.argv[4] if len(sys.argv) > 4 else None
    meshes, bc = read_glb(path)
    if not bc:
        sys.exit('no bc meshes in %s' % path)
    tree = cKDTree(np.vstack(bc))
    for name, pos, stress in meshes:
        if only and only not in name:
            continue
        dist, _ = tree.query(pos)
        keep = dist > away
        if not keep.any():
            print('%s: no point farther than %.1f mm from a support or load' % (name, away))
            continue
        k = int(np.argmax(np.where(keep, stress, -1.0)))
        g = int(np.argmax(stress))
        print('%s: away (> %.1f mm, %d of %d points) %.2f MPa at (%.1f, %.1f, %.1f), safety %.2f | global %.2f MPa at (%.1f, %.1f, %.1f), %.1f mm from a bc face'
              % (name, away, keep.sum(), len(pos), stress[k], *pos[k], strength / stress[k], stress[g], *pos[g], dist[g]))


if __name__ == '__main__':
    main()
