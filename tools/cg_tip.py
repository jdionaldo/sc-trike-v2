"""Mass, centre of gravity and static tip-over angle of the built model.

    python -I tools/cg_tip.py output/model.step [--json out.json] [--verbose]
        [--battery-g 120] [--battery-xyz X Y Z] [--roof-drop 25] [--roof-z 100]

Reads the STEP assembly (soma.import_assembly), uses exact OCC volume properties (no tessellation).
Masses: printed parts = solid volume x density x infill factor; bought parts = point masses from the
table HW below at the part's volume centroid (every value is ASSUMED, the project's BOM files give none).
Tip-over: lateral, about the ground line from a front tyre contact to the rear tyre contact.
Reusable on later builds: parts are matched by name; unmatched parts are listed and counted as steel
(volume x 7.85) so nothing is silently dropped. Units: mm, g.
"""
import argparse, json, math, os, re, sys, time
import numpy as np
import cadquery as cq
from soma import import_assembly

PETG, PLA = 1.27, 1.24      # g/cm3
STEEL = 7.85

# ---- bought parts: regex on the lower-case leaf name -> (grams or None = steel by volume, note)
# All ASSUMED (no mass in BOM_COST.*, kit/bom.md, NOTES.md). First match wins.
HW = [
    (r'^b_motor$',                 160.0, 'Hobbywing Fusion SE 2-in-1 motor+ESC, typical 150-170 g'),
    (r'^b_motor_shaft$',           0.0,   'included in motor'),
    (r'^b_motor_pinion$',          5.0,   '12 mm pinion'),
    (r'^b_spur_gear$',             10.0,  '33 mm spur gear'),
    (r'^b_rear_ring$',             30.0,  'rear ring gear 25.6 mm, steel'),
    (r'^b_gt2_belt',               4.0,   'GT2 300x6 mm belt'),
    (r'^steer_servo_shaft$',       0.0,   'included in servo'),
    (r'^steer_servo(_-?1)?$',      55.0,  'low-profile servo 40.8x20.2 mm (same value as tools/cg_fit.py); version 2 has one per side'),
    (r'^steer_horn_-?1$',          4.0,   'version 2: bought metal servo horn with ball stud, each'),
    (r'^mock_receiver$',           12.0,  'FS-iA6B class receiver'),
    (r'^mock_tire',                45.0,  '1.9 in crawler tyre + foam, each'),
    (r'^rear_tire$',               60.0,  '2.2 in crawler tyre + foam'),
    (r'^mock_rear_hub$',           10.0,  '12 mm hex hub'),
    (r'^rear_shock',               30.0,  '100 mm piggyback shock'),
    (r'^heave_shock_body$',        18.0,  'front heave shock body'),
    (r'^heave_shock_shaft$',       5.0,   'front heave shock shaft'),
    (r'^(wheel_hex_hub|b_rear_shaft|mock_rear_axle|m6_shock_pin|pivot_rod|rod_end|ball_stud|steer_stud|tie_rod|'
     r'heave_shock_pin|axle_pin|axle_nut|b_front_pivot_axle|tip_shaft|mock_axle|lever_ball_stud|uplink_pin|front_knuckle|kingpin|front_axle_pin|'
     r'knuckle_ball_stud|rocker_pin|rocker_ball_stud|drag_link|carrier_pin|beam_yaw_pin|shock_pin|heave_stop_dowel)', None, 'steel rod/pin/stud/nut, CAD volume x 7.85'),
]
BATTERY_NOTE = '3S 1300 mAh LiPo (BOM B8); not in the model (removed Nov 22), ASSUMED 120 g and position'

# ---- printed parts: group / name patterns. body = counted in the "body" set.
PRINTED_PLA = re.compile(r'^(print_shell|desert_shell|lights)(/|$)')
BODY = re.compile(r'^(print_shell|desert_shell|lights)(/|$)')    # body = Print_shell (stealth) or Desert_shell + Lights strips
PRINTED_NAME = re.compile(r'^(w3_|cage$|roof_plate$|front_light_contact_block$|lean_head$|lower_arm_|upper_arm_|'
                          r'knuckle_|steer_horn$|steer_servo_mount$|pulley_|wheel_f_|wheel_r$|'
                          r'arm_-?1$|uplink_-?1$|front_carrier_-?1$|balance_beam$|beam_carrier$|rocker_-?1$|rocker_bracket_-?1$|droop_stop_pad_-?1$|cup_block_-?1$|'
                          r'servo_floor_-?1$|servo_clip_-?1$|yaw_stop_pad_|heave_saddle_)')      # last line: version 2 swing arm front end


def leaves(node, path, loc):
    l = loc * node.loc if getattr(node, 'loc', None) is not None else loc
    p = path + [node.name]
    if node.children:
        for c in node.children:
            yield from leaves(c, p, l)
    else:
        yield p, node.obj, l


def to_shape(o):
    return o.val() if hasattr(o, 'val') else o


def props(shape):
    c = shape.Center()
    return shape.Volume(), np.array([c.x, c.y, c.z])


def classify(path, name):
    """-> (kind, density_or_grams, note). kind: printed / hw / steel"""
    grp = '/'.join(path[1:-1]).lower() if len(path) > 2 else ''
    low = name.lower()
    if PRINTED_PLA.match(grp) or PRINTED_NAME.match(low):
        dens = PLA if PRINTED_PLA.match(grp) else PETG
        return 'printed', dens, ('PLA 1.24' if dens == PLA else 'PETG 1.27')
    for rx, g, note in HW:
        if re.match(rx, low):
            return ('steel', STEEL, note) if g is None else ('hw', g, note)
    return 'steel', STEEL, 'UNMATCHED: steel density x volume'


def slice_factor(root):
    """Effective fill factor: sliced grams (all plates, incl. supports) / sum(solid volume x density) of the kit."""
    try:
        parts = json.load(open(os.path.join(root, 'kit/print/parts.json')))['parts']
        sl = json.load(open(os.path.join(root, 'kit/print/bambu/slice_check.json')))
        g = sum(v['grams'] for v in sl.values())
        s = sum(p['volume_cm3'] * (PLA if p['material'].startswith('PLA') else PETG) for p in parts)
        return g / s, g, s
    except Exception:
        return None, None, None


def tip(cg, contacts, gz):
    """Lateral tip angle (deg) about front-tyre-to-rear-tyre lines (min of the two), and pitch about the front axle."""
    h = cg[2] - gz
    fl, fr, rr = contacts['FL'], contacts['FR'], contacts['R']
    out = {'h': h}
    for side, f in (('left', fl), ('right', fr)):
        d = np.array(rr) - np.array(f)
        d /= np.linalg.norm(d)
        v = np.array([cg[0] - f[0], cg[1] - f[1]])
        # horizontal perpendicular distance of CG from the line, positive when inside the support triangle
        dist = d[0] * v[1] - d[1] * v[0]
        inward = np.array(rr) - np.array([cg[0], cg[1]])
        sign = 1.0
        # inside = same side as the opposite front contact
        o = np.array(fr if side == 'left' else fl) - np.array(f)
        if (d[0] * o[1] - d[1] * o[0]) * dist < 0:
            sign = -1.0
        out[side] = math.degrees(math.atan2(abs(dist) * sign, h))
    out['lateral'] = min(out['left'], out['right'])
    fy = fl[1]
    out['pitch_fwd'] = math.degrees(math.atan2(abs(cg[1] - fy), h)) * (1 if cg[1] > fy else -1)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('step')
    ap.add_argument('--json')
    ap.add_argument('--verbose', action='store_true')
    ap.add_argument('--battery-g', type=float, default=120.0)
    ap.add_argument('--battery-xyz', type=float, nargs=3)
    ap.add_argument('--battery-size', type=float, nargs=3, default=[76.0, 35.0, 25.0], help='LxWxH (x,y,z) for the default placement')
    ap.add_argument('--allowance-g', type=float, default=35.0, help='unmodelled hardware (bearings, inserts, screws, wire, magnets, LEDs)')
    ap.add_argument('--roof-drop', type=float, default=25.0)
    ap.add_argument('--roof-z', type=float, default=100.0)
    a = ap.parse_args()
    t0 = time.time()
    root = os.path.dirname(os.path.dirname(os.path.abspath(a.step)))
    asm = import_assembly(a.step)
    rows = []
    for path, obj, loc in leaves(asm, [], cq.Location()):
        sh = to_shape(obj).moved(loc)
        v, c = props(sh)
        bb = sh.BoundingBox()
        name = path[-1]
        kind, k, note = classify(path, name)
        grp = '/'.join(path[1:-1]) or '-'
        rows.append(dict(name=name, group=grp, kind=kind, vol=v / 1000.0, c=c, bb=(bb.zmin, bb.zmax), shape=sh,
                         k=k, note=note, body=bool(BODY.match(grp.lower() + '/'))))
    print('loaded %d leaves in %.0f s' % (len(rows), time.time() - t0))

    # ground and tyre contacts
    def bbx(r):
        b = r['shape'].BoundingBox(); return b
    tyres = [r for r in rows if re.match(r'^(mock_tire|rear_tire)', r['name'].lower())]
    gz = min(bbx(r).zmin for r in tyres)
    contacts = {}
    for r in tyres:
        b = bbx(r)
        key = 'R' if r['name'].lower().startswith('rear') else ('FL' if (b.xmin + b.xmax) < 0 else 'FR')
        contacts[key] = (round((b.xmin + b.xmax) / 2, 2), round((b.ymin + b.ymax) / 2, 2))
    # FL = left of the car looking forward; front faces -Y so +X is the car's left? name by sign only
    print('ground z %.2f  contacts %s' % (gz, contacts))

    fac, sg, sv = slice_factor(root)
    print('slice calibration: sliced %s g / solid-density %s g -> factor %s' % (
        None if sg is None else round(sg), None if sv is None else round(sv), None if fac is None else round(fac, 3)))

    # battery placement (default: crosswise on the deck just ahead of the motor, resting on the deck top)
    mot = [r for r in rows if r['name'].lower() == 'b_motor']
    if a.battery_xyz:
        bxyz = np.array(a.battery_xyz)
    else:
        L, W, H = a.battery_size
        if mot:
            mb = mot[0]['shape'].BoundingBox()
            by = mb.ymin - 6.0 - W / 2          # ahead of the motor (front = -Y)
        else:
            by = -60.0
        # deck top under that spot: highest zmax of w3_deck_* / motor plate parts whose bbox covers (0, by)
        top = None
        for r in rows:
            if r['name'].lower().startswith(('w3_deck', 'w3_motor_plate')):
                b = r['shape'].BoundingBox()
                if b.ymin - 1 <= by <= b.ymax + 1 and b.xmin - 1 <= 0 <= b.xmax + 1:
                    top = b.zmax if top is None else max(top, b.zmax)
        top = top if top is not None else gz + 25
        bxyz = np.array([0.0, by, top + H / 2])
    print('battery point mass %.0f g at (%.1f, %.1f, %.1f)  [%s]' % (a.battery_g, *bxyz, BATTERY_NOTE))

    sys.path.insert(0, os.path.join(root, 'kit'))
    import registry as _reg
    _byid = {p['id']: p for p in _reg.PARTS}
    for r in rows:
        r['vol_cm3'] = r['vol']          # cm3 (mm3/1000)
        _k = _reg.key_of(r['name'])
        r['solid_rule'] = bool(_k) and _reg.print_fill(_byid[_k])[1] == '100%'
    print('printed designs on the solid rule (NOTES 67):', sum(1 for r in rows if r['kind'] == 'printed' and r['solid_rule']), 'of', sum(1 for r in rows if r['kind'] == 'printed'),
          '; not in the registry:', sorted({r['name'] for r in rows if r['kind'] == 'printed' and not _reg.key_of(r['name'])})[:20])

    def mass_g(r, mode):
        if r['kind'] == 'hw':
            return r['k']
        g = r['vol_cm3'] * r['k']
        if r['kind'] == 'printed' and mode == 'sliced' and r.get('solid_rule'):
            return g          # (NOTES 67) the kit prints every load-carrying design solid (kit/registry.py print_fill)
        return g * (fac if (r['kind'] == 'printed' and mode == 'sliced' and fac) else (0.5 if (r['kind'] == 'printed' and mode == 'sliced') else 1.0))

    def combine(mode, with_body, with_batt):
        items = []
        for r in rows:
            if r['body'] and not with_body:
                continue
            items.append((mass_g(r, mode), r['c']))
        if with_batt:
            items.append((a.battery_g, bxyz))
        m0 = sum(m for m, _ in items); c0 = sum(m * c for m, c in items) / m0
        items.append((a.allowance_g, c0))     # neutral position: does not shift the CG
        return items

    def cg_of(items):
        m = sum(x for x, _ in items); c = sum(x * np.asarray(p) for x, p in items) / m
        return m, c

    # --- roof drop estimate: split every part with zmax above roof-z at that plane, move the upper portion down
    drop_frac = {}      # id(row) -> (fraction of volume above plane, centroid of the upper portion)
    for r in rows:
        if r['bb'][1] <= a.roof_z:
            continue
        sh = r['shape']
        bb = sh.BoundingBox()
        try:
            box = cq.Solid.makeBox(bb.xlen + 20, bb.ylen + 20, bb.zmax - a.roof_z + 10,
                                   cq.Vector(bb.xmin - 10, bb.ymin - 10, a.roof_z))
            up = sh.intersect(box)
            uv, uc = props(up)
            drop_frac[id(r)] = (uv / 1000.0 / r['vol_cm3'], uc)
        except Exception as e:
            print('  roof split failed for', r['name'], e)
            drop_frac[id(r)] = ((bb.zmax - a.roof_z) / bb.zlen if bb.zlen else 0, r['c'])
    print('parts crossing z=%.0f: %d (%.0f s so far)' % (a.roof_z, len(drop_frac), time.time() - t0))

    def items_lowered(mode, with_body, with_batt):
        items = []
        for r in rows:
            if r['body'] and not with_body:
                continue
            m = mass_g(r, mode)
            if id(r) in drop_frac:
                f, uc = drop_frac[id(r)]
                mu = m * f
                # lower portion keeps its centroid shifted by the removed moment; upper portion drops
                lc = (r['c'] * m - uc * mu) / (m - mu) if m - mu > 1e-9 else r['c']
                items.append((m - mu, lc))
                items.append((mu, uc - np.array([0, 0, a.roof_drop])))
            else:
                items.append((m, r['c']))
        if with_batt:
            items.append((a.battery_g, bxyz))
        m0 = sum(x for x, _ in items); c0 = sum(x * np.asarray(p) for x, p in items) / m0
        items.append((a.allowance_g, c0))
        return items

    res = {}
    def rep(label, items, lowered=False):
        m, c = cg_of(items)
        gzz = gz if not lowered else gz
        t = tip(c, {'FL': contacts['FL'], 'FR': contacts['FR'], 'R': contacts['R']}, gz)
        res[label] = dict(mass_g=round(m, 1), cg=[round(float(x), 1) for x in c], cg_height=round(float(c[2] - gz), 1),
                          tip_lateral_deg=round(t['lateral'], 1), tip_left_deg=round(t['left'], 1),
                          tip_right_deg=round(t['right'], 1), pitch_fwd_deg=round(t['pitch_fwd'], 1))
        return m, c

    for mode in ('solid', 'sliced'):
        for body in (True, False):
            for batt in (True, False):
                lbl = '%s | %s body | %s battery' % (mode, 'with' if body else 'no', 'with' if batt else 'no')
                rep(lbl, combine(mode, body, batt))
                rep(lbl + ' | roof -%g mm (ESTIMATE)' % a.roof_drop, items_lowered(mode, body, batt), True)
    # battery alone
    bh = bxyz[2] - gz
    res['battery'] = dict(mass_g=a.battery_g, cg=[round(float(x), 1) for x in bxyz], cg_height=round(float(bh), 1),
                          note=BATTERY_NOTE)

    print()
    hdr = '%-52s %8s  %-22s %7s %8s %8s' % ('case', 'mass g', 'CG x,y,z (mm)', 'h mm', 'tip lat', 'pitch')
    print(hdr); print('-' * len(hdr))
    for k, v in res.items():
        if k == 'battery':
            continue
        print('%-52s %8.0f  %6.1f %6.1f %6.1f  %7.1f %7.1f deg %6.1f' % (k, v['mass_g'], *v['cg'], v['cg_height'],
              v['tip_lateral_deg'], v['pitch_fwd_deg']))
    print('BATTERY line: %.0f g at CG height %.1f mm above ground (ASSUMED)' % (a.battery_g, bh))
    # battery contribution to the full car, sliced
    full = res['sliced | with body | with battery']; nob = res['sliced | with body | no battery']
    print('battery effect (sliced, with body): CG height %.1f -> %.1f mm, tip %.1f -> %.1f deg'
          % (nob['cg_height'], full['cg_height'], nob['tip_lateral_deg'], full['tip_lateral_deg']))

    # group summary (sliced)
    print('\nGroup summary (sliced masses, no battery/allowance)')
    grp = {}
    for r in rows:
        g = r['group'].split('/')[0] if r['group'] != '-' else '-'
        d = grp.setdefault(g, [0.0, np.zeros(3), 0.0]); m = mass_g(r, 'sliced'); d[0] += m; d[1] += m * r['c']; d[2] += mass_g(r, 'solid')
    for g, (m, s, ms) in grp.items():
        c = s / m if m else s
        print('  %-18s sliced %7.1f g  solid %7.1f g  CG %6.1f %6.1f %6.1f' % (g, m, ms, *c))
    pm = sum(mass_g(r, 'sliced') for r in rows if r['kind'] == 'printed'); ps = sum(mass_g(r, 'solid') for r in rows if r['kind'] == 'printed')
    hm = sum(mass_g(r, 'sliced') for r in rows if r['kind'] != 'printed')
    print('printed: solid %.0f g, sliced %.0f g; bought/steel (ASSUMED) %.0f g; allowance %.0f g; battery %.0f g'
          % (ps, pm, hm, a.allowance_g, a.battery_g))
    un = [r['name'] for r in rows if r['note'].startswith('UNMATCHED')]
    if un:
        print('UNMATCHED parts (steel x volume):', ', '.join(un))
    if a.verbose:
        print('\nPer part: name | group | kind | vol cm3 | solid g | sliced g | centroid | note')
        for r in sorted(rows, key=lambda r: -mass_g(r, 'sliced')):
            print('%-30s %-24s %-7s %8.2f %7.1f %7.1f  (%6.1f %6.1f %6.1f) %s' % (r['name'], r['group'], r['kind'], r['vol_cm3'],
                  mass_g(r, 'solid'), mass_g(r, 'sliced'), *r['c'], r['note']))
    if a.json:
        out = dict(ground_z=gz, contacts=contacts, slice_factor=fac, battery_xyz=[float(x) for x in bxyz], cases=res,
                   parts=[dict(name=r['name'], group=r['group'], kind=r['kind'], vol_cm3=round(r['vol_cm3'], 3),
                               solid_g=round(mass_g(r, 'solid'), 2), sliced_g=round(mass_g(r, 'sliced'), 2),
                               centroid=[round(float(x), 2) for x in r['c']], body=r['body'], note=r['note']) for r in rows])
        json.dump(out, open(a.json, 'w'), indent=1)
    print('done in %.0f s' % (time.time() - t0))


if __name__ == '__main__':
    main()
