"""Numbers of the version 2 swing arm front end (parts/swingarm2.py): travel, arm angles, spring ratio, lean residual, bump steer, clearances.
Run: python -I tools/front2_check.py   (pure math, no build needed)"""
import math, os, sys
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import parts.swingarm2 as sa

D = math.degrees
up, dn = sa.travel_for_lean(sa.LEAN_DEG, 1), sa.travel_for_lean(sa.LEAN_DEG, -1)
print('arm %.2f mm, rest slope %.2f deg' % (sa.ARM_L, D(sa._REST)))
print('lean %g deg: wheel travel %+.1f / %+.1f mm; with bump / droop %+.1f / %+.1f mm' % (sa.LEAN_DEG, up, dn, up + sa.BUMP, dn - sa.DROOP))
for name, t in (('rising side', up), ('rising side + bump', up + sa.BUMP), ('dropping side', dn), ('dropping side + droop', dn - sa.DROOP)):
    y, z = sa.lever_top(t)
    print('  %-22s arm swing %+6.1f deg, wheel back %4.1f mm, lever top y %7.1f z %5.1f' % (name, D(sa.arm_swing(t)), sa.wheel_shift(t), y, z))
y0, z0 = sa.lever_top(0.0)
print('lever top at rest y %.1f z %.1f = beam centre; shock rear eye y %.1f' % (y0, z0, y0 + sa.SHOCK_L))
print('two-wheel bump %g mm: shock stroke %.2f mm (ratio %.2f); droop %g mm: %.2f mm' % (sa.BUMP, sa.shock_stroke(sa.BUMP, sa.BUMP),
      sa.shock_stroke(sa.BUMP, sa.BUMP) / sa.BUMP, sa.DROOP, sa.shock_stroke(-sa.DROOP, -sa.DROOP)))
print('pure lean %g deg: shock length change %.2f mm, beam yaw %.1f deg' % (sa.LEAN_DEG, sa.shock_stroke(up, dn), sa.beam_yaw(up, dn)))
print('lean + bump: beam yaw %.1f deg; lean + droop: %.1f deg' % (sa.beam_yaw(up + sa.BUMP, dn + sa.BUMP), sa.beam_yaw(up - sa.DROOP, dn - sa.DROOP)))
zs = [sa.lever_top(t)[1] for t in (up + sa.BUMP, up, 0.0, dn, dn - sa.DROOP)]
print('lever top height over the travel: %.1f .. %.1f (lowered cage rail top %.1f, beam 5 mm below the ball centres)' % (min(zs), max(zs), sa.CAGE_RAIL_TOP))
print('lever top to the upper link lug (y %.1f) at full droop: %.1f mm' % (sa.UPLINK_IN[1] + 6.0, sa.lever_top(dn - sa.DROOP)[0] - (sa.UPLINK_IN[1] + 6.0)))
print('tie rod: horn ball (%.1f, %.1f, %.1f), horn %.1f mm, knuckle ball y %.1f z %.1f (%.1f behind, %.1f above the axle)' % (*sa.HORN_BALL, sa.HORN_L, *sa.KNUCKLE_BALL_YZ,
      sa.KNUCKLE_BALL_YZ[0] - sa.TIP[0], sa.KNUCKLE_BALL_YZ[1] - sa.TIP[1]))
print('bump steer over the travel:', ', '.join('%+.0f mm: %.2f deg' % (t, sa.bump_steer(t)) for t in (dn - sa.DROOP, dn, 0.0, up, up + sa.BUMP)))
for name, x, z in (('deck edge', 54.5, 12.6), ('body sill now (x 93.5, z 12.6)', 93.5, 12.6), ('tucked sill (x 93.5, z 34)', 93.5, 34.0), ('tucked sill foot (x 62, z 14)', 62.0, 14.0)):
    print('  ground clearance %-32s upright %.1f, at %g deg lean %.1f mm' % (name, z - sa.GROUND_Z, sa.LEAN_DEG, sa.clearance_at_lean(x, z)))
print('--- 3D (kingpin x %.0f is an estimate): steering, both wheels, rod length %.1f mm' % (sa.KINGPIN_X, sa.ROD_L))
for eta in (-30.0, 0.0, 30.0):
    for name, phi, ex in (('straight', 0.0, 0.0), ('lean', sa.LEAN_DEG, 0.0), ('lean + bump', sa.LEAN_DEG, sa.BUMP), ('lean + droop', sa.LEAN_DEG, -sa.DROOP)):
        row = []
        for s in (1, -1):
            t = sa.side_travel(phi, ex, s)
            d = sa.steer_angle(t, eta, s); d0 = sa.steer_angle(0.0, eta, s)
            ta = sa.transmission_angles(t, eta, s)
            row.append('side %+d steer %+6.2f (bump steer %+5.2f) transm. %3.0f / %3.0f' % (s, d, d - d0, ta[0], ta[1]))
        print('  horn %+3.0f %-13s %s' % (eta, name, ' | '.join(row)))
for name, phi, ex in (('bump', 0.0, sa.BUMP), ('droop', 0.0, -sa.DROOP), ('lean', sa.LEAN_DEG, 0.0), ('lean + bump', sa.LEAN_DEG, sa.BUMP), ('lean + droop', sa.LEAN_DEG, -sa.DROOP)):
    bp, bm, st, eye, rear, comp = sa.beam_pose(sa.side_travel(phi, ex, 1), sa.side_travel(phi, ex, -1))
    print('  beam %-13s ball distance %+5.2f mm, shock compressed %+5.2f mm (3D), eye at x %+.2f y %.1f z %.1f' % (name, st, comp, *eye))
