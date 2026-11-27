"""Printable shell only, reconstructed from the supplied trike references."""
from soma import add_part as _register_part, param, import_step, import_assembly
from parts.util import clean_registered

def add_part(name, geometry, **options):
    return _register_part(name, clean_registered(geometry), **options)
from parts.params import *
from parts.shell_geometry import strip_dust, make_shell, make_canopy, make_grille, canopy_seat_tool, grille_seat_tool, checked, split_on_creases

shell=make_shell()
canopy=make_canopy()
grille=make_grille()
# Master Surface Workflow: verify the outer body forms a single unified, watertight faceted master solid before splitting into panels.
_master_body=shell.fuse(canopy).clean()
assert _master_body.isValid() and len(_master_body.Solids())==1, 'Master outer body must be a single unified watertight solid'
shell=strip_dust(shell.cut(canopy_seat_tool(canopy)).cut(grille_seat_tool(grille)).clean())
# The glue seam follows the existing angular panel folds, with a 0.3 mm gap.
front=shell
from parts.tail import make_tail
from parts.util import fuse
rear=fuse(make_tail())
# Intentional glue clearances, with no interpenetration between printed parts.
for name,a,b in [('body seam',front,rear),('front canopy seat',front,canopy),
                 ('rear canopy seat',rear,canopy),('grille seat',front,grille)]:
    assert a.intersect(b).Volume()<0.001, name+' interferes'
    assert a.distance(b)>=PANEL_GAP_M*0.8, name+' has insufficient assembly clearance'
import math
import cadquery as cq
from parts.relief import relief_tools, swingarm_relief_tools, rear_lift_angle, rot_yz, PIVOT, REAR_AXLE, FRONT_Y, SHOCK_TOP0, SHOCK_BOT0
_tools = relief_tools()
# Rake: the shell pitches tail-up about the front axle, so the front fenders stay on the tires.
_phi = math.radians(SHELL_PITCH); _c, _s = math.cos(_phi), math.sin(_phi)
_cy, _cz = FRONT_Y + CHASSIS_Y, 55.86 + CHASSIS_Z
_SX, _SZ = SHELL_SCALE, SHELL_SCALE*SHELL_Z
_WX = SHELL_SCALE*SHELL_HALF_WIDTH/BASE_HALF_WIDTH
_pitch = cq.Matrix([[_WX, 0, 0, 0],
                    [0, _c*_SX, -_s*_SZ, _cy - _c*_cy + _s*_cz],
                    [0, _s*_SX,  _c*_SZ, _cz - _s*_cy - _c*_cz]])
# Round flank scoop behind each front tire (14:09 markup): one circular arch, centre ~48 mm behind the tire axle,
# replaces the ragged notches the suspension travel cuts left on the lower flank. (y, z) centre and radius in mm, final frame.
SCOOP_YZ = (-170.0, 33.4); SCOOP_R = 91.0; SCOOP_X0 = 56.5   # X0 and R cover every suspension/servo clearance cut through the flank (they start at x 56.9 and reach 94 mm from the old centre), so the arch edge is one clean arc
# The hood cheek above the tire ties the nose to the body sides, so the scoop stops just above the tire top.
SCOOP_TOP = 55.86 + CHASSIS_Z + 55.86 + TIRE_GAP + 4.0
_scoop = cq.Solid.makeCylinder(SCOOP_R, 140, cq.Vector(SCOOP_X0, SCOOP_YZ[0], SCOOP_YZ[1]), cq.Vector(1, 0, 0)).intersect(
    cq.Workplane('XY').box(140, 400, SCOOP_TOP + 100, centered=(False, True, False)).val().translate(cq.Vector(SCOOP_X0, SCOOP_YZ[0], -100)))
_fins = [_scoop, _scoop.mirror('YZ')]
# Trim the shell bottom flat along the chassis deck plane (deck top rises from 32.6 mm at y -221 to 59.8 mm at y 160, ground reference).
_zp = lambda y: DECK_REF_Z0 + DECK_SLOPE*(y - CHASSIS_Y + 189.0) + CHASSIS_Z + DECK_TRIM   # the deck top plane (DECK_TRIM 0), -6.06 = its underside
_deck_trim = cq.Workplane('YZ', origin=(-400, 0, 0)).polyline([(-420, _zp(-420)), (420, _zp(420)), (420, _zp(420) - 400), (-420, _zp(-420) - 400)]).close().extrude(800).val()
_nose_cut = [cq.Solid.makeBox(300, 300, 400, cq.Vector(sx*NOSE_HALF_WIDTH + (0 if sx > 0 else -300), NOSE_END_Y - 300, -100)) for sx in (1, -1)]
def scaled(s, name):
    s = s.transformGeometry(_pitch)
    for t in _tools + _fins + [_deck_trim] + _nose_cut:
        s = s.cut(t)
    s = strip_dust(s.clean(), min_volume=1500.0)   # slivers of rocker edge left by the relief cuts
    return checked(s, name)
_shells = {n: scaled(g, n) for n, g in (('Front shell', front), ('Rear shell', rear), ('Canopy', canopy), ('Grille insert', grille))}
# Closing panel beside the nose (markups 00:10 / 00:11 / 00:33): the fender front ends open at y = NOSE_END_Y. A two-facet arrow
# panel follows the hood lines: top edge along the hood front, a fold running from the outer hood corner to a forward tip, then
# down to the nose bottom beside the grille. It stays inside the tire and suspension envelope (check with tools/collide.py).
FRONT_CAP_T = WALL_PRINT   # same printed wall as the rest of the shell (was 5 mm, the thickest panel on the part)
_nose_cap_edges = {}
def _front_cap(sx):
    # Intersect the real hood with its trimming plane: no guessed floating
    # vertices, and the bottom nose corner shares the original nose rake.
    from parts.shell_geometry import point
    def mapped(p):
        x,y,z=point(p)
        return cq.Vector(_WX*x,_c*_SX*y-_s*_SZ*z+_cy-_c*_cy+_s*_cz,
                         _s*_SX*y+_c*_SZ*z+_cz-_s*_cy-_c*_cz)
    nose_top=mapped((sx*24,-139,57))
    nose_bottom=mapped((sx*NOSE_BASE_HALF,-139,25))
    # The original hood is a pair of planar facets. Probe each end of its
    # closing edge before adding either cap.
    shape=_shells['Front shell']
    def hood_z(x):
        line=cq.Edge.makeLine((x,NOSE_END_Y+0.05,80),(x,NOSE_END_Y+0.05,170))
        pts=shape.intersect(line).Vertices()
        assert pts, 'Front cap cannot find the hood edge'
        return max(v.Z for v in pts)
    xo=sx*70.0; xi=sx*(NOSE_HALF_WIDTH-0.4)
    outer=cq.Vector(xo,NOSE_END_Y+0.15,hood_z(xo))
    inner=cq.Vector(xi,NOSE_END_Y+0.15,hood_z(xi))
    # Broad second facet follows the lower cheek edge and lies exactly in the plane of (outer, nose_top, nose_bottom).
    _np=(outer-nose_top).cross(nose_bottom-nose_top)
    _ly=NOSE_END_Y+0.15; _lz=97.3
    _lx=nose_top.x-(_np.y*(_ly-nose_top.y)+_np.z*(_lz-nose_top.z))/_np.x
    lower=cq.Vector(_lx,_ly,_lz)
    _nose_cap_edges[sx]=(nose_top,outer)
    facets=[(outer,inner,nose_top),(outer,nose_top,lower),(lower,nose_top,nose_bottom)]
    from parts.panels import panel_prisms
    cap=_hull_union(panel_prisms([[p.toTuple() for p in tri] for tri in facets],(-sx,1,0),FRONT_CAP_T,final_frame=True))
    # Clip the mitred return to the real hood plane: its inward offset must
    # not project upward as a raised strip along the closing-panel edge.
    normal=(inner-outer).cross(nose_top-outer).normalized()
    if normal.z<0: normal=-normal
    plane=cq.Plane(origin=outer,xDir=(inner-outer).normalized(),normal=normal)
    below=cq.Workplane(plane).rect(1000,1000).extrude(-500).val()
    return cap.intersect(below).clean()
def _hull_union(solids):
    from parts.util import fuse
    return fuse(solids)
def _hull_pts(pts):
    from parts.util import hull
    return hull([p.toTuple() for p in pts])
_fs = _shells['Front shell']
for _sx in (1, -1):
    _fs = _fs.fuse(_front_cap(_sx))
_fs = _fs.clean()
_shells['Front shell'] = checked(_fs, 'Front shell')

# The canopy ends at the B-pillar. Its former rear roof-window cut is obsolete.
# Light strips: separate printed red parts that sit in channels cut into the shell, pegs (5 mm, 3 mm LED bore) pass through the wall.
from parts.light_strips import make_tail_strip, make_nose_strip, nose_backing, STRIP_FIT, NOSE_SOCKET_CUTTERS
_tail_strip, _tail_chan = make_tail_strip()
_nose_strip, _nose_chan = make_nose_strip(_shells['Front shell'])
_shells['Rear shell'] = checked(_shells['Rear shell'].cut(_tail_chan).clean(), 'Rear shell')
# Tail housing screws (2 x M3 countersunk, behind the red lens): boss on the inside of the wall, hole + countersink through the seat floor.
from parts.light_strips import make_tail_screws, TAIL_SCREW_INFO
_tail_scr = make_tail_screws()
_rs = _shells['Rear shell']
for _pad in _tail_scr['pads']: _rs = _rs.fuse(_pad)
_rs = _rs.clean()
for _cut in _tail_scr['cuts']: _rs = _rs.cut(_cut)
_shells['Rear shell'] = checked(_rs.clean(), 'Rear shell')
assert len(_shells['Rear shell'].Solids())==1, 'tail screw bosses must stay one solid with the housing'
print('tail screws: M3 countersunk, axial length to insert bottom', {k: v['screw_axial_to_insert_bottom'] for k, v in TAIL_SCREW_INFO.items()}, 'tilt deg', TAIL_SCREW_INFO[1]['tilt_deg'])
# The opaque backing must stay inside the hood skin: cut away everything above the hood plane of each half.
def _hood_top(x,y):
    pts=_shells['Front shell'].intersect(cq.Edge.makeLine((x,y,60),(x,y,200))).Vertices()
    return max(v.Z for v in pts)
_backing=nose_backing()
for _sx in (1,-1):
    _p=[cq.Vector(_sx*x,y,_hood_top(_sx*x,y)) for x,y in ((28,-302),(28,-286),(4,-294))]
    _n=(_p[1]-_p[0]).cross(_p[2]-_p[0]).normalized()
    if _n.z<0:_n=-_n
    _above=cq.Workplane(cq.Plane(origin=_p[0]-_n*0.3,normal=_n.toTuple())).rect(400,400).extrude(100).val()
    _half=cq.Solid.makeBox(60,60,200,cq.Vector(0 if _sx>0 else -60,-330,40))
    _backing=_backing.cut(_above.intersect(_half))
_front_cut=_shells['Front shell'].fuse(_backing).cut(_nose_chan,tol=1e-4)
for _socket_cut in NOSE_SOCKET_CUTTERS:
    _front_cut=_front_cut.cut(_socket_cut,tol=1e-4)
_big=[x for x in _front_cut.Solids() if x.Volume()>1.0]   # zero-volume slivers left where the lens channel meets the nose edge
_front_cut=_big[0] if len(_big)==1 else cq.Compound.makeCompound(_big)
_shells['Front shell']=checked(strip_dust(_front_cut.clean(),min_volume=0.001).fix(),'Front shell')
from parts.util import weld
_shells['Front shell']=weld(_shells['Front shell'],'Hood and front shell')
# Wing ends of the nose strip: close the open end faces with a short backing block, so no red end shows from inside the cabin.
from parts.util import hull as _hull
_end_blocks=[]
for _f in _nose_strip.Faces():
    if _f.geomType()!='PLANE': continue
    _c=_f.Center(); _nn=_f.normalAt(_c)
    if abs(_c.x)>50 and _c.y>-256 and _nn.y>0.7 and _nn.dot(cq.Vector(0.49*(1 if _c.x>0 else -1),0.87,0))>0.95:
        _vs=[v.Center() for v in _f.Vertices()]
        _pts=[(q+_nn*0.15).toTuple() for q in _vs]+[(q+_nn*(3.0+0.15)).toTuple() for q in _vs]
        _end_blocks.append(_hull(_pts))
print('nose strip end faces closed:',len(_end_blocks))
import os as _os
if _end_blocks and _os.environ.get('FUGU_LEGACY_NOSE_PATCH'):
    _fs2=_shells['Front shell'].fuse(*_end_blocks).clean()
    assert len(_fs2.Solids())==1 and _fs2.isValid(),'wing end backing broke the shell'
    _shells['Front shell']=_fs2
# Cover every flat strip face that looks into the cabin with a 2 mm backing, so no red strip shows from inside (0.15 mm clearance).
_cover_n = 0
for _f in (_nose_strip.Faces() if _os.environ.get('FUGU_LEGACY_NOSE_PATCH') else []):
    if _f.geomType() != 'PLANE' or _f.Area() < 30.0: continue
    _c = _f.Center(); _nn = _f.normalAt(_c)
    if _nn.y < -0.3 or _nn.z > 0.3: continue                    # forward-facing lens stays visible; upward faces sit under the hood skin
    _vs = [v.Center() for v in _f.Vertices()]
    _blk = _hull([(q + _nn*0.15).toTuple() for q in _vs] + [(q + _nn*2.15).toTuple() for q in _vs])
    for _cut in list(NOSE_SOCKET_CUTTERS) + [_nose_strip]:
        _blk = _blk.cut(_cut)
    try:
        _fs3 = _shells['Front shell'].fuse(_blk).clean()
        if len(_fs3.Solids()) == 1 and _fs3.isValid() and _fs3.intersect(_nose_strip).Volume() < 0.001:
            _shells['Front shell'] = _fs3; _cover_n += 1
    except Exception:
        pass
print('nose strip inner faces covered:', _cover_n)
assert _shells['Front shell'].intersect(_nose_strip).Volume()<0.001, 'Nose lens interferes with shell'
# One flat hood panel per side: lift the fender facet onto the hood plane with a thin wedge, so hood and fender top become a single face.
def _top_face(shape, x, y):
    c=cq.Vertex.makeVertex(x,y,max(v.Z for v in shape.intersect(cq.Edge.makeLine((x,y,60),(x,y,200))).Vertices()))
    return min((f for f in shape.Faces() if f.normalAt(cq.Vector(x,y,c.Z)).z>0.5 if True),key=lambda f:f.distance(c))
_v1=_shells['Front shell'].Volume()
for _sx in (1,-1):
    _hood=_top_face(_shells['Front shell'],_sx*25.0,-271.0)
    _fender=_top_face(_shells['Front shell'],_sx*48.7,-269.0)
    _hv=[cq.Vector(*v.toTuple()) for v in _hood.Vertices()]
    _n=(_hv[1]-_hv[0]).cross(_hv[2]-_hv[0]).normalized()
    if _n.z<0:_n=-_n
    _proj=lambda p:p+cq.Vector(0,0,-((p-_hv[0]).dot(_n))/_n.z)
    _fv=[q for q in (cq.Vector(*v.toTuple()) for v in _fender.Vertices()) if abs(q.x)>39.0 and q.y>-297.0]
    # Every near-coplanar top facet beside this hood plane (fender, nose-end hood piece, cap top) joins the plate.
    _near=[]
    for _f in _shells['Front shell'].Faces():
        _c=_f.Center()
        if _c.x*_sx<=0 or abs(_c.x)>80 or not (-306.0<_c.y<-245.0): continue
        if _f.normalAt(_c).dot(_n)<0.985: continue
        _fvv=[cq.Vector(*v.toTuple()) for v in _f.Vertices()]
        if all(-1.2<=(q-_hv[0]).dot(_n)<=0.15 for q in _fvv): _near.extend(q for q in _fvv if q.y>-306.0 and (abs(q.x)>34.0 or q.y>-297.0 or abs(q.x)<1.0))
    _top=[_proj(q) for q in _fv]+[_proj(q) for q in _hv]+[_proj(q) for q in _near]
    # Plate on the hood plane, 1.5 mm deep inside the wall: top face is one plane covering hood and fender facet.
    _pts=_top+[p-_n*1.5 for p in _top]
    from parts.util import hull as _h
    _wedge=_h([p.toTuple() for p in _pts])
    # Keep the plate behind the raked nose face (no wall sticking out of the nose), and shave anything that crosses above the hood plane.
    from parts.light_strips import _plane as _face_plane, NOSE_TOP as _NT, NOSE_BOT as _NB
    _fp,_fn=_face_plane(_NT,_NB,(0,-1,-0.3))
    _behind=cq.Workplane(_fp).rect(600,600).extrude(-300).val()
    _wedge=_wedge.intersect(_behind)
    _above=_h([p.toTuple() for p in _top+[p+_n*3.0 for p in _top]]).intersect(_behind)
    _pre=_shells['Front shell'].fuse(_wedge)
    _shells['Front shell']=_pre.cut(_above).clean()
    print('hood plane cut side',_sx,'removed mm3',round(_pre.Volume()-_shells['Front shell'].Volume(),1),'cutter vol',round(_above.Volume(),1))
    # Convex crease hood-front/cowl: the surface is the lower of the two planes. The front plane stands up to ~1.1 mm proud
    # of the cowl plane just ahead of the cowl edge (step wall). Shave everything above the cowl plane in that strip.
    _cowl=_top_face(_shells['Front shell'],_sx*25.0,-240.0)
    _cv=[cq.Vector(*v.toTuple()) for v in _cowl.Vertices()]
    _cn=(_cv[1]-_cv[0]).cross(_cv[2]-_cv[0]).normalized()
    if _cn.z<0:_cn=-_cn
    _cpl=cq.Plane(origin=(_cv[0]+_cn*0.0).toTuple(),normal=_cn.toTuple())
    _slab=cq.Workplane(_cpl).rect(1000,1000).extrude(4.0).val()
    _strip=cq.Solid.makeBox(75,16,80,cq.Vector(0.0 if _sx>0 else -75.0,-262.0,90.0))
    _cutc=_slab.intersect(_strip)
    # Keep the skin (1.5 mm) over the nose light strip: shaving it away would expose the strip from inside.
    _keep=cq.Compound.makeCompound([_nose_strip.translate(cq.Vector(0,0,_dz)) for _dz in (0.0,0.5,1.0,1.5)])
    _cutc=_cutc.cut(_keep)
    _pre=_shells['Front shell']; _shells['Front shell']=_pre.cut(_cutc).clean()
    print('cowl step shaved side',_sx,'mm3',round(_pre.Volume()-_shells['Front shell'].Volume(),1))
_shells['Front shell']=checked(strip_dust(_shells['Front shell'],min_volume=0.001).fix(),'Front shell')
def _unify_flat(shape,name,volume_tol=0.005):
    """Merge faces that lie in one plane (also B-spline faces that are numerically planar) across the whole part.
    Loosest tolerance first; accepted only if valid, one solid and the volume moves by under 0.5 %."""
    from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
    for lt,at in ((0.1,0.015),(0.05,0.01),(0.02,0.005),(0.01,0.003),(0.005,0.002),(1e-3,1e-3)):
        u=ShapeUpgrade_UnifySameDomain(shape.wrapped,True,True,True); u.SetLinearTolerance(lt); u.SetAngularTolerance(at); u.Build()
        g=cq.Shape.cast(u.Shape()).fix()
        ok=g.isValid() and len(g.Solids())==1 and abs(g.Volume()-shape.Volume())<volume_tol*shape.Volume()
        print(name,'flat-face merge tol',lt,len(shape.Faces()),'->',len(g.Faces()),'valid',g.isValid(),'solids',len(g.Solids()),'dvol',round(g.Volume()-shape.Volume(),1),'accepted' if ok else 'rejected')
        if ok: return g
    return shape
_shells['Front shell']=_unify_flat(_shells['Front shell'],'Front shell',volume_tol=0.001)
print('hood/fender wedge added, mm3, face verts:',len(_fv),'; ',round(_shells['Front shell'].Volume()-_v1,1))
# Cowl-edge groove (markup 15:45): a narrow V slot (wall + ramp facet, about 0.7 mm deep, 30 mm long) runs along the outer cowl edge
# from the A-pillar foot to y = -242. Fill it up to the cowl plane (2 mm slab under the plane, over the slot footprint), stopping
# short of the canopy pocket at the foot (y < -216).
def _fill_cowl_groove(shell):
    import numpy as _np
    from scipy.spatial import ConvexHull as _CH
    _done={}; _failed=[]
    for _sx in (1,-1):
        _cowl=max((f for f in shell.Faces() if f.normalAt(f.Center()).z>0.97 and f.Area()>1000 and f.Center().x*_sx>0),key=lambda f:f.Area())
        _c0=_cowl.Center(); _cn=_cowl.normalAt(_c0)
        _x0,_x1=sorted((_sx*74.0,_sx*82.0))
        _probe=cq.Solid.makeBox(_x1-_x0,20,3.5,cq.Vector(_x0,-236,128.5))
        _sel=[f for f in shell.Faces() if _probe.isInside(f.Center()) and 20<f.Area()<200]
        if len(_sel)!=2: print('cowl groove side',_sx,'not found (%d faces), skipped'%len(_sel)); continue
        _P=_np.array([[v.X,v.Y] for f in _sel for v in f.Vertices()]); _P=_P[_P[:,1]>-243.5]
        _poly=[tuple(map(float,_P[i])) for i in _CH(_P).vertices]
        _prism=cq.Workplane('XY').workplane(offset=100).polyline(_poly).close().extrude(80).val()
        # The slab depth and a fuzzy fuse are retried: an exact fuse fails when the slab's underside lands on an inner facet.
        _new=None
        for _depth,_tol in ((2.0,None),(2.0,1e-3),(2.4,1e-3),(1.6,1e-3)):
            _slab=cq.Workplane(cq.Plane(origin=_c0.toTuple(),normal=_cn.toTuple())).rect(600,600).extrude(-_depth).val()
            _fill=_prism.intersect(_slab).intersect(cq.Solid.makeBox(200,40,200,cq.Vector(-100,-256,0)))
            _try=(shell.fuse(_fill) if _tol is None else shell.fuse(_fill,tol=_tol)).clean()
            if _try.isValid() and len(_try.Solids())==1 and _try.Volume()>shell.Volume():
                _new=_try; break
        if _new is None:
            _failed.append(_sx); continue
        print('cowl groove filled side',_sx,'mm3',round(_new.Volume()-shell.Volume(),1))
        shell=_new; _done[_sx]=_fill
    # The shell is symmetric: a side whose own fill would not fuse takes the mirrored fill of the other side.
    for _sx in _failed:
        _new=None
        if -_sx in _done:
            for _tol in (None,1e-3,5e-3):
                _m=_done[-_sx].mirror('YZ')
                _try=(shell.fuse(_m) if _tol is None else shell.fuse(_m,tol=_tol)).clean()
                if _try.isValid() and len(_try.Solids())==1 and _try.Volume()>shell.Volume():
                    _new=_try; break
        if _new is None:
            print('cowl groove side',_sx,'fill would not join the shell (groove geometry changed), skipped'); continue
        print('cowl groove filled side',_sx,'with the mirrored fill, mm3',round(_new.Volume()-shell.Volume(),1))
        shell=_new
    return shell
assert _shells['Front shell'].Volume()-_v1<1400,'hood/fender wedge too large'
_shells['Front shell']=_fill_cowl_groove(_shells['Front shell'])
# The Front shell is symmetric by design, but the patches above act on each side on their own and do not always land the same
# way (the cowl groove fill joined on one side only). Rebuild it from one half and its mirror image; keep the half with the
# larger volume (the filled one). Accepted only if it is one valid solid within 0.5 % of the volume before.
def _symmetric(shell,name,pick=lambda h:h.Volume()):
    _halves=[]
    for _sx in (1,-1):
        _box=cq.Solid.makeBox(600,1200,800,cq.Vector(0 if _sx>0 else -600,-600,-200))
        _halves.append(shell.intersect(_box))
    _h=max(_halves,key=pick)
    for _tol in (None,1e-3):
        _m=_h.mirror('YZ')
        _try=(_h.fuse(_m) if _tol is None else _h.fuse(_m,tol=_tol)).clean()
        _ok=_try.isValid() and len(_try.Solids())==1 and abs(_try.Volume()-shell.Volume())<0.005*shell.Volume()
        print(name,'symmetric rebuild: halves mm3',[round(h.Volume(),1) for h in _halves],'faces',len(shell.Faces()),'->',len(_try.Faces()),'dvol',round(_try.Volume()-shell.Volume(),1),'accepted' if _ok else 'rejected')
        if _ok: return checked(_try,name)
    return shell
_shells['Front shell']=_symmetric(_shells['Front shell'],'Front shell')
assert _shells['Rear shell'].intersect(_tail_strip).Volume()<0.001, 'Tail lens interferes with housing'
assert _shells['Rear shell'].distance(_tail_strip)>=STRIP_FIT*0.8, 'Tail lens clearance is too small'
# ---- Stealth shell (parts/stealth_shell.py) replaces the traced Front shell, Canopy and Grille insert built above. It is built
# in printed coordinates from a few convex faceted blocks with an exact wall, then gets the same wheel-arch, deck and
# suspension-travel cuts. The traced shell above is still built because the tail housing and the light strips come from it.
STEALTH = True
# Shell style. 'Desert armored' is the owner's exo cage shell (parts/desert_shell.py, parts/exo_cage_front.py): nose tub, hood, side pods with gill
# louvers and a roof panel take the place of the front shell, grille insert, nose light strip and roof plate; the rear end and its lights stay.
# 'Stealth' is the shell before it. The stealth front shell is still built in both styles: the cage takes its belt line and cabin rim from it.
SHELL_STYLE = param('Desert armored', 'Shell style', options=['Desert armored', 'Stealth'], group='Shell')
DESERT = SHELL_STYLE == 'Desert armored'
# Open cockpit (Yeti style): the Canopy is not built into the model, the stealth Front shell is the lower body up to the belt line and the cage's
# cabin tubes (parts/cage.py) are the visible cabin. False restores the closed stealth canopy.
OPEN_COCKPIT = param(True, 'Open cockpit: no canopy, the cage forms the visible cabin', group='Cage')
_traced = dict(_shells)   # the cage keeps following the traced shell's belt and lower edge: the stealth skin was fitted around that cage
if STEALTH:
    import parts.stealth_shell as _ss
    _sf, _sc, _, _ = _ss.make()
    for _t in _ss.arch_cut() + [_deck_trim] + _tools:
        _sf = _sf.cut(_t)
    for _t in _tools:
        _sc = _sc.cut(_t)
    # 0.5 mm break on the outside edge of the wheel arches (stepped, the arch outline is curved). The bottom trim line and, with the open cockpit, the
    # cabin opening (the visible belt rim) get theirs as an exact facet further down, where those rims are squared off (square_cuts).
    _sf = _ss.chamfer_rims(_sf, [lambda g: _ss.arch_cut(grow=g)[:2]])
    _shells['Front shell'] = checked(strip_dust(_sf.clean(), min_volume=50.0), 'Front shell')
    if OPEN_COCKPIT: _shells.pop('Canopy')     # the traced canopy (in _traced) stays: the cage tail tubes follow its rear quarter
    else: _shells['Canopy'] = checked(strip_dust(_sc.clean(), min_volume=50.0), 'Canopy')
    _shells['Grille insert'] = checked(_ss.grille_insert(), 'Grille insert')
    assert OPEN_COCKPIT or _shells['Front shell'].intersect(_shells['Canopy']).Volume() < 0.01, 'Canopy sits inside the body skin'
    assert _shells['Front shell'].intersect(_shells['Grille insert']).Volume() < 0.01, 'Grille insert interferes with the nose'
    _stealth_strip = checked(_ss.nose_strip(), 'Nose light strip')
    assert _shells['Front shell'].intersect(_stealth_strip).Volume() < 0.01 and _shells['Grille insert'].intersect(_stealth_strip).Volume() < 0.01, 'Nose light strip interferes'
    # (The fill of the pocket beside each strip end is gone: since the hood filler closes that pocket in parts/stealth_shell.py, all it still found was
    # the 0.5 mm bevel of the hood edge over the cheek, a 6 mm3 hairline 50 mm long, which it filled back in and left a lump on the nose corner.)
    import numpy as _np
    _env = _ss.outer_solid()
    # The groove behind each strip end is open to the outside (no skin over it, marked hole): close it with a plug that fills the groove void behind the strip end,
    # between the cheek's outer skin and the hollow, bounded by the outer envelope so the outer face stays the faceted skin.
    for _sx in (1, -1):
        _A = _np.array([_sx * 56.5, -262.0]); _d = _np.array([_sx * 0.315, 0.949]); _n = _np.array([_sx * 0.949, -0.315])
        _ring = [tuple(_A + t * _d + u * _n) for t, u in ((-7, -5.0), (16, -5.0), (16, 12), (-7, 12))]      # from 6 mm ahead of the strip's end: no open recess between strip end and plug
        _prism = cq.Workplane('XY').workplane(offset=99.0).polyline(_ring).close().extrude(26.0).val()
        _plug = _env.intersect(_prism).cut(_shells['Front shell']).cut(_stealth_strip)
        for _g in (0.25, 0.5):      # the strip's fit gap (0.25 mm) at its end face
            _plug = _plug.cut(_stealth_strip.translate(cq.Vector(_d[0] * _g, _d[1] * _g, 0)))
        _plug = cq.Compound.makeCompound([q for q in _plug.Solids() if q.Volume() > 0.5])
        print('nose strip rear groove plug side', _sx, 'solids', len(_plug.Solids()), 'mm3', round(_plug.Volume(), 1))
        for _tol in (None, 1e-3):
            _t = (_shells['Front shell'].fuse(_plug) if _tol is None else _shells['Front shell'].fuse(_plug, tol=_tol)).clean()
            if _t.isValid() and len(_t.Solids()) == 1 and _t.Volume() > _shells['Front shell'].Volume():
                _shells['Front shell'] = _t; break
        else:
            raise AssertionError('groove plug did not fuse on side %d' % _sx)
    assert _shells['Front shell'].intersect(_stealth_strip).Volume() < 0.01, 'groove plug touched the strip'
    assert OPEN_COCKPIT or _shells['Front shell'].distance(_shells['Canopy']) > 0.15, 'Canopy tabs have no clearance'
    print('stealth shell: Front shell faces', len(_shells['Front shell'].Faces()), 'Canopy faces', len(_shells['Canopy'].Faces()) if 'Canopy' in _shells else '(open cockpit)')
add_part('Tail light housing',_shells['Rear shell'],color=BODY,group='Print shell')
if not STEALTH: _shells['Canopy']=_unify_flat(_shells['Canopy'],'Canopy')
# Continuous recessed backing flanges (stepped lap joints) are built directly into make_canopy() and canopy_seat_tool()
# along the entire side/rear seam, mechanically stopping both outer surfaces flush with a uniform 0.3 mm PANEL_GAP.
# travel of the new front end (upper arms with their shock posts, knuckles): cut from the finished front shell (after all later fills)
_wb_tools = swingarm_relief_tools()      # version 2: travel of the swing arm front end (was wishbone_relief_tools)
for _t in _wb_tools:
    _shells['Front shell'] = _shells['Front shell'].cut(_t)
if STEALTH:
    # These cutters and the deck trim cross the skin at an angle or only graze it, which left feather edges (the wall running out to nothing over up to
    # 8 mm beside the upper arm notch above the arch) and knife edges along the bottom line; the cabin opening is cut straight up through a deck that
    # slopes. Every such opening is squared off through the wall, facet by facet (parts/stealth_shell.py square_cuts), and its outside edge gets the
    # 0.5 mm break of the other rims as one exact facet.
    _v0 = _shells['Front shell'].Volume()
    _sq_tools = [_deck_trim] + ([_ss.cabin_cut(convex=True)] if OPEN_COCKPIT else []) + _tools + _wb_tools
    _shells['Front shell'], _sq = _ss.square_cuts(_shells['Front shell'], _sq_tools, chamfer=_ss.CHAMFER)
    print('front shell rims squared and broken 0.5 mm: %d openings (cutter x facet), mm3 removed %.1f' % (len(_sq), _v0 - _shells['Front shell'].Volume()))
_shells['Front shell'] = strip_dust(_shells['Front shell'].clean(), min_volume=1500.0)
assert _shells['Front shell'].isValid() and len(_shells['Front shell'].Solids()) == 1, 'wishbone relief broke the front shell (%d solids)' % len(_shells['Front shell'].Solids())
if not DESERT: add_part('Front shell',_shells['Front shell'],color=BODY,group='Print shell')
if 'Canopy' in _shells: add_part('Canopy',_shells['Canopy'],color=ROOF,group='Print shell')
if not DESERT: add_part('Grille insert',_shells['Grille insert'],color=(0.06,0.06,0.07),group='Print shell')
add_part('Tail light strip',_tail_strip,color=(0.85,0.05,0.05),alpha=1,group='Lights')
if DESERT: pass
elif STEALTH:
    add_part('Nose light strip',_stealth_strip,color=(0.85,0.05,0.05),alpha=1,group='Lights')   # plain bar in the slot across the nose face
else:
    _nose_strip=_unify_flat(_nose_strip,'Nose light strip')
    # E1 collision fix: the strip sat 78 mm3 into the Front shell groove; trim it flush so it touches instead of interpenetrating.
    _ns0=_nose_strip.Volume(); _nose_strip=_nose_strip.cut(_shells['Front shell']).clean()
    assert _nose_strip.isValid() and len(_nose_strip.Solids())==1 and _ns0-_nose_strip.Volume()<150,'nose strip trim failed'
    print('E1 nose strip trimmed mm3',round(_ns0-_nose_strip.Volume(),1))
    # The trim can leave a speck of dust on one side (0.004 mm3 at x 34, flagged by the kit mesh audit); the strip is symmetric,
    # so rebuild it from the half with fewer faces.
    _nose_strip=_symmetric(_nose_strip,'Nose light strip',pick=lambda h:-len(h.Faces()))
    add_part('Nose light strip',_nose_strip,color=(0.85,0.05,0.05),alpha=1,group='Lights')

# Chassis and components, imported from inputs/WS03_chassis_only.glb as named parts grouped by function
# (tools/make_chassis_ref.py: each component as a light convex-hull stand-in, standard 1:10 size, included in the print build).
_off = cq.Vector(0, CHASSIS_Y, CHASSIS_Z)
_ch = import_assembly('parts/chassis_ref.step')
_ch.loc = cq.Location(_off)
# The front deck plate is lifted so its top sits flush against the base of the front diff housing (deck and diff were 0.9-1.0 mm apart).
from parts.params import DECK_FRONT_RAISE
for _dn in _ch.objects.values():
    if getattr(_dn, 'name', None) == 'w3 deck front':
        _dv = _dn.obj.val() if isinstance(_dn.obj, cq.Workplane) else _dn.obj
        _dn.obj = _dv.translate(cq.Vector(0, 0, DECK_FRONT_RAISE))

# Deck mounting for the cage feet: M3 screws come from the top, so the decks get heat-set insert pockets (M3 x 4 mm insert, pocket D4.2 x 5 deep).
# The rail feet are outboard of the deck edge (x 54.5): a small ear (5 mm wide, 14 long, 6 thick) carries the pocket there.
from parts.cage import RAIL_FEET, RAIL_FOOT_X, RAIL_FOOT_DY, FRONT_LEG_X as _FLX, FRONT_FOOT_Y as _FFY, FRONT_TAB_DY as _FTD
from parts.cage import FRONT_TAB_Z as _FTZ, RAIL_TAB_LIFT as _RTL, FOOT_SEAT as _FSEAT
from parts import foot_sockets as _fsock
DECK_SCREWS = []   # (deck node, world x, world y, ear) for the hardware list
FRONT_FOOT_SITES = []   # cage feet on the front deck: made again on the flat front deck further down (parts/flat_deck.py replaces the old plate)
def _foot_site(deck, X, y, rail):
    """Deck (chassis frame) with the screw hole, pocket and socket for the cage foot at world (X, y). rail: bracket keyed into the side wall
    (rear deck), otherwise a free-standing collar (front deck)."""
    yr = y - CHASSIS_Y
    for _px in (53.0, 50.0, 46.0, 42.0):
        probe = deck.intersect(cq.Solid.makeBox(0.2, 0.2, 300, cq.Vector(math.copysign(min(abs(X), _px), X) - 0.1, yr - 0.1, -100)))
        if probe.Solids(): break
    assert probe.Solids(), 'no deck under the screw at x %.1f y %.1f' % (X, y)
    bb = probe.BoundingBox(); zt, zb = bb.zmax, bb.zmin
    # M3 countersunk screw from the underside: clearance hole 3.4 through the deck, 90 degree countersink 6.3 at the bottom face
    deck = deck.cut(cq.Solid.makeCylinder(1.7, zt - zb + 2.0, cq.Vector(X, yr, zb - 1.0), cq.Vector(0, 0, 1)))
    deck = deck.cut(cq.Solid.makeCone(3.15 + 1.0, 1.7, 1.45 + 1.0, cq.Vector(X, yr, zb - 1.0), cq.Vector(0, 0, 1)))
    assert deck.isValid() and len(deck.Solids()) == 1, 'deck screw hole broke the deck at x %.1f y %.1f' % (X, y)
    # pocket under the foot and the socket it slots into (parts/foot_sockets.py)
    _zf = [z for yy, z in RAIL_FEET if abs(yy - y) < 1.0][0] + _RTL - _FSEAT - CHASSIS_Z
    _add, _cut = _fsock.socket(X, yr, zt, _zf, rail)
    deck = deck.fuse(_add).clean().cut(_cut).clean()
    assert deck.isValid() and len(deck.Solids()) == 1, 'foot socket broke the deck at x %.1f y %.1f' % (X, y)
    return deck
def _deck_pockets(assembly):
    nodes = {n.name: n for n in assembly.objects.values() if getattr(n, 'name', None) in ('w3 deck front', 'w3 deck rear')}
    for X, y in [(sx*RAIL_FOOT_X, y) for y, _ in RAIL_FEET for sx in (-1, 1)]:
        front = y - CHASSIS_Y < -71.0
        node = nodes['w3 deck front'] if front else nodes['w3 deck rear']
        if front:
            FRONT_FOOT_SITES.append((X, y))
        else:
            node.obj = _foot_site(node.obj.val() if isinstance(node.obj, cq.Workplane) else node.obj, X, y, True)
        DECK_SCREWS.append((node.name, X, y, False))
    print('deck insert pockets:', len(DECK_SCREWS))
# Clean up deck plates: plug unused leftover holes from the old layout, widen rear cage foot margin (markup 13:41),
# and extend w3 deck tail to y_ref = 139.0 to fully support w3 rbc lower and its rear slots (markup 13:42).
from parts.deck import band_pieces, clean_deck_plates
clean_deck_plates(_ch)
# Debris walls along both side edges of the rear (centre) deck (parts/deck_walls.py), fused into the plate.
from parts.deck_walls import make_deck_walls
_dr = [n for n in _ch.objects.values() if getattr(n, 'name', None) == 'w3 deck rear']
assert len(_dr) == 1, 'rear deck node not found'
_d0 = _dr[0].obj.val() if isinstance(_dr[0].obj, cq.Workplane) else _dr[0].obj
_d1 = _d0
for _w in make_deck_walls(_d0):
    _d1 = _d1.fuse(_w)
_d1 = _d1.clean()
assert _d1.isValid() and len(_d1.Solids()) == 1, 'deck walls broke the rear deck'
print('rear deck walls added, mm3', round(_d1.Volume() - _d0.Volume(), 1))
_dr[0].obj = _d1
_dt = [n for n in _ch.objects.values() if getattr(n, 'name', None) == 'w3 deck tail']
assert len(_dt) == 1, 'tail deck node not found'
from parts.deck_walls import make_tail_walls
_t0 = _dt[0].obj.val() if isinstance(_dt[0].obj, cq.Workplane) else _dt[0].obj
_t1 = _t0
for _w in make_tail_walls(_t0):
    _t1 = _t1.fuse(_w)
_t1 = _t1.clean()
assert _t1.isValid() and len(_t1.Solids()) == 1, 'deck walls broke the tail deck (%d solids)' % len(_t1.Solids())
print('tail deck walls added, mm3', round(_t1.Volume() - _t0.Volume(), 1))
_dt[0].obj = _t1
# The ballast carrier, lid and plate (hung under the tail deck) are dropped: no ballast pocket in this build.
for _bn in [n for n in _ch.objects.values() if getattr(n, 'name', None) in ('w3 ballast carrier', 'w3 ballast lid', 'w3 ballast plate')]:
    _bn.parent.children.remove(_bn)
_ch.objects = {k: v for k, v in _ch.objects.items() if getattr(v, 'name', None) not in ('w3 ballast carrier', 'w3 ballast lid', 'w3 ballast plate')}
_nodes_d = {n.name: n for n in _ch.objects.values() if getattr(n, 'name', None) in ('w3 deck front', 'w3 deck rear', 'w3 deck tail')}
for _band, _dn in band_pieces([_shells[k] for k in ('Front shell', 'Canopy', 'Rear shell') if k in _shells]):
    _node = _nodes_d[_dn]
    _deck = _node.obj.val() if isinstance(_node.obj, cq.Workplane) else _node.obj
    _deck2 = _deck.fuse(_band.translate(-_off)).clean()
    assert _deck2.isValid() and len(_deck2.Solids()) == 1, 'deck band broke ' + _dn
    _node.obj = _deck2
_deck_pockets(_ch)
_nm0 = lambda _n: str(getattr(_n, 'name', ''))
_ob0 = lambda _n: (_n.obj.val() if isinstance(_n.obj, cq.Workplane) else _n.obj)
_deckf = [n for n in _ch.objects.values() if _nm0(n) == 'w3 deck front'][0]
# Deck front corners: the diagonal bevel (54.5, -200) -> (37.3, -212.5) left an open triangle beside the skid. A gusset closes it: the corner now runs
# (54.5, -200) -> (54.5, -205) -> (40, -212.5), following the skid side, and has the same thickness and slope as the deck plate (world frame, converted to ref).
_gp = [(37.0, -212.5), (40.0, -212.5), (54.5, -205.0), (54.5, -199.0), (50.0, -199.0)]
_slope = (19.6 - 18.7) / 12.0   # deck top rise per mm of y between y -212 and -200
_gzb = lambda y: 12.7 + (y + 212.0) * _slope + DECK_FRONT_RAISE * 0.0
for _sg in (1, -1):
    _pr = cq.Workplane('XY').polyline([(_sg * x, y - 9.0) for x, y in _gp]).close().extrude(80).translate(cq.Vector(0, 0, -20))
    _y0, _y1 = -213.0, -198.0
    _sl = (cq.Workplane('YZ').polyline([(_y0 - 9.0, _gzb(_y0) + 15.3), (_y1 - 9.0, _gzb(_y1) + 15.3), (_y1 - 9.0, _gzb(_y1) + 15.3 + 6.0), (_y0 - 9.0, _gzb(_y0) + 15.3 + 6.0)]).close().extrude(200).translate(cq.Vector(-100, 0, 0)))
    _gus = _pr.intersect(_sl).val()
    _d0 = _ob0(_deckf)
    _d1 = _d0.fuse(_gus).clean()
    assert _d1.isValid() and len(_d1.Solids()) == 1, 'deck corner gusset broke the front deck'
    print('deck corner gusset side', _sg, 'added mm3', round(_d1.Volume() - _d0.Volume(), 1))
    _deckf.obj = _d1
# Rear wheel drive only (parts/rwd.py): the front belts, pulleys, front and centre differential and their housings are removed,
# the motor drives the rear shaft through one spur stage.
from parts.pulleys import make_pulley as _mkp, belt_loop as _mkbelt, belt_length as _blen, tooth_od as _tod
from parts import rwd as _rwd
_nn = lambda n: str(getattr(n, 'name', ''))
def _bb0(n):
    _o = _ob0(n); _b = _o.BoundingBox(); return _b, ((_b.ymin + _b.ymax) / 2, (_b.zmin + _b.zmax) / 2)
_piv_yz, _rs_front, _rs_dir = _rwd.remove_front_drive(_ch)
print('rwd: front arm pivot (y, z)', [round(v, 2) for v in _piv_yz], 'rear shaft front end', [round(v, 1) for v in _rs_front.toTuple()])
from parts import chamfer as _cmf      # 0.5 mm chamfers on the printed moving parts (owner, Nov 24; NOTES 30)
_CHAMFER_LOG = []
_CHAMFER_PARTS = ('Front carrier', 'Front knuckle', 'Rocker', 'Droop stop pad', 'Balance beam', 'Beam carrier', 'Servo floor', 'Servo clip', 'Heave saddle', 'Yaw stop pad')   # printed (TPU included); 'Rocker' also matches 'Rocker bracket'; hardware ('Rocker pin', 'Rocker ball stud') excluded below
def _chamfered(_s, _name):
    # the rocker's two arm plates meet in the plane z = 30: chamfers on the step edges there left slivers that exported as an open mesh
    _skip = (lambda _e: abs(_e.Center().z - _sa2.ROCKER_PLATES_Z[0][1]) < 0.05) if _name.startswith('Rocker ') and not _name.startswith('Rocker bracket') else None
    _r, _rep = _cmf.chamfer_part(_s, _name, skip=_skip)
    assert _rep['fits_kept'], 'chamfer changed a fit diameter on ' + _name
    _CHAMFER_LOG.append(_rep)
    print('chamfer %-22s corners %3d sharp %3d size %.1f dvol %.1f mm3 (%.2f %%) fits kept %s' % (_name, _rep['edges'], _rep['sharp'], _rep['size'], _rep['dvol'], 100 * _rep['dvol'] / _s.Volume(), _rep['fits_kept']))
    return _r
_dfn = [n for n in _ch.objects.values() if _nn(n) == 'w3 deck front'][0]
_drn = [n for n in _ch.objects.values() if _nn(n) == 'w3 deck rear'][0]
for _dn_, _ds_, _dc_ in _rwd.drive(_rs_front, _rs_dir):
    if _dn_ == 'w3 motor plate':
        _v0 = _ds_.Volume()
        for _dkn in (_drn, _dfn): _ds_ = _ds_.cut(_ob0(_dkn))
        _ds_ = max(_ds_.Solids(), key=lambda q: q.BoundingBox().zmax)   # the piece above the deck
        print('rwd: motor plate seated on the deck, removed mm3', round(_v0 - _ds_.Volume(), 1), 'z', round(_ds_.BoundingBox().zmin, 1), round(_ds_.BoundingBox().zmax, 1))
        assert _v0 - _ds_.Volume() > 50.0, 'motor plate does not reach the deck'
    if _dn_ == 'w3 motor plate': _ds_ = _chamfered(_ds_, 'Motor plate')
    _ch.add(_ds_, name=_dn_, color=cq.Color(*_dc_))
# Printed 6-spoke wheels (parts/wheels.py) replace the solid rim stand-ins: 1.9 in front wheels (bead seat 48.3 mm, 34 mm wide) carry bought rubber tyres.
# The tyre stand-in becomes a ring around the wheel's flange lips (a bought part: it only has to show where the rubber is).
from parts.wheels import make_wheel as _mkw, bead_seat as _bseat, tyre_seat_radius as _tsr, FRONT_BUSH as _FRONT_BUSH
FRONT_WHEEL_IN, FRONT_WHEEL_W = 1.9, 34.0
FRONT_WHEEL_X = 144.0    # wheel centre plane (tyre centre)
def _relieve(_w, _s, _d=0.2):
    """Cut `_s` out of `_w`, then again shifted 0.2 mm each way (sequential cuts: fusing the shifted copies fails on the faceted stand-ins)."""
    _r = _w.cut(_s)
    for _v in ((_d, 0, 0), (-_d, 0, 0), (0, _d, 0), (0, -_d, 0), (0, 0, _d), (0, 0, -_d)): _r = _r.cut(_s.translate(cq.Vector(*_v)))
    return _r
for _sx in (1, -1):
    _rim = [n for n in _ch.objects.values() if _nn(n) == 'mock rim %d' % _sx]
    _rdc = [n for n in _ch.objects.values() if _nn(n) == 'mock rimdisc %d' % _sx]
    _tyr = [n for n in _ch.objects.values() if _nn(n) == 'mock tire %d' % _sx]
    assert len(_rim) == len(_rdc) == len(_tyr) == 1, 'front rim / tyre stand-ins not found'
    _rb_, _rc = _bb0(_rim[0])
    _xc = (_rb_.xmin + _rb_.xmax) / 2
    _w = _mkw(_bseat(FRONT_WHEEL_IN), FRONT_WHEEL_W, axle_bore=4.4)
    if _sx < 0: _w = _w.mirror('YZ')
    _w = _w.translate(cq.Vector(_xc, *_rc))
    for _n in (_rim[0], _rdc[0]): _n.parent.children.remove(_n)
    _ch.add(_w, name='wheel f %d' % _sx, color=cq.Color(0.72, 0.45, 0.30))
    _t0 = _ob0(_tyr[0])
    _t1 = _t0.cut(cq.Solid.makeCylinder(_tsr(_bseat(FRONT_WHEEL_IN)), 80, cq.Vector(_xc - 40, *_rc) if False else cq.Vector(_xc - 40, _rc[0], _rc[1]), cq.Vector(1, 0, 0)))
    assert _t1.isValid() and len(_t1.Solids()) == 1, 'tyre ring broke'
    _tyr[0].obj = _t1
    print('front wheel side', _sx, 'centre x %.1f y %.1f z %.1f' % (_xc, *_rc), 'wheel mm3 %.0f' % _w.Volume(), 'tyre ring mm3 %.0f' % _t1.Volume())
# Old lean mechanism (servo, mount, ears, horn, rocker, lean shock stand-ins) and the lower front differential box: removed. Passive lean now runs
# through the balance beam of parts/lean_beam.py (added with the cage further down).
for _lname in ('lean servo c', 'w3 fdc lower', 'lean mount c', 'lean ears c', 'lean horn c', 'lean rocker c', 'b lean shock 1', 'b lean shock -1'):
    _lf = [n for n in _ch.objects.values() if str(getattr(n, 'name', '')).replace('_', ' ') == _lname]
    assert len(_lf) == 1, 'lean node not found: ' + _lname
    _lf[0].parent.children.remove(_lf[0])
# Version 2 front end (parts/swingarm_front.py, hard points in parts/swingarm2.py): the leading swing arms of November 21 are back. Borrowed: the printed
# truss arms (parts/frontarm.py), the upper links (parts/uplink.py), pivot axle, pivot blocks and tip pins (parts/rwd.py), carriers and knuckles (stand-ins).
# New: a spring lever on each arm root, one balance beam on the two lever tops, one heave shock on the centre line under a cage spine, the servos in a new
# place and tie rods parallel to the arms. The flat front deck of the wishbone version stays.
from parts import swingarm_front as _sf, swingarm2 as _sa2
from parts import uplink as _up
from parts.frontarm import make_front_arm
from parts.flat_deck import make_flat_deck, lap_skid as _lap_skid
_big = lambda _o: max(_o.Solids(), key=lambda q: q.Volume())
_flat = make_flat_deck(_ob0(_dfn), _ob0(_drn))                 # ends in a butt seam on the rear deck's front face
for _X, _y in FRONT_FOOT_SITES:                              # cage feet on the flat front deck: screw hole, pocket and collar
    _flat = _foot_site(_flat, _X, _y, False)
_skn = [n for n in _ch.objects.values() if _nn(n) == 'w3 skid'][0]
_flat, _skn.obj = _lap_skid(_flat, _ob0(_skn))                 # the skid's flange is let into the underside of the deck (stepped lap, 2 x M3)
# battery and its tray are gone (owner, November 22): both stand-ins are removed from the chassis
for _nm in ('b battery', 'battery tray c'):
    _bn = [n for n in _ch.objects.values() if _nn(n) == _nm][0]
    _bn.parent.children.remove(_bn)
# receiver: on the centre line of the deck between the two steering servos
_rcn = [n for n in _ch.objects.values() if _nn(n) == 'mock receiver'][0]
_rb = _ob0(_rcn).BoundingBox().center
_rc1 = _ob0(_rcn).translate(cq.Vector(0.0 - _rb.x, (-90.0 - _off.y) - _rb.y, 0)); _lift = 0.0
while _lift > -30.0 and _rc1.intersect(_flat).Volume() < 0.01:
    _rc1 = _rc1.translate(cq.Vector(0, 0, -0.25)); _lift -= 0.25
while _lift < 30.0 and _rc1.intersect(_flat).Volume() > 0.01:
    _rc1 = _rc1.translate(cq.Vector(0, 0, 0.25)); _lift += 0.25
_rcn.obj = _rc1
def _fnode(_name):
    _f = [n for n in _ch.objects.values() if _nn(n) == _name]
    assert len(_f) == 1, 'front end node not found: ' + _name
    return _f[0]
assert abs(_piv_yz[0] + _off.y - _sa2.PIVOT[0]) < 0.2 and abs(_piv_yz[1] + _off.z - _sa2.PIVOT[1]) < 0.2, 'arm pivot differs from parts/swingarm2.py'
_rwd.BLOCK_X = _sa2.PIVOT_BLOCK_X          # the axle's deck blocks stand inboard of the servos
_ch.add(_rwd.front_axle(_piv_yz), name='b front pivot axle', color=cq.Color(0.78, 0.80, 0.84))
_hi, _ho = _up.hinges()
for _sx in (1, -1):
    # stand-ins that are replaced by parts of their own (the old carrier and knuckle sat inside the wheel: wheel <> carrier 90 mm3 at rest)
    for _nm in ('steer servo %d', 'steer horn %d', 'tie rod %d', 'carrier %d', 'knuckle %d', 'mock axle %d'):
        _n0 = _fnode(_nm % _sx); _n0.parent.children.remove(_n0)
    # printed truss arm with the spring lever on its root
    _arm = make_front_arm(_sx).fuse(_sf.lever_ref(_sx)).clean()
    if _os.environ.get('FUGU_FEM_BORE_SEG') == '1':
        _arm = _arm.cut(_sf.stud_bore_seg(_sx))            # FEM only: stud bore wall split at the blade's mid plane (NOTES 58)
    assert _arm.isValid() and len(_arm.Solids()) == 1, 'spring lever broke the front arm (%d solids)' % len(_arm.Solids())
    _sa2.guard('arm %d' % _sx, _arm)
    _fnode('arm %d' % _sx).obj = _arm
    # pivot block on the flat deck, screws from below, tip shaft cut back to a pin
    _pb = _chamfered(_rwd.pivot_block(_sx, _piv_yz, _flat), 'Arm pivot block %d' % _sx)
    _ch.add(_pb, name='w3 arm pivot block %d' % _sx, color=cq.Color(0.55, 0.57, 0.60))
    for _hh in _rwd.deck_screw_holes(_sx, _piv_yz): _flat = _flat.cut(_hh)
    _tp = _fnode('tip shaft %d' % _sx); _tp.obj = _rwd.tip_pin(_ob0(_tp), _sx)
    # upper link (its outer clevis is part of the new carrier, the inner clevis goes on the cage further down)
    # FEM only (FUGU_FEM_NO_CHAMFER=uplink, NOTES 52): the link's 0.4 mm chamfers leave sliver faces (0.0002 mm2) at the tube ends that the mesher
    # cannot take; the FEM runs on the sharp link (on the safe side). The build never sets it.
    _fnode('uplink %d' % _sx).obj = _up.link(_sx) if _os.environ.get('FUGU_FEM_NO_CHAMFER') == 'uplink' else _chamfered(_up.link(_sx), 'Upper link %d' % _sx)
    print('front end side', _sx, 'arm mm3', round(_arm.Volume()), 'pivot block mm3', round(_pb.Volume()))
for _sx in (1, -1):   # front wheels centred on the axle (wheel centre of the tyre): both wheels are exact mirror images
    _wn = [n for n in _ch.objects.values() if _nn(n) == 'wheel f %d' % _sx]
    _tn_ = [n for n in _ch.objects.values() if _nn(n) == 'mock tire %d' % _sx]
    assert len(_wn) == 1 and len(_tn_) == 1, 'front wheel / tyre not found'
    _tc = _ob0(_tn_[0]).BoundingBox().center
    assert abs(abs(_tc.x) - FRONT_WHEEL_X) < 0.2 and abs(_tc.y + _off.y - _sa2.TIP[0]) < 0.2 and abs(_tc.z + _off.z - _sa2.TIP[1]) < 0.2, 'tyre is not on the axle'
    _wc = cq.Vector(_sx * FRONT_WHEEL_X, _sa2.TIP[0] - _off.y, _sa2.TIP[1] - _off.z)    # chassis frame, like the tyre
    _w = _mkw(_bseat(FRONT_WHEEL_IN), FRONT_WHEEL_W, axle_bore=4.4, hub_out=1.0, bush=_FRONT_BUSH)   # flanged bronze bushing, no hex (owner: "plain bushings")
    if _sx < 0: _w = _w.mirror('YZ')
    _w = _chamfered(_w, 'Front wheel %d' % _sx)
    _wn[0].obj = _w.translate(_wc)
    _t1 = _ob0(_tn_[0]).cut(cq.Solid.makeCylinder(_tsr(_bseat(FRONT_WHEEL_IN)), 80, _wc - cq.Vector(40, 0, 0), cq.Vector(1, 0, 0)))
    assert _t1.isValid() and len(_t1.Solids()) == 1, 'tyre ring broke'
    _tn_[0].obj = _t1
for _sx in (1, -1):      # M3 clearance hole in the deck under the servo clip's insert (screw from below through deck and floor tongue)
    _flat = _flat.cut(cq.Solid.makeCylinder(1.7, 60.0, cq.Vector(_sx * 25.0, -98.9, -10.0) - _off, cq.Vector(0, 0, 1)))
assert _flat.isValid() and len(_flat.Solids()) == 1, 'pivot block screw holes broke the flat deck'
# Rear ring gear (bought, stand-in): the stand-in was a solid disc, so the printed ring hub and the rear shaft ran through it (391 mm3, found by the
# all-pairs pose check). It gets its bore; the hub's bore is opened to the shaft (0.1 mm3 of faceting).
_rgn, _rhn, _rsn = _fnode('b rear ring'), _fnode('w3 rring hub'), _fnode('b rear shaft')
_rg1 = _big(_ob0(_rgn).cut(_ob0(_rhn)).cut(_ob0(_rsn))); _rh1 = _big(_ob0(_rhn).cut(_ob0(_rsn)))
assert _rg1.isValid() and _rh1.isValid(), 'rear ring bore failed'
print('rear ring bore mm3', round(_ob0(_rgn).Volume() - _rg1.Volume(), 1), 'hub bore mm3', round(_ob0(_rhn).Volume() - _rh1.Volume(), 2))
_rgn.obj, _rhn.obj = _rg1, _rh1
_dfn.obj = _flat
_pod_lower = []
if DESERT:
    # Side pods' lower edge (owner, November 24: "yes, add it"): a short bracket off the rear deck's side, under each pod's lower skin, with a pilot
    # for an M3 screw from outside through the pod. The pods are built here already (their inner skin sets the bracket's length) and reused below.
    from parts import desert_shell as _ds
    _d_pods_early = dict(zip((1, -1), _ds.build_side_pods()))
    _dr_w = _ob0(_drn).moved(_drn.loc).moved(cq.Location(_off))
    _dr_x = _dr_w.BoundingBox().xmax                                     # deck side face (54.5)
    _brk = None
    for _sx in (1, -1):
        for _py in _ds.POD_LOWER_Y:
            _pz, _r = _ds.POD_LOWER_Z, _ds.POD_LOWER_D / 2.0
            _hits = [_ds.skin_x(_d_pods_early[_sx], _py + _dy, _pz + _dz, _sx) for _dy in (-_r, 0.0, _r) for _dz in (-_r, 0.0, _r)]
            assert _hits[4] is not None, 'no pod skin at the lower screw y %.0f z %.0f' % (_py, _pz)
            _inner = min(abs(_h[0]) for _h in _hits if _h is not None)      # corners under the pod's bottom edge have no skin: they do not limit the bracket
            _x_end = round(_inner - _ds.FIT_CUT, 2)
            _x0 = _dr_x - _ds.POD_LOWER_EMBED
            _b = cq.Solid.makeCylinder(_r, _x_end - _x0, cq.Vector(_sx * _x0, _py, _pz), cq.Vector(_sx, 0, 0))
            _brk = _b if _brk is None else _brk.fuse(_b)
            _pod_lower.append((_sx, _py, _pz, _x_end))
            print('pod lower bracket side', _sx, 'y', _py, 'z', _pz, 'x', round(_x0, 1), '..', _x_end)
    _to_node = lambda _s: _s.moved(cq.Location(-_off)).moved(_drn.loc.inverse)
    _v0 = _ob0(_drn).Volume()
    _dr_new = _ob0(_drn).fuse(_to_node(_brk)).clean()
    for _sx, _py, _pz, _x_end in _pod_lower:      # pilot through the bracket and POD_LOWER_PILOT_DEPTH on into the deck plate
        _x1 = _dr_x - _ds.POD_LOWER_PILOT_DEPTH
        _dr_new = _dr_new.cut(_to_node(cq.Solid.makeCylinder(_ds.SCREW_PILOT / 2.0, _x_end + 1.0 - _x1, cq.Vector(_sx * _x1, _py, _pz), cq.Vector(_sx, 0, 0))))
    assert _dr_new.isValid() and len(_dr_new.Solids()) == 1, 'pod lower brackets broke the rear deck'
    _drn.obj = _dr_new
    print('rear deck: pod lower brackets added, mm3', round(_dr_new.Volume() - _v0, 1))
# Pockets under the steering servos' rear ears (worst-case servo envelope, parts/swingarm2.py SERVO_ENV; NOTES 27): the rear ear hangs 1 mm into
# the deck tops at the deck seam, so both decks get a pocket 1 mm under it (the servo stays at its height; raising it put the clip into the cage rail).
_e = _sa2.SERVO_ENV; _tx0, _tx1 = _sa2.SERVO_TAB_X
_ry0 = _sa2.SERVO_SHAFT_B[0] - _sa2.SERVO_SHAFT_IN + _e['L'] - 0.5
_zb = _sa2.SERVO_SHAFT_B[1] - _e['W'] / 2 - 1.0
for _dn in (_dfn, _drn):
    _o = _ob0(_dn); _v0 = _o.Volume()
    for _sx in (1, -1):
        _bx = cq.Solid.makeBox(_tx1 - _tx0 + 1.0, _e['ear_overhang'] + 1.0, 12.0, cq.Vector(_tx0 - 0.5 if _sx > 0 else -_tx1 - 0.5, _ry0, _zb))
        _o = _o.cut(_bx.moved(cq.Location(-_off)).moved(_dn.loc.inverse))
    _o = max(_o.Solids(), key=lambda q: q.Volume()) if len(_o.Solids()) > 1 else _o
    assert _o.isValid(), 'servo ear pocket broke a deck'
    _dn.obj = _o
    print('servo rear ear pockets in', _nn(_dn), 'mm3', round(_v0 - _o.Volume(), 2))
_skid_nodes = None
if DESERT:
    # Lower front cage on the skid (parts/exo_cage_front.py skid_nodes): the nodes come from the skid's outline; the skid is shortened so it ends
    # SKID_GAP behind the V bar, which becomes its front edge. The node at the front corner is built here with the same members as in the cage
    # (V bar, skid rail, V leg), so the skid is trimmed against the real joint, not only the bar.
    from parts import exo_cage_front as _ex
    from parts.cage import _Frame
    _sk_w = _ob0(_skn).moved(_skn.loc).moved(cq.Location(_off))
    _sk_out = _ex.skid_outline(_sk_w)
    _dk_front_y = _ob0(_dfn).moved(_dfn.loc).moved(cq.Location(_off)).BoundingBox().ymin
    _skid_nodes = _ex.skid_nodes(_sk_out, _dk_front_y)
    print('skid outline', {k: (tuple(round(c, 2) for c in v) if isinstance(v, tuple) else round(v, 2)) for k, v in _sk_out.items()}, 'deck front y', round(_dk_front_y, 2))
    print('skid nodes', {k: _skid_nodes[k] for k in ('C', 'R', 'T', 'slope_deg')})
    _F = _Frame(hull_nodes=True)
    _m = lambda p: (-p[0], p[1], p[2])
    for _a, _b in [(_m(_skid_nodes['C']), _skid_nodes['C'])] + [(_f(_skid_nodes['C']), _f(_q)) for _f in (lambda p: p, _m) for _q in (_skid_nodes['R'], _ex.V_MID)]:
        _F.tube(_a, _b, _ex.EXO_D)
    _vbar = _F.build()
    _fc = _ds.FIT_CUT
    _v0 = _sk_w.Volume(); _sk_t = _sk_w
    for _dd in [(0, 0, 0), (_fc, 0, 0), (-_fc, 0, 0), (0, _fc, 0), (0, -_fc, 0), (0, 0, _fc), (0, 0, -_fc)]:
        _sk_t = _sk_t.cut(_vbar.translate(cq.Vector(*_dd)))
    _pieces = sorted(_sk_t.Solids(), key=lambda q: -q.Volume())
    assert sum(q.Volume() for q in _pieces[1:]) < 20.0, 'skid trim left loose pieces of %.1f mm3' % sum(q.Volume() for q in _pieces[1:])
    _sk_t = _pieces[0].clean()
    assert _sk_t.isValid(), 'skid trim left an invalid solid'
    _skn.obj = _sk_t.moved(cq.Location(-_off)).moved(_skn.loc.inverse)
    print('skid shortened to the V bar, mm3 removed', round(_v0 - _sk_t.Volume(), 1), 'new front y', round(_sk_t.BoundingBox().ymin, 2))
add_part('Chassis', _ch, alpha=1, color=(0.55, 0.57, 0.60))
from parts import swingarm_guide as _sg
_deck_w = _flat.moved(cq.Location(_off))
for _n, _s, _g, _c in _sf.parts() + _sg.parts():
    if _n.startswith('Servo floor'):      # the wedge floor is built a little into the deck and cut by it: it sits exactly on the deck top
        _v0 = _s.Volume(); _s = max(_s.cut(_deck_w).Solids(), key=lambda q: q.BoundingBox().zmax)
        assert _s.isValid(), 'servo floor broke when seated on the deck'
        print('seated on the deck:', _n, 'removed mm3', round(_v0 - _s.Volume(), 1))
    _sa2.guard(_n, _s)        # load-bearing parts: one solid, volume within 1 % of parts/swingarm2.py GUARD_VOLUMES, else the build fails
    # the rocker itself is NOT chamfered: its chamfered solid is valid but tessellates open (4 bodies) in every variant tried (0.5, 0.3, no circles,
    # clean, ShapeFix), so the kit's mesh audit and the slicer reject it (NOTES 30)
    if _n.startswith(_CHAMFER_PARTS) and not _n.startswith(('Rocker pin', 'Rocker ball stud', 'Rocker 1', 'Rocker -1')): _s = _chamfered(_s, _n)   # after the guard: the chamfer helper checks one valid solid, volume and fit radii
    # registered without clean_registered(): a .clean() on a ball stud (two fused hemispheres) merges the halves and crashed the build (exit 139);
    # these shapes are cleaned where they are built
    _register_part(_n, _s, group=_g, alpha=1, color=_c)
# Rear swingarm group rotates about its pivot to carry the bigger rear tire; the axle rises with the tire radius.
# Redesigned as one printable high-strength part (parts/swingarm.py): twin ladder-truss plates around the belt, arch over the top ending in twin shock ears.
from parts.swingarm import make_rear_arm
_th = rear_lift_angle()
_ry = PIVOT[0]*math.cos(_th) - PIVOT[1]*math.sin(_th); _rz = PIVOT[0]*math.sin(_th) + PIVOT[1]*math.cos(_th)
_rr = import_assembly('parts/rear_ref.step')
_arm_nodes = [n for n in _rr.objects.values() if getattr(n, 'name', None) == 'w3 rear arm']
assert len(_arm_nodes) == 1, 'rear arm node not found in parts/rear_ref.step'
_arm_nodes[0].obj = make_rear_arm()
_rr.loc = cq.Location(cq.Vector(0, PIVOT[0]-_ry+CHASSIS_Y, PIVOT[1]-_rz+CHASSIS_Z), cq.Vector(1, 0, 0), math.degrees(_th))
# Rear arm belt drive: printed GT2 pulleys 26T (pivot) and 30T (axle) and a bought 6 mm GT2 closed loop belt (2GT-300). The ratio 26:30 keeps the rear wheel speed with the
# 140 mm tyre (the old 22T:20T pair on a 112 mm tyre, x 112/140).
_nr = lambda n: str(getattr(n, 'name', ''))
_rp = [n for n in _rr.objects.values() if _nr(n) == 'mock rear pulley pivot']
_ra = [n for n in _rr.objects.values() if _nr(n) == 'mock rear pulley axle']
_rb = [n for n in _rr.objects.values() if _nr(n) == 'mock rear belt']
assert len(_rp) == len(_ra) == len(_rb) == 1, 'rear pulley stand-ins not found'
_bp, _bq = _rp[0].obj.val().BoundingBox() if isinstance(_rp[0].obj, cq.Workplane) else _rp[0].obj.BoundingBox(), None
_ba = _ra[0].obj.val().BoundingBox() if isinstance(_ra[0].obj, cq.Workplane) else _ra[0].obj.BoundingBox()
_cp = ((_bp.ymin + _bp.ymax) / 2, (_bp.zmin + _bp.zmax) / 2); _ca = ((_ba.ymin + _ba.ymax) / 2, (_ba.zmin + _ba.zmax) / 2)
_xr = (_bp.xmin + _bp.xmax) / 2 + 1.6     # 1.6 mm toward the outer plate: the flange stands 0.4 mm off the inner plate face (x 52.3)
_rax = [n for n in _rr.objects.values() if _nr(n) == 'mock rear axle']
assert len(_rax) == 1, 'rear axle stand-in not found'
_bx = _rax[0].obj.val().BoundingBox() if isinstance(_rax[0].obj, cq.Workplane) else _rax[0].obj.BoundingBox()
_ca = ((_bx.ymin + _bx.ymax) / 2, (_bx.zmin + _bx.zmax) / 2)   # axle pulley centred on the steel axle
_dr = math.hypot(_cp[0] - _ca[0], _cp[1] - _ca[1])
print('rear belt centre distance %.2f' % _dr, 'length 26/30 %.2f (2GT-300)' % _blen(26, 30, _dr))
assert abs(_blen(26, 30, _dr) - 300.0) < 1.0, 'rear belt is not the stock length'
for _n in (_rp[0], _ra[0], _rb[0]): _n.parent.children.remove(_n)
_rr.add(_mkp(26, 5.0, side=1).translate(cq.Vector(_xr, *_cp)), name='pulley r pivot', color=cq.Color(0.2, 0.2, 0.22))
_rr.add(_mkp(30, 8.0, side=1).translate(cq.Vector(_xr, *_ca)), name='pulley r axle', color=cq.Color(0.2, 0.2, 0.22))
_rr.add(_mkbelt(_cp, 26, _ca, 30, _xr), name='b gt2 belt r', color=cq.Color(0.08, 0.08, 0.08))
# Rear wheel: 2.2 in printed 6-spoke wheel (bead seat 55.9 mm, 40 mm wide), outer face toward -x where the hub stand-in sits.
_rrim = [n for n in _rr.objects.values() if _nr(n) == 'mock rear rim']
_rrdc = [n for n in _rr.objects.values() if _nr(n) == 'mock rear rim disc']
assert len(_rrim) == len(_rrdc) == 1, 'rear rim stand-ins not found'
_rbb = _rrim[0].obj.val().BoundingBox() if isinstance(_rrim[0].obj, cq.Workplane) else _rrim[0].obj.BoundingBox()
_rrc = ((_rbb.xmin + _rbb.xmax) / 2, (_rbb.ymin + _rbb.ymax) / 2, (_rbb.zmin + _rbb.zmax) / 2)
REAR_WHEEL_IN, REAR_WHEEL_W = 2.2, 46.0
for _n in (_rrim[0], _rrdc[0]): _n.parent.children.remove(_n)
_rhub = [n for n in _rr.objects.values() if _nr(n) == 'mock rear hub']
assert len(_rhub) == 1, 'rear hub stand-in not found'
_rw = _mkw(_bseat(REAR_WHEEL_IN), REAR_WHEEL_W, axle_bore=8.4, hub_od=22.0).mirror('YZ').translate(cq.Vector(*_rrc))
_rw0 = _rw; _rw = _relieve(_rw, _rhub[0].obj.val() if isinstance(_rhub[0].obj, cq.Workplane) else _rhub[0].obj); _rw = max(_rw.Solids(), key=lambda q: q.Volume())   # pocket for the hub stand-in
print('rear wheel relief around the hub mm3', round(_rw0.Volume() - _rw.Volume(), 1))
_rw = _chamfered(_rw, 'Rear wheel')
_rr.add(_rw, name='wheel r', color=cq.Color(0.72, 0.45, 0.30))
add_part('Rear swingarm', _rr, alpha=1, color=(0.55, 0.57, 0.60))
_ay = PIVOT[0] + (REAR_AXLE[0]-PIVOT[0])*math.cos(_th) - (REAR_AXLE[1]-PIVOT[1])*math.sin(_th)
_az = PIVOT[1] + (REAR_AXLE[0]-PIVOT[0])*math.sin(_th) + (REAR_AXLE[1]-PIVOT[1])*math.cos(_th)
_tire = cq.Solid.makeCylinder(REAR_TIRE_OD/2, 62, cq.Vector(-31, _ay, _az), cq.Vector(1, 0, 0)).translate(_off)
_tire = _tire.cut(cq.Solid.makeCylinder(_tsr(_bseat(REAR_WHEEL_IN)) + 0.2, 80, cq.Vector(-40, _ay, _az), cq.Vector(1, 0, 0)).translate(_off))   # ring around the wheel lips
add_part('Rear tire', _tire, group='Chassis', alpha=1, color=(0.10, 0.10, 0.11))
# Rear suspension. The original shock mount rod over the tire was cut from the arm (it sat inside the 140 mm tire, tools/make_chassis_ref.py).
# The shock now stands on the wheel centre plane (x = 0): lower eye on a printed bridge that hooks over the tire from the arm apex node,
# upper eye on the pin nodes of the cage tube frame. Eyes in the final frame at ride height.
from parts.relief import BRIDGE_ARM
_eb = cq.Vector(0.0, rot_yz(SHOCK_BOT0, _th)[1], rot_yz(SHOCK_BOT0, _th)[2]) + _off                 # lower eye (on the bridge pin)
_et = cq.Vector(SHOCK_TOP0[0], SHOCK_TOP0[1], SHOCK_TOP0[2] + REAR_SHOCK_RISE) + _off               # upper eye (tower pin)
_u = (_et - _eb).normalized(); _L = (_et - _eb).Length
def _cyl(r, ln, p, d): return cq.Solid.makeCylinder(r, ln, p, d)
_X = cq.Vector(1, 0, 0)
def _eye(c): return _cyl(7.0, 8.0, c - _X*4.0, _X).cut(_cyl(3.2, 10.0, c - _X*5.0, _X))   # eye ring OD 14, bore for an M6 pin
# Damper body at the top (sprung mass, starting 10.5 mm below _et to clear the top clevis ears) and shaft at the bottom (unsprung mass)
_shock = _cyl(9.0, 0.48*_L, _et - _u*(10.5 + 0.48*_L), _u).fuse(_cyl(2.5, 0.52*_L - 7.5, _eb + _u*7.5, _u))
_shock = _shock.fuse(_cyl(3.5, 5.5, _et - _u*11.0, _u)).fuse(_cyl(3.5, 4.0, _eb + _u*5.5, _u)).fuse(_eye(_eb)).fuse(_eye(_et))
for _eye_centre in (_eb,_et):
    _shock=_shock.cut(_cyl(3.2,24.0,_eye_centre-_X*12.0,_X))
_shock=_shock.clean()
assert _shock.isValid() and len(_shock.Solids())==1, 'Shock eyelets must join the strut'
add_part('Rear shock (100 mm class)', _shock, group='Chassis', alpha=1, color=(0.80, 0.15, 0.12))
def _m6_clevis_bolt(c, half_span):
    """M6 socket-head shoulder bolt + nylon locknut clamping a double-shear clevis of outer half-width `half_span`."""
    shank = _cyl(3.0, 2 * half_span + 9.0, c - _X * (half_span + 5.0), _X)
    head = _cyl(5.0, 5.0, c + _X * half_span, _X).cut(_cyl(2.5, 3.5, c + _X * (half_span + 2.0), _X))
    nut_hex = cq.Workplane(cq.Plane(origin=(c - _X * half_span).toTuple(), normal=(-1, 0, 0))).polygon(6, 11.5).extrude(4.5).val()
    nut_dome = _cyl(4.5, 1.5, c - _X * (half_span + 6.0), _X)
    return shank.fuse(head).fuse(nut_hex).fuse(nut_dome).clean()

_pin_lo = _m6_clevis_bolt(_eb, 10.4)   # M6 bolt + locknut clamping the lower swingarm clevis ears (|x| = 4.4..10.4 mm)
add_part('M6 shock pin (lower)', _pin_lo, group='Chassis', alpha=1, color=(0.78, 0.80, 0.84))
# No separate tower: the shock top pin is carried by the cage's overhead double-shear clevis bracket (|x| = 4.4..12.0 mm).
_pin = _m6_clevis_bolt(_et, 12.0)      # M6 bolt + locknut clamping the upper cage clevis ears (|x| = 4.4..12.0 mm)
add_part('M6 shock pin', _pin, group='Chassis', alpha=1, color=(0.78, 0.80, 0.84))
# Magnetic power connection for the lights, built into the cage (front and rear, on the centre line, above all the componentry).
from parts.light_contacts import make_connector, roof_underside, LIGHT_FRONT_Y, LIGHT_REAR_Y
from parts.cage import make_cage, belt_profile
_belt = belt_profile([_traced['Front shell'], _traced['Rear shell']], LIGHT_FRONT_Y, max(LIGHT_REAR_Y,20.0))
print('cage belt line vertices (y, z):', [(y, round(z, 1)) for y, z in _belt])
if STEALTH:
    # The stealth belt line rises straight from the cowl (y -208, z 134) to the tail apex (y 116, z 171): the cage side rails follow it, the same
    # BELT_CLEAR below the sill, instead of staying on the old flat belt (which left them up to 40 mm under the sill at the tail).
    _bz = lambda y: round(134.0 + (y + 208.0) * (171.0 - 134.0) / (116.0 + 208.0), 1)
    _belt = list(_belt) + [(float(y), _bz(y)) for y in (-150, -50, 38, 60)]
    print('cage belt line (stealth):', _belt)
# Frame nodes of the shell (A pillar, B pillar, roof, rear quarter) in the final frame, from the same trace the canopy is built from.
from parts.reference_trace import ROOF, WINDOW_BELT, traced, belt_station
from parts.shell_geometry import point as _pt
def _final(p):
    c = cq.Solid.makeSphere(0.05, cq.Vector(*p)).transformGeometry(_pitch).Center()
    return (round(c.x, 1), round(c.y, 1), round(c.z, 1))   # rounded: coincident float noise makes the tube booleans invalid
_nodes = {k: _final(_pt(traced(p, 1))) for k, p in {'belt0': WINDOW_BELT[0], 'roof0': ROOF[0], 'roof1': ROOF[1], 'roof2': ROOF[2], 'roofN': ROOF[-1],
                                                    'belt930': belt_station(930)}.items()}
# The visor lies in the windscreen glass plane, whose underside runs through the front roof header (node roof0). The header node drops by this much so the
# header tube clears the visor underside by 0.3 mm; the A pillars, the strut and the roof side rails that end on the node re-angle with it.
ROOF0_DROP = 3.3
_nodes['roof0'] = (_nodes['roof0'][0], _nodes['roof0'][1], round(_nodes['roof0'][2] - ROOF0_DROP, 1))
# Underside of the rear shell hump above the rear tire at the support stations (6 mm clear of the skin)
from parts.cage import REAR_SUP_X
_sup = []
from parts.tail import light_face,TUBE_END_SETBACK
from parts.cage import CAGE_BEAM
_tail_top,_ = light_face()
_tail_node=(REAR_SUP_X,round(_tail_top[1]-TUBE_END_SETBACK,1),round(_tail_top[2]-WALL_PRINT-CAGE_BEAM/2-GLUE_GAP,1))
print('cage nodes', {k: tuple(round(v, 1) for v in p) for k, p in _nodes.items()}, 'supports', _sup)
if STEALTH:
    import parts.cage as _cage_mod
    _cage_mod.BOSS_TARGETS[:] = [_shells[k] for k in ('Front shell', 'Canopy') if k in _shells]   # boss stubs run to the stealth skin
_cabin = None
if STEALTH and OPEN_COCKPIT:
    # exposed cabin frame on the stealth canopy outline (cowl post, roof front edge, roof peak, B-pillar foot) and the cabin opening's rim
    import parts.stealth_shell as _ssh      # (_ss is reused further up for another module)
    _cabin = dict(cowl=_ssh.COWL_POST, roof_front=_ssh.ROOF_FRONT, roof_peak=_ssh.ROOF_PEAK, b_foot=_ssh.B_FOOT, ring=_ssh.cabin_ring(),
                  rim_z=lambda x, y, _s=_shells['Front shell']: max(q.BoundingBox().zmax for q in _s.intersect(cq.Solid.makeBox(0.6, 0.6, 120.0, cq.Vector(x - 0.3, y - 0.3, 90.0))).Solids()))
_exo = None
if DESERT:
    # Desert shell: the panels are built first (untrimmed) so the cage's screw bosses can reach their skin, then the cage with the exo frame in its
    # node graph, then (further down) the panels are trimmed against the finished cage, deck and skid and get their screw holes at the bosses.
    from parts import desert_shell as _ds, exo_cage_front as _ex
    import parts.cage as _cage_mod
    _d_tub, _d_grille, _d_led = _ds.build_nose_tub_and_inserts()
    _d_hood = _ds.build_armored_hood(y_end=_ds.COWL_Y_END if _ds.COWL_MODE == 'fold' else None)   # 'fold': split further down (NOTES 44)
    _d_pod = _d_pods_early      # built above, with the deck's lower brackets
    print('desert shell louvers (right pod):', _ds.LOUVER_INFO)
    _cage_mod.BOSS_TARGETS[:] = [_d_pod[1], _d_pod[-1], _d_tub]
    _onl = lambda a, b, y: tuple(round(a[i] + (y - a[1]) / (b[1] - a[1]) * (b[i] - a[i]), 1) for i in range(3))
    _L0 = (_cage_mod.CAGE_RAIL_X, _cage_mod.RAIL_START_Y, _cage_mod.CAGE_Z)
    _exo_bosses = [((_cage_mod.CAGE_RAIL_X, _ds.POD_BOSS_FRONT[0], _ds.POD_BOSS_FRONT[1]), 1, 3.0, _ds.POD_BOSS_FRONT_D),   # side pod: front ear on the front upright (slim: the ear's front wall is 3 mm ahead)
                   (_onl(_L0, _ex.V_MID, _ds.TUB_BOSS_Y[0]), -1, 3.0, None),                                       # nose tub: rear screw on the lower side rail
                   (_onl(_ex.SIDE_KNEE, _ex.V_MID, _ds.TUB_BOSS_Y[1]), -1, 3.0, None)]                             # nose tub: front screw on the swept side bar
    # hood tabs: a block under each hood screw, from the upper exo rail inward, its top FIT_CUT under the hood's underside
    _tabs, _tab_holes = [], []
    for _sx in (1, -1):
        for _hx, _hy in _ds.HOOD_SCREWS:
            _zt = round(min(_ds.underside_z(_d_hood, _hx + _dx, _hy + _dy) for _dx, _dy in ((-4.5, -4.5), (4.5, -4.5), (-4.5, 4.5), (4.5, 4.5))) - _ds.FIT_CUT, 2)   # lowest underside at the tab's corners (the screw hole itself is open); _rx = _ex.rail_point(LIGHT_FRONT_Y, _hy)[0]
            _rx = _ex.rail_point(LIGHT_FRONT_Y, _hy)[0]
            _x0, _x1 = sorted((_sx * (_hx - _ds.HOOD_TAB_W / 2), _sx * _rx))
            _tabs.append(cq.Solid.makeBox(_x1 - _x0, _ds.HOOD_TAB_W, _ds.HOOD_TAB_T, cq.Vector(_x0, _hy - _ds.HOOD_TAB_W / 2, _zt - _ds.HOOD_TAB_T)))
            _tab_holes.append(cq.Solid.makeCylinder(_ds.SCREW_PILOT / 2, _ds.HOOD_TAB_T + 2.0, cq.Vector(_sx * _hx, _hy, _zt - _ds.HOOD_TAB_T - 1.0), cq.Vector(0, 0, 1)))
            print('hood tab side', _sx, 'y', _hy, 'top z', _zt, 'x', round(_x0, 1), round(_x1, 1))
    _exo = dict(hoop_z=_ex.HOOP_Z, members=lambda _fy, _zs: _ex.members(_fy, _zs, _skid_nodes), bosses=_exo_bosses, pilot=_ds.SCREW_PILOT, max_reach=_ds.BOSS_MAX_REACH,
                tail_boss_y=_ds.POD_BOSS_TAIL_Y, diag_boss_y=_ds.POD_BOSS_DIAG_Y, solids=_tabs, holes=_tab_holes)
_cage, _, _ = make_cage(LIGHT_FRONT_Y, LIGHT_REAR_Y, _belt, _nodes, _sup, tie=(_et.y, _et.z),tail=_tail_node,tail_screws=_tail_scr['cage'],cabin=_cabin,exo=_exo)
from parts.cage import FRONT_LEG_X, REAR_LEG_X, FRONT_FOOT_Z, CAGE_Z, CAGE_BEAM
assert _cage.isValid() and len(_cage.Solids()) == 1, 'cage: expected one valid solid'
# Version 2 additions to the cage: a spine on the centre line between two cross tubes (the heave shock's rear eye hangs under it), and on each side a
# stub from the front foot post back to the lug and clevis of the upper link's inner hinge.
from parts.tubes import square_bar as _sqb
from parts.cage import CAGE_RAIL_X as _CRX, TRUSS_Y as _TRY, RAIL_END_Y as _REY, RAIL_START_Y as _RSY
_cs = CAGE_BEAM * 0.88
_add = [_sqb((-_CRX, _TRY, _sa2.SPINE_Z), (_CRX, _TRY, _sa2.SPINE_Z), side=_cs), _sqb((-_CRX, _REY, _sa2.SPINE_Z), (_CRX, _REY, _sa2.SPINE_Z), side=_cs),
        _sqb((0.0, _TRY, _sa2.SPINE_Z), (0.0, _REY, _sa2.SPINE_Z), side=_cs),
        *_sg.cage_pieces(_sa2.SPINE_Z - 2.5)]
# stop struts: from the rear lower corner of each pad block back and down to the rail's rear foot node. FEM without them: 63 MPa (safety 0.32),
# the beam's push on the carrier (262 N, 31 mm ahead of and 29 mm under the front cross tube) twisted the cross tube and bent the spine (11.3 N m)
_add += [_sqb((_sx * 36.0, -28.5, 94.5), (_sx * _CRX, _REY, CAGE_Z), side=11.0) for _sx in (1, -1)]      # 11 mm again, now from x 36: 2 mm clear of the motor plate's corner      # rooted in the lump behind the dowel window; 9.5 mm from x 34 (11 mm from x 31 clipped the motor plate's corner at x -31.5: 13 mm3)      # from x 34: the first route (x 24.4) cut the motor plate (63.8 mm3)      # carrier clevis with dowel slots and the two pad blocks under the front cross tube, shock clevis under the spine
_hiw = _hi + _off
for _sx in (1, -1):
    _add += [_sqb((_sx * _CRX, _RSY, _hiw.z - 12.9), (_sx * _CRX, _hiw.y + 6.0, _hiw.z - 12.9), side=_cs),
             _up.rail_lug(_sx).translate(_off), _up.clevis_u(_hi, _sx).translate(_off)]
for _sx in (1, -1):
    _sa2.guard('Pad block %d' % _sx, _sg.pad_block(_sx, _sa2.SPINE_Z - 2.5))
_v0 = _cage.Volume()
for _a in _add:
    _cage = _cage.fuse(_a, tol=1e-3)
_cage = _cage.clean()
assert _cage.isValid() and len(_cage.Solids()) == 1, 'version 2 cage additions broke the cage (%d solids)' % len(_cage.Solids())
print('cage: spine, cross tubes, shock clevis and upper link lugs added, mm3', round(_cage.Volume() - _v0, 1))
for _sx in (1, -1):
    for _hn, _hh in (('inner', _hi), ('outer', _ho)):
        add_part('Uplink pin %s %d' % (_hn, _sx), _up.pin(_hh, _sx).translate(_off), group='Chassis/Front suspension', alpha=1, color=(0.78, 0.80, 0.84))
# (old passive lean bridge, balance beam, front shocks and uplink lugs removed: front end rebuilt in parts/wishbone.py)
_d_cover, _d_flange, _cov_full = {}, None, {}
if DESERT:
    from parts.cage import _interp as _cg_interp, _half as _cg_half, CAGE_BEAM as _CGB, CAGE_RAIL_X as _CRX2, RAIL_END_Y as _REY_   # (the dash panel uses them too)
    if _ds.POD_TOP_SPLIT:
        # NOTES 62 (owner markup 2026-11-26, "no steps between mating surfaces"): the side covers are the pods' own top skin, cut out of the
        # closed hull on the mitre through its fold (parts/desert_shell.py _split_pod_top): front cover = the wing's top, rear cover = the main
        # block's top. The rear cover carries the flange (y 48 .. 128): a flat WALL strip from the pod's inboard top edge (one hull edge) in to the
        # cage, its top at that edge's height, so it meets the cover's top on the edge (a 4.6 deg crease, no step).
        (_ea, _eb) = _ds.POD_TOP['edge_in']
        _edge = lambda _y: (_ea[0] + (_eb[0] - _ea[0]) * (_y - _ea[1]) / (_eb[1] - _ea[1]), _ea[2] + (_eb[2] - _ea[2]) * (_y - _ea[1]) / (_eb[1] - _ea[1]))
        _fsecs = []
        for _y0, _y1 in zip(_ds.FLANGE_Y, _ds.FLANGE_Y[1:]):
            (_xe0, _ze0), (_xe1, _ze1) = _edge(_y0), _edge(_y1)
            _reach = [_ds.band_xmax(_cage, _yy, 20.0, max(_xe0, _xe1), min(_ze0, _ze1) - _ds.WALL - _ds.FIT_CUT, max(_ze0, _ze1) + _ds.FIT_CUT)
                      for _yy in [_y0 + _k * _ds.FLANGE_SAMPLE for _k in range(int((_y1 - _y0) / _ds.FLANGE_SAMPLE) + 1)]]
            _reach = [_r for _r in _reach if _r is not None]
            _xi = (max(_reach) if _reach else _ds.FLANGE_DEFAULT_X) + _ds.FIT_CUT
            if _xi > min(_xe0, _xe1) - 1.0:
                print('pod flange span y %.0f..%.0f: no gap (cage reach x %.2f, pod edge x %.1f / %.1f)' % (_y0, _y1, _xi, _xe0, _xe1))
                continue
            _sec = lambda _xe, _ze: [(_xe + 0.6, _ze), (_xi, _ze), (_xi, _ze - _ds.WALL), (_xe + 0.6, _ze - _ds.WALL)]
            _p = _ds.strip_solid([(_y0, _sec(_xe0, _ze0)), (_y1, _sec(_xe1, _ze1))])
            _d_flange = _p if _d_flange is None else _d_flange.fuse(_p)
            print('pod flange span y %.0f..%.0f: inner edge x %.2f (cage reach + fit), on the rear cover' % (_y0, _y1, _xi))
        _rear = _ds.POD_TOP['rear']
        if _d_flange is not None:
            _g = _ds.POD_TOP_GAP          # the flange keeps the pod's gap too (it reached 0.04 mm3 into the pod at the tail, y 128)
            _d_flange = _d_flange.clean()
            import itertools as _it      # 124 directions (axes and diagonals left 0.01 / 0.16 mm at the tail tip, y 128); two runs: fit_cut would keep one
            _fb = _d_flange.BoundingBox()
            _pl = _d_pods_early[1].intersect(cq.Solid.makeBox(_fb.xlen + 4.0, _fb.ylen + 4.0, _fb.zlen + 4.0, cq.Vector(_fb.xmin - 2.0, _fb.ymin - 2.0, _fb.zmin - 2.0)))
            for _dv in _it.product((-2, -1, 0, 1, 2), repeat=3):
                _nv = sum(c * c for c in _dv) ** 0.5 or 1.0
                _d_flange = _d_flange.cut(_pl.translate(cq.Vector(*(_g * c / _nv for c in _dv))))
            _rear = _rear.fuse(_d_flange).clean()
            assert _rear.isValid() and len(_rear.Solids()) == 1, 'flange did not fuse to the rear cover'
        _pieces = {'front': _ds.POD_TOP['front'], 'rear': _rear}
        # cover screws: along the local top normal, countersunk, into a boss inside the pod (clipped to the envelope under the covers, so its
        # top stays POD_TOP_GAP under the cover's underside); the boss reaches outward into the flank wall
        _cov_boss = {1: [], -1: []}
        _r0 = _ds.COVER_BOSS[0] / 2.0
        for _ys in _ds.POD_TOP_SCREWS_Y:
            _k = 'front' if _ys < -55.0 else 'rear'
            _xf = _ds.band_xmax(_ds.POD_TOP[_k], _ys, 40.0, 100.0, 100.0, 200.0)
            _xb = _xf - _ds.WALL - _ds.POD_TOP_GAP - _r0 - 0.5
            _pt, _n = _ds.top_point(_pieces[_k], _xb, _ys)
            _P, _N = cq.Vector(*_pt), cq.Vector(*_n)
            _ray = cq.Solid.makeCylinder(3.0, 80.0, _P + _N * 4.0, _N)
            if _ray.intersect(_cage).Volume() > 1e-3:
                print('side cover screw y %.0f: no tool path along the normal past the cage, left out' % _ys)
                continue
            _bh = _ds.COVER_BOSS[1] + _ds.WALL + 3.0
            _b0 = _P - _N * (_ds.COVER_BOSS[1] + _ds.WALL)
            _bs = cq.Solid.makeCylinder(_r0, _bh, _b0, _N).fuse(
                  cq.Solid.makeBox(_ds.COVER_BOSS_REACH, _ds.COVER_BOSS[0], _bh, cq.Vector(_xb, _ys - _r0, _b0.z)))
            _bs = _bs.intersect(_ds.POD_TOP['under']).cut(cq.Solid.makeCylinder(_ds.SCREW_PILOT / 2.0, _bh + 1.0, _b0 - _N * 0.5, _N))
            _pieces[_k] = _ds.countersink(_pieces[_k], _pt, _n)
            _cov_boss[1].append(_bs); _cov_boss[-1].append(_bs.mirror('YZ'))
            print('side cover screw y %.0f (%s): (%.1f, %.1f, %.1f), normal (%.3f, %.3f, %.3f), countersunk %.1f, boss %.1f mm3'
                  % (_ys, _k, _pt[0], _pt[1], _pt[2], _n[0], _n[1], _n[2], _ds.COVER_CSK_D, _bs.Volume()))
        for _sx in (1, -1):
            for _k, _c in _pieces.items():
                _c = _c if _sx > 0 else _c.mirror('YZ')
                _v0 = _c.Volume()
                _c = _ds.fit_cut(_c, [_cage], [(_ds.FIT_CUT, 0, 0), (-_ds.FIT_CUT, 0, 0), (0, _ds.FIT_CUT, 0), (0, -_ds.FIT_CUT, 0), (0, 0, _ds.FIT_CUT), (0, 0, -_ds.FIT_CUT)])
                # (NOTES 62 follow-up) along the upper rail the cover stops at the rail's outer face + FIT_CUT: inboard of it the fit around the
                # rail left a thin strip beside the channel (slivers at (49.7, -113.2, 139.2) and (49.8, 31.4, 160.5))
                _xr = _CRX2 + _cg_half(_CGB) + _ds.FIT_CUT; _yr = _REY_ + 8.5       # (to y 46.5: the rail's end node and the upright's boss reach past x 49.8 up to y 45.5, MEASURED)
                _c = _ds.fit_cut(_c, [cq.Solid.makeBox(_xr, _yr + 160.0, 120.0, cq.Vector(0.0 if _sx > 0 else -_xr, -160.0, 100.0))], [])
                # beside the rail's end node (y 36 .. 42) the fit around its hull left a face at x 49.9, 0.1 mm off the trim plane: the cover
                # stops at x 50.3 there (a 0.5 mm step in its inboard edge, under the node)
                _c = _ds.fit_cut(_c, [cq.Solid.makeBox(_xr + 0.5, 6.0, 120.0, cq.Vector(0.0 if _sx > 0 else -_xr - 0.5, _REY_ - 2.0, 100.0))], [])
                # install path: cover up COVER_LIFT, then outward COVER_OUT (pods stay on); cage members met on the lift get a through-notch
                _cb = _c.BoundingBox(); _met = None
                for _j in range(1, 2 * _ds.COVER_LIFT + 1):
                    _h = _c.translate(cq.Vector(0, 0, 0.5 * _j)).intersect(_cage)
                    if _h.Volume() > 0.01:
                        _met = _h if _met is None else _met.fuse(_h)
                if _met is not None:
                    from scipy.spatial import ConvexHull as _CH
                    for _q in _met.Solids():
                        _vs = [(_v.X, _v.Y) for _v in _q.Vertices()]
                        _poly = [_vs[_i] for _i in _CH(_vs).vertices]
                        _cut = cq.Workplane('XY').workplane(offset=_cb.zmin - 1.0).polyline(_poly).close().offset2D(_ds.FIT_CUT).extrude(_cb.zlen + 2.0).val()
                        _c = _ds.fit_cut(_c, [_cut], [])
                _c = _c.clean()
                assert _c.isValid() and len(_c.Solids()) == 1, 'side cover %s %d: %d solids after the fit' % (_k, _sx, len(_c.Solids()))
                _d_cover[(_k, _sx)] = _c
                print('side cover %s %d: %.1f mm3 (fit to the cage %.1f, lift notches %s)' % (_k, _sx, _c.Volume(), _v0 - _c.Volume(), 'none' if _met is None else len(_met.Solids())))
        _d_flange = None            # it is on the rear cover, not fused to the pods
    else:
        # Side covers and pod flanges (parts/desert_shell.py, NOTES decision 25). Sections come from the built cage and the (untrimmed) pods.
        from parts.cage import _interp as _cg_interp, _half as _cg_half, CAGE_BEAM as _CGB, CAGE_RAIL_X as _CRX2
        _pr = _d_pods_early[1]
        _secs = []
        for _y in _ds.COVER_Y:
            if _y <= _ds.COVER_NODE_Y:
                _zr = _cg_interp(_cage_mod.LAST_UPPER, _y) + _cg_half(_CGB)
            else:
                _zr = _ds.top_slab(_cage, _y, _CRX2 - 4.0, _CRX2 + 4.0, 135.0, 172.0)[2]
            _pt = _ds.top_slab(_pr, _y, 44.0, 100.0, **(dict(dz=0.05, w=0.02) if _ds.COVER_FLUSH else {}))   # flush: the top chine itself (a 1 mm slab put the
            # cover's outer corner 1 mm down the flank's 51 deg slope, a 2.1 mm ledge, NOTES 46)
            _zi, _zo = _zr + _ds.FIT_CUT + _ds.WALL, _pt[2] + (0.0 if _ds.COVER_FLUSH else _ds.FIT_CUT + _ds.WALL)   # flush: outer top on the pod's top edge (NOTES 46)
            # option (c), NOTES 31: the cover starts FIT_CUT off the rail's outer face (it laid over the rail and could not be lifted past the struts)
            _secs.append((_y, [(_CRX2 + _cg_half(_CGB) + _ds.FIT_CUT, _zi), (_pt[1], _zo)]))
            print('side cover station y %.0f: rail top %.2f, pod top x %.1f z %.2f' % (_y, _zr, _pt[1], _pt[2]))
        _cov = _ds.strip_solid(_secs, _ds.WALL)
        # Pod flange (thickness 54 fixes, orchestrator option C): one straight inner edge per 8 mm span at the cage's maximum reach in that span
        # (sampled every FLANGE_SAMPLE) + FIT_CUT, so the fit cut does not shave the 45 deg plate; the top corner FLANGE_DROP under the pod top.
        # Markup 2, areas 3 / 4 (owner, November 25: "molded/merged seamlessly to the existing shell"; "leaving the 45 deg plate is NOT allowed"):
        # the flange is now a flat 3 mm top in the plane of the pod's top land, fused into the pod. The land's height is read in a 0.02 mm slice
        # (the 0.6 mm slice read 0.05 mm high on the land's slope along y and the plate poked through: thickness 54), so FLANGE_DROP is 0.
        _st, _land_out = [], {}
        for _y in _ds.FLANGE_Y:
            _pt = _ds.top_slab(_pr, _y, 44.0, 100.0, dz=0.05, w=0.02)
            _st.append((_y, _pt[0], _pt[2])); _land_out[_y] = _pt[1]
        def _fsec(_xp, _zt, _xi, _xo):
            # flat top on the land's plane from FLANGE_SEAT outboard of the land's inner edge in to the cage, WALL thick, square inner end
            _zf = _zt - _ds.FLANGE_DROP
            _xs = min(_xp + _ds.FLANGE_SEAT, _xo - 0.2)        # under the land only, never past its outer edge
            return [(_xs, _zf), (_xi, _zf), (_xi, _zf - _ds.WALL), (_xs, _zf - _ds.WALL)]
        _d_flange, _fruns = None, []
        for (_y0, _xp0, _zt0), (_y1, _xp1, _zt1) in zip(_st, _st[1:]):
            _reach = [_ds.band_xmax(_cage, _yy, 20.0, max(_xp0, _xp1), min(_zt0, _zt1) - _ds.WALL - _ds.FIT_CUT, max(_zt0, _zt1) + _ds.FIT_CUT)
                      for _yy in [_y0 + _k * _ds.FLANGE_SAMPLE for _k in range(int((_y1 - _y0) / _ds.FLANGE_SAMPLE) + 1)]]
            _reach = [_r for _r in _reach if _r is not None]
            _xi = (max(_reach) if _reach else _ds.FLANGE_DEFAULT_X) + _ds.FIT_CUT
            if _xi > min(_xp0, _xp1) - 1.0:
                print('pod flange span y %.0f..%.0f: no gap (cage reach x %.2f, pod x %.1f / %.1f)' % (_y0, _y1, _xi, _xp0, _xp1))
                continue
            _p = _ds.strip_solid([(_y0, _fsec(_xp0, _zt0, _xi, _land_out[_y0])), (_y1, _fsec(_xp1, _zt1, _xi, _land_out[_y1]))])
            _v = _p.Volume(); _pc = _p.cut(_cage)
            print('pod flange span y %.0f..%.0f: inner edge x %.2f (cage reach + fit), cage overlap before the fit cut %.3f mm3' % (_y0, _y1, _xi, _v - _pc.Volume()))
            _d_flange = _p if _d_flange is None else _d_flange.fuse(_p)
            _fruns.append([(_y0,), (_y1,)])
        _d_flange = _d_flange.clean()
        print('pod flange runs (y):', [(r[0][0], r[-1][0]) for r in _fruns])
        for _sx in (1, -1):
            _c = _cov if _sx > 0 else _cov.mirror('YZ')
            _c = _ds.fit_cut(_c, [_cage] + ([] if _ds.COVER_FLUSH else [_d_pods_early[_sx]]), [(_ds.FIT_CUT, 0, 0), (-_ds.FIT_CUT, 0, 0), (0, _ds.FIT_CUT, 0), (0, -_ds.FIT_CUT, 0), (0, 0, _ds.FIT_CUT), (0, 0, -_ds.FIT_CUT)])
            # install path (orchestrator, November 25): pods off, cover up COVER_LIFT, then outward COVER_OUT. Cage members the cover meets on the lift
            # get a through-notch: their outline in plan (2D convex hull of each met piece's vertices, + FIT_CUT) cut straight through the cover,
            # so there is no floor and the wall beside the notch stays 3.0 mm. Nothing met: no notch.
            _cb = _c.BoundingBox()
            _met = None
            for _k in range(1, 2 * _ds.COVER_LIFT + 1):
                _h = _c.translate(cq.Vector(0, 0, 0.5 * _k)).intersect(_cage)
                if _h.Volume() > 0.01:
                    _met = _h if _met is None else _met.fuse(_h)
            if _met is None:
                print('side cover %d: free lift %d mm, no cage member met, no notch' % (_sx, _ds.COVER_LIFT))
            else:
                for _q in _met.Solids():
                    _vs = [(_v.X, _v.Y) for _v in _q.Vertices()]
                    from scipy.spatial import ConvexHull as _CH
                    _hl = _CH(_vs); _poly = [_vs[_i] for _i in _hl.vertices]
                    _cut = cq.Workplane('XY').workplane(offset=_cb.zmin - 1.0).polyline(_poly).close().offset2D(_ds.FIT_CUT).extrude(_cb.zlen + 2.0).val()
                    _v0 = _c.Volume(); _c = _ds.fit_cut(_c, [_cut], [])
                    _qb = _q.BoundingBox()
                    print('side cover %d: met member at x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f: through-notch %.1f mm3' % (_sx, _qb.xmin, _qb.xmax, _qb.ymin, _qb.ymax, _qb.zmin, _qb.zmax, _v0 - _c.Volume()))
            _d_cover[_sx] = _c
            _cov_full[_sx] = _c                  # without screw holes: the pod's rebate is cut with it (NOTES 46)
        # cover screws: from above, through the cover into a boss on the inside of the pod's top edge (the rail top is no longer under the cover)
        _cov_boss = {1: [], -1: []}
        for _ys in _ds.COVER_POD_SCREWS_Y:
            _pt = _ds.top_slab(_pr, _ys, 44.0, 100.0)
            _bxc, _bzt = _pt[0] - _ds.COVER_BOSS[2], _pt[2]
            for _sx in (1, -1):
                # built 3 mm taller than the land and clipped to the pod's outer envelope: its top is then exactly the land's plane (thickness 58 / 59:
                # a top 0.15 mm under the land poked through where the land slopes, 1.0 mm under it left the land's edge as a 1.0 mm lip)
                _zb0, _bh = _bzt - _ds.COVER_BOSS[1], _ds.COVER_BOSS[1] + 3.0
                _bs = cq.Solid.makeCylinder(_ds.COVER_BOSS[0] / 2, _bh, cq.Vector(_sx * _bxc, _ys, _zb0), cq.Vector(0, 0, 1)).fuse(
                      cq.Solid.makeBox(_ds.COVER_BOSS_REACH, _ds.COVER_BOSS[0], _bh, cq.Vector(_sx * _bxc - (0 if _sx > 0 else _ds.COVER_BOSS_REACH), _ys - _ds.COVER_BOSS[0] / 2, _zb0)))
                _env = _ds.POD_ENV[0] if _sx > 0 else _ds.POD_ENV[0].mirror('YZ')
                _bs = _bs.intersect(_env).cut(cq.Solid.makeCylinder(_ds.SCREW_PILOT / 2, _bh + 1.0, cq.Vector(_sx * _bxc, _ys, _zb0 - 0.5), cq.Vector(0, 0, 1)))
                _cov_boss[_sx].append(_bs)
                _d_cover[_sx] = _ds.screw_hole(_d_cover[_sx], (_sx * _bxc, _ys, 120.0), (0, 0, 1), 80.0)
                _ray = cq.Solid.makeCylinder(3.0, 80.0, cq.Vector(_sx * _bxc, _ys, _bzt + 4.0), cq.Vector(0, 0, 1))
                assert _ray.intersect(_cage).Volume() < 1e-3, 'no tool path above the cover screw at y %.0f' % _ys
            print('side cover screw y %.0f: x %.1f, pod top z %.1f (tool path from above clear)' % (_ys, _bxc, _bzt))
        print('side covers mm3 %.1f each, pod flange mm3 %.1f' % (_d_cover[1].Volume(), _d_flange.Volume()))
    # Cowl panel (markup 2, area 1): the hood's top planes carried back over the cowl bar, a separate part that lifts straight up.
    # Bosses under the bulge onto the bar's top, M3 from above into a pilot in the bar (cut into the cage here, before it is registered).
    _d_cowl = _ds.build_cowl()          # also sets COWL_ENV (the bosses are clipped to it)
    if _ds.COWL_MODE == 'fold':         # the cowl is the hood's rear piece, split on the hood's fold at y -252 (NOTES 44)
        _d_hood, _d_cowl = _ds.fold_split(_d_hood)
        print('hood split on its fold at y %.1f: front %.1f mm3, rear (cowl panel) %.1f mm3' % (_ds.COWL_FOLD_Y, _d_hood.Volume(), _d_cowl.Volume()))
    _d_cowl = _ds.fit_cut(_d_cowl, [_cage], [(_ds.FIT_CUT, 0, 0), (-_ds.FIT_CUT, 0, 0), (0, _ds.FIT_CUT, 0), (0, -_ds.FIT_CUT, 0), (0, 0, _ds.FIT_CUT), (0, 0, -_ds.FIT_CUT)])
    for _cx0, _cy in _ds.COWL_SCREWS:
        for _sx in (1, -1):
            _cx = _sx * _cx0
            _bt = _ds.top_slab(_cage, _cy, _cx - 0.3, _cx + 0.3, 120.0, 150.0)[2]       # bar top under the screw
            _zb = _bt + _ds.FIT_CUT
            _bs = cq.Solid.makeCylinder(_ds.COWL_BOSS_D / 2, 20.0, cq.Vector(_cx, _cy, _zb), cq.Vector(0, 0, 1)).intersect(_ds.COWL_ENV[0])
            _d_cowl = _d_cowl.fuse(_bs).clean()
            _d_cowl = _ds.screw_hole(_d_cowl, (_cx, _cy, 100.0), (0, 0, 1), 80.0)
            _cage = _cage.cut(cq.Solid.makeCylinder(_ds.SCREW_PILOT / 2, _ds.COWL_PILOT_DEPTH + 0.5, cq.Vector(_cx, _cy, _bt - _ds.COWL_PILOT_DEPTH), cq.Vector(0, 0, 1))).clean()
            print('cowl screw x %.1f y %.1f: bar top %.2f, boss from z %.2f' % (_cx, _cy, _bt, _zb))
    _v0 = _d_cowl.Volume()
    _d_cowl = _ds.fit_cut(_d_cowl, [_cage], [(_ds.FIT_CUT, 0, 0), (-_ds.FIT_CUT, 0, 0), (0, _ds.FIT_CUT, 0), (0, -_ds.FIT_CUT, 0), (0, 0, _ds.FIT_CUT), (0, 0, -_ds.FIT_CUT)])
    print('cowl bosses trimmed to the bar (the bar top rises toward the V root), mm3 %.1f' % (_v0 - _d_cowl.Volume()))
    assert _d_cowl.isValid() and len(_d_cowl.Solids()) == 1, 'cowl: %d solids' % len(_d_cowl.Solids())
    assert _cage.isValid() and len(_cage.Solids()) == 1, 'cage after the cowl pilots: %d solids' % len(_cage.Solids())
    _hit = [k for k in range(1, 61) if _d_cowl.translate(cq.Vector(0, 0, 0.5 * k)).intersect(_cage).Volume() > 0.01]
    print('cowl panel mm3 %.1f, bbox %s, lift 30 mm: %s' % (_d_cowl.Volume(), tuple(round(v, 1) for v in (_d_cowl.BoundingBox().ymin, _d_cowl.BoundingBox().ymax, _d_cowl.BoundingBox().zmin, _d_cowl.BoundingBox().zmax)),
                                                          'clear' if not _hit else 'meets the cage at %.1f mm' % (0.5 * _hit[0])))
    # Dash panel (markup 2, area 2): removable, the side covers' inner-edge section (rail top + FIT_CUT, WALL thick) carried across between the
    # covers (SHELL_GAP to each), so its top is flush with the covers' inner edge. The windscreen V legs above it get a through-notch (their plan
    # outline + FIT_CUT): it lifts straight up between them, and from above the legs hide the notch. M3 from above into the upper rails' tops.
    _xin = _CRX2 + _cg_half(_CGB) + _ds.FIT_CUT - _ds.SHELL_GAP
    _dsec = []
    for _y in _ds.DASH_Y:
        _zt = _cg_interp(_cage_mod.LAST_UPPER, _y) + _cg_half(_CGB) + _ds.FIT_CUT + _ds.WALL
        _dsec.append((_y, [(-_xin, _zt), (_xin, _zt)]))
    _d_dash = _ds.strip_solid(_dsec, _ds.WALL)
    _db = _d_dash.BoundingBox()
    # lift space: from 0.05 over the dash's top surface up DASH_LIFT, following its slope (the rails under it are not in it)
    _lift = _ds.strip_solid([(_y, [(_x, _z + _ds.DASH_LIFT) for _x, _z in _p]) for _y, _p in _dsec], _ds.DASH_LIFT - 0.05)
    _over = _cage.intersect(_lift)
    from scipy.spatial import ConvexHull as _CH
    for _q in _over.Solids():
        _qb0 = _q.BoundingBox(); _s = 1.0 if _qb0.center.x > 0 else -1.0
        # the notch runs out to the dash's side edge: no sliver is left outboard of the leg
        _vs = [(_v.X, _v.Y) for _v in _q.Vertices()] + [(_s * 60.0, _qb0.ymin), (_s * 60.0, _qb0.ymax)]
        _hl = _CH(_vs); _poly = [_vs[_i] for _i in _hl.vertices]
        _cut = cq.Workplane('XY').workplane(offset=_db.zmin - 1.0).polyline(_poly).close().offset2D(_ds.FIT_CUT).extrude(_db.zlen + 2.0).val()
        _v0 = _d_dash.Volume(); _d_dash = _d_dash.cut(_cut).clean()
        _qb = _q.BoundingBox()
        print('dash: member over it at x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f: through-notch %.1f mm3' % (_qb.xmin, _qb.xmax, _qb.ymin, _qb.ymax, _qb.zmin, _qb.zmax, _v0 - _d_dash.Volume()))
    _sols = sorted(_d_dash.Solids(), key=lambda s: -s.Volume())
    if len(_sols) > 1:
        print('dash: notches left %d loose slivers outside the legs, mm3 %s (dropped: the rail under them closes that strip from above)' % (len(_sols) - 1, [round(s.Volume(), 1) for s in _sols[1:]]))
    _d_dash = _sols[0]
    _d_dash = _ds.fit_cut(_d_dash, [_cage], [(_ds.FIT_CUT, 0, 0), (-_ds.FIT_CUT, 0, 0), (0, _ds.FIT_CUT, 0), (0, -_ds.FIT_CUT, 0), (0, 0, _ds.FIT_CUT), (0, 0, -_ds.FIT_CUT)])
    for _dy in _ds.DASH_SCREWS_Y:
        for _sx in (1, -1):
            _rt = _ds.top_slab(_cage, _dy, _sx * _CRX2 - 0.3, _sx * _CRX2 + 0.3, 120.0, 160.0)[2]
            _d_dash = _ds.screw_hole(_d_dash, (_sx * _CRX2, _dy, 100.0), (0, 0, 1), 80.0)
            _cage = _cage.cut(cq.Solid.makeCylinder(_ds.SCREW_PILOT / 2, _ds.DASH_PILOT_DEPTH + 0.5, cq.Vector(_sx * _CRX2, _dy, _rt - _ds.DASH_PILOT_DEPTH), cq.Vector(0, 0, 1))).clean()
            _ray = cq.Solid.makeCylinder(3.0, 80.0, cq.Vector(_sx * _CRX2, _dy, _rt + 4.0), cq.Vector(0, 0, 1))
            assert _ray.intersect(_cage).Volume() < 1e-3, 'no tool path above the dash screw at y %.0f' % _dy
            print('dash screw x %.1f y %.1f: rail top %.2f (tool path from above clear)' % (_sx * _CRX2, _dy, _rt))
    assert _d_dash.isValid() and len(_d_dash.Solids()) == 1, 'dash: %d solids' % len(_d_dash.Solids())
    assert _cage.isValid() and len(_cage.Solids()) == 1, 'cage after the dash pilots: %d solids' % len(_cage.Solids())
    _hit = [k for k in range(1, 61) if _d_dash.translate(cq.Vector(0, 0, 0.5 * k)).intersect(_cage).Volume() > 0.01]
    _db = _d_dash.BoundingBox()
    print('dash panel mm3 %.1f, x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f, lift 30 mm: %s' % (_d_dash.Volume(), _db.xmin, _db.xmax, _db.ymin, _db.ymax, _db.zmin, _db.zmax,
                                                                                          'clear' if not _hit else 'meets the cage at %.1f mm' % (0.5 * _hit[0])))
SHOW_CAGE =param(True, 'Show the cage (cage, roof plate, light contact block); off while the front end is redesigned', group='Cage')
if SHOW_CAGE: add_part('Cage', _cage, color=(0.05, 0.05, 0.06), group='Cage')   # matt black so it disappears behind the shell windows
from parts.cage import cabin_nodes as _cabn, CABIN_BEAM as _CBM, _half as _hf
if DESERT:
    # Trim the desert panels against what they sit on (FIT_CUT clear), cut the screw holes at the cage's bosses and tabs, register.
    def _chassis_solid(name):
        for _k, _nd in _ch.objects.items():
            if _k.rsplit('/', 1)[-1].replace('_', ' ') == name and _nd.obj is not None:
                return (_nd.obj.val() if isinstance(_nd.obj, cq.Workplane) else _nd.obj).moved(_nd.loc).moved(cq.Location(_off))
        raise KeyError(name)
    _fc = _ds.FIT_CUT
    _six = [(_fc, 0, 0), (-_fc, 0, 0), (0, _fc, 0), (0, -_fc, 0), (0, 0, _fc), (0, 0, -_fc)]      # the neighbour moved FIT_CUT along each axis: no face is left touching
    _skid, _deck_r, _deck_t = _chassis_solid('w3 skid'), _chassis_solid('w3 deck rear'), _chassis_solid('w3 deck tail')
    _v0 = _d_tub.Volume(); _d_tub = _ds.fit_cut(_d_tub, [_cage, _skid], _six)
    print('nose tub trimmed mm3', round(_v0 - _d_tub.Volume(), 1))
    # Chin above the V bar (skid line-up): the bar trimmed the chin's lower facet at a slant and left a 0.13 mm feather 44 mm long. The chin is
    # cut flat instead, FIT_CUT above the bar's top face, back to where the tub's lower edge (CHIN_BOT -> TUB_MID_BOT) rises through that height.
    _zc = _skid_nodes['C'][2] + _ex.EXO_D * 0 + __import__('parts.cage', fromlist=['_half'])._half(_ex.EXO_D) + _fc
    _cb, _mb = _ds.CHIN_BOT, _ds.TUB_MID_BOT
    _yc = _cb[1] + (_zc - _cb[2]) / (_mb[2] - _cb[2]) * (_mb[1] - _cb[1])
    _v0 = _d_tub.Volume()
    _d_tub = _ds.fit_cut(_d_tub, [cq.Solid.makeBox(120.0, _yc + 320.0, _zc, cq.Vector(-60.0, -320.0, 0.0))], [])
    print('nose tub chin cut flat at z %.2f ahead of y %.2f, mm3 %.1f' % (_zc, _yc, _v0 - _d_tub.Volume()))
    _v0 = _d_hood.Volume(); _d_hood = _ds.fit_cut(_d_hood, [_cage], _six)
    if _ds.COWL_FUSED:      # area 1: the cowl (already fitted to the cage, with its bosses) becomes part of the hood: one solid, the facets run on
        _vh, _vc = _d_hood.Volume(), _d_cowl.Volume()
        _d_hood = _d_hood.fuse(_d_cowl).clean()
        assert _d_hood.isValid() and len(_d_hood.Solids()) == 1, 'hood with cowl: %d solids' % len(_d_hood.Solids())
        print('cowl fused into the hood: hood %.1f + cowl %.1f -> %.1f mm3, %d faces' % (_vh, _vc, _d_hood.Volume(), len(_d_hood.Faces())))
    print('hood trimmed mm3', round(_v0 - _d_hood.Volume(), 1))
    for _sx in (1, -1):      # pod flanges (rear strip) fused before the pods are trimmed (NOTES 62: on the rear cover instead)
        _v0 = _d_pod[_sx].Volume(); _t = _d_pod[_sx]
        if _d_flange is not None:
            _t = _t.fuse(_d_flange if _sx > 0 else _d_flange.mirror('YZ')).clean()
        assert _t.isValid() and len(_t.Solids()) == 1, 'pod flange did not fuse on side %d' % _sx
        for _b in _cov_boss[_sx]:
            _t = _t.fuse(_b)
        _t = _t.clean(); assert _t.isValid() and len(_t.Solids()) == 1, 'cover bosses did not fuse to pod %d' % _sx
        if _ds.COVER_FLUSH:     # rebate: the pod gives way under its cover (cover + SHELL_GAP all round), so the cover lies flush in the pod's top edge
            _g = _ds.SHELL_GAP
            _t0 = _t
            _vr = _t.Volume(); _t = _ds.fit_cut(_t, [_cov_full[_sx]], [(_g, 0, 0), (-_g, 0, 0), (0, _g, 0), (0, -_g, 0), (0, 0, _g), (0, 0, -_g)])
            # seat (attempt 3, NOTES 46): the cover's underside cut the sloped walls into feathers; a WALL thick band under the cover, inside the pod's
            # envelope and within WALL of the old walls, so every cut wall ends on a seat the cover rests on
            _cv = _cov_full[_sx]
            _band = _cv.translate(cq.Vector(0, 0, -_g - _ds.WALL)).fuse(_cv.translate(cq.Vector(0, 0, -_g - _ds.WALL / 2)))
            _near = _t0
            for _d in ((_ds.WALL, 0, 0), (-_ds.WALL, 0, 0), (0, _ds.WALL, 0), (0, -_ds.WALL, 0), (0, 0, _ds.WALL), (0, 0, -_ds.WALL)):
                _near = _near.fuse(_t0.translate(cq.Vector(*_d)))
            _env = _ds.POD_ENV[0] if _sx > 0 else _ds.POD_ENV[0].mirror('YZ')
            _seat = _band.intersect(_env).intersect(_near)
            _seat = _ds.fit_cut(_seat, [_cv], [(_g, 0, 0), (-_g, 0, 0), (0, _g, 0), (0, -_g, 0), (0, 0, _g), (0, 0, -_g)])
            _vs = _t.Volume(); _t2 = _t.fuse(_seat).clean()
            if _t2.isValid() and len(_t2.Solids()) == 1:
                _t = _t2; print('pod', _sx, 'seat under its side cover, mm3', round(_t.Volume() - _vs, 1))
            else:
                print('pod', _sx, 'seat did not fuse (%d solids), left out' % len(_t2.Solids()))
            assert _t.isValid() and len(_t.Solids()) == 1, 'pod %d after the cover rebate: %d solids' % (_sx, len(_t.Solids()))
            print('pod', _sx, 'rebate under its side cover, mm3', round(_vr - _t.Volume(), 1))
        _d_pod[_sx] = _t; print('pod flange and cover bosses side', _sx, 'added mm3', round(_t.Volume() - _v0, 1))
    for _sx in (1, -1):
        _v0 = _d_pod[_sx].Volume(); _d_pod[_sx] = _ds.fit_cut(_d_pod[_sx], [_cage, _deck_r, _deck_t], _six)
        print('side pod', _sx, 'trimmed mm3', round(_v0 - _d_pod[_sx].Volume(), 1))
        # install path (tools/assembly_path.py): the pod slides off outward along x; whatever of it lies inboard of the rear deck's side wall in
        # that direction is cut away (the deck swept 30 mm inboard, FIT_CUT clear): it was a patch of the pod's front wall, x 49 .. 52
        _sw = _deck_r
        for _k in range(1, _ds.POD_SLIDE + 1):
            _sw = _sw.fuse(_deck_r.translate(cq.Vector(-_sx * _k, 0, 0)))
        _v0 = _d_pod[_sx].Volume(); _d_pod[_sx] = _ds.fit_cut(_d_pod[_sx], [_sw.translate(cq.Vector(-_sx * _fc, 0, 0))], [])
        print('side pod', _sx, 'cut for its sideways install path, mm3', round(_v0 - _d_pod[_sx].Volume(), 1))
    _n_scr = {'pod': 0, 'tub': 0}
    for (_bx, _by, _bz), _bd, _br in _cage_mod.LAST_BOSSES:
        _side = 1 if _bx > 0 else -1
        if _bd == _side:       # outward boss: side pod screw, through the skin from outside
            _before = _d_pod[_side].Volume(); _d_pod[_side] = _ds.screw_hole(_d_pod[_side], (_bx, _by, _bz), (_bd, 0, 0), _br + 14.0)
            _n_scr['pod'] += _before - _d_pod[_side].Volume() > 5.0
        else:                  # inward boss: nose tub screw, through the tub wall from inside
            _before = _d_tub.Volume(); _d_tub = _ds.screw_hole(_d_tub, (_bx, _by, _bz), (_bd, 0, 0), _br + 8.0)
            _n_scr['tub'] += _before - _d_tub.Volume() > 5.0
    print('desert shell screw holes cut at cage bosses:', _n_scr, 'of', len(_cage_mod.LAST_BOSSES), 'bosses')
    for _sx, _py, _pz, _x_end in _pod_lower:        # lower edge screws into the deck brackets
        _before = _d_pod[_sx].Volume(); _d_pod[_sx] = _ds.screw_hole(_d_pod[_sx], (_sx * (_x_end - 1.0), _py, _pz), (_sx, 0, 0), 20.0)
        assert _before - _d_pod[_sx].Volume() > 5.0, 'pod lower screw hole missed the pod at y %.0f' % _py
    print('pod lower edge screw holes:', len(_pod_lower))
    assert _n_scr['pod'] == 2 * _ds.POD_SCREWS and _n_scr['tub'] == 2 * len(_ds.TUB_BOSS_Y), 'a screw boss does not meet its panel: %r' % _n_scr
    for _nm, _sh in (('Nose tub', _d_tub), ('Hood deck', _d_hood), ('Side pod right', _d_pod[1]), ('Side pod left', _d_pod[-1])):
        _sh = _sh.clean(); assert _sh.isValid() and len(_sh.Solids()) == 1, _nm + ' is not one valid solid'
        add_part(_nm, _sh, color=_ds.COLOR_ARMOR, group='Desert shell')
    if _ds.POD_TOP_SPLIT:
        for (_k, _sx), _c in sorted(_d_cover.items()):
            add_part('Side cover %s %s' % (_k, 'right' if _sx > 0 else 'left'), _c, color=_ds.COLOR_ARMOR, group='Desert shell')
    else:
        for _sx, _nm in ((1, 'Side cover right'), (-1, 'Side cover left')):
            add_part(_nm, _d_cover[_sx], color=_ds.COLOR_ARMOR, group='Desert shell')
    if _ds.COWL_MODE != 'fused': add_part('Cowl panel', _d_cowl, color=_ds.COLOR_ARMOR, group='Desert shell')
    add_part('Dash panel', _d_dash, color=_ds.COLOR_ARMOR, group='Desert shell')
    add_part('Hex grille', _d_grille, color=_ds.COLOR_GRILLE, group='Desert shell')
    add_part('Nose light bar', _d_led, color=_ds.COLOR_LED, alpha=1, group='Lights')
    if _cabin and SHOW_CAGE:
        _d_roof = _ds.build_armored_roof(_cabn(_cabin)[1], _cabn(_cabin)[2], _hf(_CBM))
        add_part('Roof panel', _d_roof, color=_ds.COLOR_ROOF, group='Desert shell')
if _cabin and SHOW_CAGE and not DESERT:   # roof plate on the cage's roof frame (parts/roof_plate.py)
    from parts.roof_plate import make_roof_plate
    _K, _A, _B = _cabn(_cabin)
    _roofp = make_roof_plate(_A, _B, _hf(_CBM), _cage)
    print('roof plate mm3', round(_roofp.Volume()), 'bbox z', round(_roofp.BoundingBox().zmin, 1), round(_roofp.BoundingBox().zmax, 1))
    add_part('Roof plate', _roofp, color=(0.19, 0.22, 0.26), group='Cage')   # roof colour of parts/params.py (ROOF is the roof trace here)
# open cockpit: no roof above the rear contact any more, so the rear contact (cage boss and block) is dropped; the tail light wiring runs along the cage
for _n, (_y, _sh) in {k: v for k, v in {'Front': (LIGHT_FRONT_Y, None if DESERT else _shells['Front shell']), 'Rear': (LIGHT_REAR_Y, _shells.get('Canopy'))}.items() if v[1] is not None}.items():
    _rz = roof_underside(_sh, _y, 90.0)
    _, _block, _zd = make_connector(_y, _rz, _n, FRONT_LEG_X if _n == 'Front' else REAR_LEG_X, 0.0, legs=False,roof=_sh)
    _b0=_block.Volume(); _block=_block.cut(_sh).clean()   # E1: rear block sat 226 mm3 inside the canopy roof
    assert _block.isValid() and len(_block.Solids())==1 and _b0-_block.Volume()<400,'contact block trim failed'
    print('E1 contact block',_n,'trimmed mm3',round(_b0-_block.Volume(),1))
    if SHOW_CAGE: add_part(_n + ' light contact block', _block, color=(0.05, 0.05, 0.06), group='Cage/Light connectors')
