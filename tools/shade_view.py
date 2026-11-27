"""Shaded orthographic pictures of the built model without the app (VTK offscreen): renders/<prefix>_top.png and <prefix>_frontup.png.
top: straight down, front (-y) at the top of the picture (the owner's markup view). frontup: from the front, 35 deg above.
Run: python -I -B tools/shade_view.py [prefix]   (default prefix v2_shade)"""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'tools'))
import math
import vtk
import front2_pose as fp

ARMOR = (0.42, 0.47, 0.36)
COLORS = {'Cage': (0.07, 0.07, 0.08), 'Roof_panel': (0.20, 0.22, 0.26), 'Hex_grille': (0.10, 0.10, 0.11), 'Rear_tire': (0.10, 0.10, 0.11),
          'mock_tire': (0.85, 0.85, 0.87), 'Tail_light': (0.35, 0.40, 0.48)}
SHELL = ('Nose_tub', 'Hood_deck', 'Side_pod', 'Side_cover', 'Cowl_panel', 'Dash_panel', 'Rear_cover')


def colour(leaf):
    for k, c in COLORS.items():
        if leaf.startswith(k):
            return c
    if leaf.startswith(SHELL):
        return ARMOR
    return (0.80, 0.81, 0.83)


def actor(shape, rgb, tol=0.2):
    vs, ts = shape.tessellate(tol, 0.3)
    pts = vtk.vtkPoints()
    for v in vs:
        pts.InsertNextPoint(v.x, v.y, v.z)
    cells = vtk.vtkCellArray()
    for a, b, c in ts:
        cells.InsertNextCell(3); cells.InsertCellPoint(a); cells.InsertCellPoint(b); cells.InsertCellPoint(c)
    pd = vtk.vtkPolyData(); pd.SetPoints(pts); pd.SetPolys(cells)
    nm = vtk.vtkPolyDataNormals(); nm.SetInputData(pd); nm.SetFeatureAngle(20.0); nm.SplittingOn()
    m = vtk.vtkPolyDataMapper(); m.SetInputConnection(nm.GetOutputPort())
    a = vtk.vtkActor(); a.SetMapper(m); a.GetProperty().SetColor(*rgb)
    a.GetProperty().SetAmbient(0.25); a.GetProperty().SetDiffuse(0.8)
    return a, pd


def edges(pd):
    fe = vtk.vtkFeatureEdges(); fe.SetInputData(pd); fe.SetFeatureAngle(20.0); fe.BoundaryEdgesOn(); fe.FeatureEdgesOn()
    fe.ManifoldEdgesOff(); fe.NonManifoldEdgesOff()
    m = vtk.vtkPolyDataMapper(); m.SetInputConnection(fe.GetOutputPort()); m.ScalarVisibilityOff()
    a = vtk.vtkActor(); a.SetMapper(m); a.GetProperty().SetColor(0.12, 0.12, 0.12); a.GetProperty().SetLineWidth(1.0)
    return a


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else 'v2_shade'
    parts = fp.load(ROOT / 'output' / 'model.step')
    ren = vtk.vtkRenderer(); ren.SetBackground(0.15, 0.18, 0.24)
    for k, s in parts.items():
        lf = fp.leaf(k)
        a, pd = actor(s, colour(lf))
        ren.AddActor(a)
        if lf.startswith(SHELL) or lf.startswith('Roof_panel'):
            ren.AddActor(edges(pd))
    win = vtk.vtkRenderWindow(); win.SetOffScreenRendering(1); win.AddRenderer(ren); win.SetSize(1300, 1500)
    cam = ren.GetActiveCamera(); cam.ParallelProjectionOn()
    views = {'top': ((0.0, -30.0, 1000.0), (0.0, -1.0, 0.0), 330.0),
             'frontup': ((0.0, -1000.0 * math.cos(math.radians(35)), 1000.0 * math.sin(math.radians(35))), (0.0, 0.0, 1.0), 260.0)}
    for name, (pos, up, scale) in views.items():
        cam.SetFocalPoint(0.0, -30.0, 120.0); cam.SetPosition(pos[0], pos[1] - (30.0 if name == 'top' else 0.0), pos[2] + 120.0)
        cam.SetViewUp(*up); cam.SetParallelScale(scale); ren.ResetCameraClippingRange()
        if name == 'frontup':
            win.SetSize(1600, 1100)
        win.Render()
        w2i = vtk.vtkWindowToImageFilter(); w2i.SetInput(win); w2i.Update()
        out = ROOT / 'renders' / ('%s_%s.png' % (prefix, name))
        wr = vtk.vtkPNGWriter(); wr.SetFileName(str(out)); wr.SetInputConnection(w2i.GetOutputPort()); wr.Write()
        print('written', out)


if __name__ == '__main__':
    main()
