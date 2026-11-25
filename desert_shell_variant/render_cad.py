"""Headless Blender script to render multi-view CAD previews of the built Option 3 Armored STL parts.

Run with:
    /Applications/Blender.app/Contents/MacOS/Blender --background --python desert_shell_variant/render_cad.py
"""
import math
import os
import bpy
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(HERE, "exports")
RENDER_DIR = os.path.join(HERE, "renders")
os.makedirs(RENDER_DIR, exist_ok=True)

PART_MATS = {
    "Front_Exo_Cage":       ((0.06, 0.065, 0.075, 1.0), 0.35, 0.45),
    "Armored_Nose_Tub":     ((0.18, 0.21, 0.17, 1.0),   0.15, 0.55),
    "Armored_Hood_Deck":    ((0.22, 0.25, 0.20, 1.0),   0.15, 0.50),
    "Armored_Roof_Panel":   ((0.09, 0.10, 0.11, 1.0),   0.20, 0.48),
    "Armored_Tail_Housing": ((0.18, 0.21, 0.17, 1.0),   0.15, 0.55),
    "Hex_Mesh_Grille":      ((0.09, 0.10, 0.11, 1.0),   0.45, 0.40),
    "Nose_Light_Bar":       ((0.02, 0.90, 0.98, 1.0),   0.0,  0.10),
    "Side_Pod_Right":       ((0.18, 0.21, 0.17, 1.0),   0.15, 0.55),
    "Side_Pod_Left":        ((0.18, 0.21, 0.17, 1.0),   0.15, 0.55),
}

# Reset scene
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.engine = "BLENDER_EEVEE_NEXT" if "BLENDER_EEVEE_NEXT" in bpy.types.RenderSettings.bl_rna.properties["engine"].enum_items else "CYCLES"
if scene.render.engine == "CYCLES":
    scene.cycles.samples = 48
scene.render.resolution_x = 1280
scene.render.resolution_y = 800
scene.render.film_transparent = False

# Neutral light studio world
world = bpy.data.worlds.new("StudioWorld")
scene.world = world
world.use_nodes = True
bg = world.node_tree.nodes.get("Background")
if bg:
    bg.inputs[0].default_value = (0.90, 0.91, 0.92, 1.0)
    bg.inputs[1].default_value = 0.9

# Import all exported STLs (scaled mm -> m by 0.001)
for name, (rgba, metal, rough) in PART_MATS.items():
    stl_path = os.path.join(EXPORT_DIR, f"{name}.stl")
    if not os.path.exists(stl_path):
        continue
    if hasattr(bpy.ops.wm, "stl_import"):
        bpy.ops.wm.stl_import(filepath=stl_path, global_scale=0.001)
    else:
        bpy.ops.import_mesh.stl(filepath=stl_path, global_scale=0.001)
    obj = bpy.context.active_object
    obj.name = name
    mat = bpy.data.materials.new(name=f"Mat_{name}")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs["Base Color"].default_value = rgba
        bsdf.inputs["Metallic"].default_value = metal
        bsdf.inputs["Roughness"].default_value = rough
        if name == "Nose_Light_Bar":
            if "Emission Color" in bsdf.inputs:
                bsdf.inputs["Emission Color"].default_value = rgba
                bsdf.inputs["Emission Strength"].default_value = 3.5
    obj.data.materials.append(mat)

# Ground pad at z = -0.0156 (Fugu V2 ground plane)
bpy.ops.mesh.primitive_plane_add(size=0.9, location=(0.0, -0.10, -0.0156))
ground = bpy.context.active_object
gmat = bpy.data.materials.new("Ground")
gmat.use_nodes = True
gbsdf = gmat.node_tree.nodes.get("Principled BSDF")
if gbsdf:
    gbsdf.inputs["Base Color"].default_value = (0.25, 0.26, 0.28, 1.0)
    gbsdf.inputs["Roughness"].default_value = 0.85
ground.data.materials.append(gmat)

# Studio 3-point lighting
def add_light(name, ltype, loc, energy, rot=(0, 0, 0), size=0.6):
    ldata = bpy.data.lights.new(name=name, type=ltype)
    ldata.energy = energy
    if ltype == "AREA":
        ldata.size = size
    lobj = bpy.data.objects.new(name=name, object_data=ldata)
    scene.collection.objects.link(lobj)
    lobj.location = loc
    lobj.rotation_euler = rot
    return lobj

add_light("Key", "AREA", (0.55, -0.65, 0.70), 35.0, (math.radians(50), 0, math.radians(40)), 0.8)
add_light("Fill", "AREA", (-0.65, -0.45, 0.45), 18.0, (math.radians(65), 0, math.radians(-55)), 0.9)
add_light("Rim", "AREA", (0.0, 0.55, 0.65), 22.0, (math.radians(-55), 0, 0), 0.8)

# Camera helper
cam_data = bpy.data.cameras.new("Cam")
cam_data.lens = 50.0
cam = bpy.data.objects.new("Cam", cam_data)
scene.collection.objects.link(cam)
scene.camera = cam

def look_at(pos, target):
    cam.location = Vector(pos)
    direction = Vector(target) - cam.location
    cam.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()

views = [
    ("cad_iso.png",   (0.54, -0.64, 0.38), (0.0, -0.06, 0.125)),
    ("cad_front.png", (0.00, -0.62, 0.16), (0.0, -0.22, 0.120)),
    ("cad_side.png",  (0.72, -0.05, 0.15), (0.0, -0.06, 0.125)),
]

for fname, pos, tgt in views:
    look_at(pos, tgt)
    scene.render.filepath = os.path.join(RENDER_DIR, fname)
    bpy.ops.render.render(write_still=True)
    print("Rendered:", scene.render.filepath)
