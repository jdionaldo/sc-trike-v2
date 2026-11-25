"""Assemble the complete Fugu V2 reverse trike with the Option 3 Armored front shell, chamfered-square
front exo-cage, hexagonal mesh intake grille, sculpted side pods, and matching tapered roof panel.

Loads all existing components from /Users/johndionaldo/GitHub/sc-trike-v2/output/model.step
(preserving colors, transforms, and sub-assemblies), replaces ONLY:
  - Front_shell       -> Armored_Nose_Tub + Armored_Hood_Deck + Side_Pod_Right + Side_Pod_Left
  - Grille_insert     -> Hex_Mesh_Grille
  - Nose_light_strip  -> Nose_Light_Bar
  - Roof_plate        -> Armored_Roof_Panel
  + adds Front_Exo_Cage tying directly into the existing structural Cage.

Outputs:
  - desert_shell_variant/exports/Fugu_V2_Desert_Armored_Complete.step
  - /Users/johndionaldo/GitHub/sc-trike-v2/sc/Fugu_V2_Desert_Armored.sc  (via solidcraft-cli)
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HERE = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(HERE, "exports")
sys.path.insert(0, ROOT)

import cadquery as cq
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorGen, XCAFDoc_ColorSurf, XCAFDoc_ColorCurv
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.Quantity import Quantity_Color
from OCP.TopLoc import TopLoc_Location

from desert_shell_variant.exo_cage import build_front_exocage
from desert_shell_variant.armored_shell import build_all_armor

SKIP_PARTS = {
    "Front_shell",
    "Front shell",
    "Grille_insert",
    "Grille insert",
    "Nose_light_strip",
    "Nose light strip",
    "Roof_plate",
    "Roof plate",
    "Tail_light_housing",
    "Tail light housing",
}

NEW_COLORS = {
    "Front_Exo_Cage":       cq.Color(0.14, 0.15, 0.17, 1.0),
    "Armored_Nose_Tub":     cq.Color(0.28, 0.31, 0.27, 1.0),
    "Armored_Hood_Deck":    cq.Color(0.30, 0.33, 0.29, 1.0),
    "Armored_Roof_Panel":   cq.Color(0.16, 0.17, 0.19, 1.0),
    "Armored_Tail_Housing": cq.Color(0.28, 0.31, 0.27, 1.0),
    "Hex_Mesh_Grille":      cq.Color(0.18, 0.20, 0.22, 1.0),
    "Nose_Light_Bar":       cq.Color(0.05, 0.88, 0.95, 1.0),
    "Side_Pod_Right":       cq.Color(0.28, 0.31, 0.27, 1.0),
    "Side_Pod_Left":        cq.Color(0.28, 0.31, 0.27, 1.0),
}


def _label_name(lbl):
    nm = TDataStd_Name()
    if lbl.FindAttribute(TDataStd_Name.GetID_s(), nm):
        return nm.Get().ToExtString()
    return "Part"


def _shape_color(ct, shp):
    qc = Quantity_Color()
    for ctype in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen, XCAFDoc_ColorCurv):
        if ct.GetColor(shp, ctype, qc):
            return cq.Color(qc.Red(), qc.Green(), qc.Blue(), 1.0)
    return None


def main():
    step_in = os.path.join(ROOT, "output", "model.step")
    step_out = os.path.join(EXPORT_DIR, "Fugu_V2_Desert_Armored_Complete.step")
    sc_out = os.path.join(ROOT, "sc", "Fugu_V2_Desert_Armored.sc")
    os.makedirs(EXPORT_DIR, exist_ok=True)

    print(f"Reading existing Fugu V2 assembly from {step_in}...")
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    reader.ReadFile(step_in)
    reader.Transfer(doc)

    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    asm = cq.Assembly(name="Fugu_V2_Desert_Armored")
    kept = 0
    skipped = []
    seen_names = {}

    def unique_name(base):
        cnt = seen_names.get(base, 0)
        seen_names[base] = cnt + 1
        return base if cnt == 0 else f"{base}_{cnt}"

    def walk(lbl, loc_acc, prefix=""):
        nonlocal kept
        name = _label_name(lbl)
        if name in SKIP_PARTS:
            skipped.append(name)
            return
        if st.IsAssembly_s(lbl):
            comps = TDF_LabelSequence()
            st.GetComponents_s(lbl, comps)
            for i in range(1, comps.Length() + 1):
                comp = comps.Value(i)
                cloc = st.GetLocation_s(comp)
                new_loc = loc_acc.Multiplied(cloc)
                ref = TDF_Label()
                target = ref if st.GetReferredShape_s(comp, ref) else comp
                walk(target, new_loc, prefix)
        else:
            shp = st.GetShape_s(lbl)
            if shp.IsNull():
                return
            col = _shape_color(ct, shp)
            if not loc_acc.IsIdentity():
                shp = shp.Moved(loc_acc)
            cq_shp = cq.Shape.cast(shp)
            asm.add(cq_shp, name=unique_name(name), color=col)
            kept += 1

    free = TDF_LabelSequence()
    st.GetFreeShapes(free)
    for i in range(1, free.Length() + 1):
        walk(free.Value(i), TopLoc_Location())

    print(f"Kept {kept} existing Fugu V2 parts; replaced {len(skipped)}: {skipped}")

    print("Building and adding 8 new Option 3 Armored parts...")
    new_parts = {"Front_Exo_Cage": build_front_exocage()}
    new_parts.update(build_all_armor())
    for name, sol in new_parts.items():
        asm.add(sol, name=name, color=NEW_COLORS[name])

    print(f"Writing complete combined STEP to {step_out}...")
    asm.save(step_out)
    print(f"STEP saved ({os.path.getsize(step_out) / 1e6:.1f} MB)")

    sc_cli = "/Applications/SolidCraft.app/Contents/MacOS/solidcraft-cli"
    if os.path.exists(sc_cli):
        print(f"Saving native SolidCraft design to {sc_out}...")
        cmd = [sc_cli, "run", step_out, "--save", sc_out, "--quiet"]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0:
            print("solidcraft-cli stderr:", res.stderr)
            raise RuntimeError(f"solidcraft-cli failed with code {res.returncode}")
        print(f"SolidCraft file saved: {sc_out} ({os.path.getsize(sc_out) / 1e6:.1f} MB)")
    else:
        print("solidcraft-cli not found; STEP assembly saved at:", step_out)


if __name__ == "__main__":
    main()
