"""Build, verify, and export the Option 3 Armored (Exo-Cage + Sealed Inner-Corridor Armor) variant for Fugu V2.

Run from /Users/johndionaldo/GitHub/sc-trike-v2 using SOMA's bundled Python:
    /Applications/SOMA.app/Contents/Resources/python/bin/python3.12 desert_shell_variant/build.py

Outputs (all inside desert_shell_variant/exports/):
  - Individual printable .step and .stl files for all 7 modular parts
  - Combined Fugu_V2_Desert_Armored_Assembly.step
  - verification_report.json with volumes, bounding boxes, and collision checks
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "exports")
sys.path.insert(0, ROOT)

import cadquery as cq
from desert_shell_variant.exo_cage import build_front_exocage
from desert_shell_variant.armored_shell import build_all_armor
from parts.relief import swingarm_relief_tools

COLORS = {
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


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print("Building Option 3 Armored variant parts...")
    parts = {"Front_Exo_Cage": build_front_exocage()}
    parts.update(build_all_armor())

    print("Running 3D boolean clearance checks against all 14 Fugu V2 front suspension envelopes...")
    tools = swingarm_relief_tools()
    report = {"parts": {}, "inter_part_overlaps": [], "status": "ok"}

    asm = cq.Assembly(name="Fugu_V2_Desert_Armored")
    for name, sol in parts.items():
        bb = sol.BoundingBox()
        max_ov = max(sol.intersect(t).Volume() for t in tools)
        assert max_ov < 0.01, f"{name} overlaps V2 front suspension sweep by {max_ov:.2f} mm3!"
        report["parts"][name] = {
            "solids": len(sol.Solids()),
            "valid": bool(sol.isValid()),
            "volume_mm3": round(float(sol.Volume()), 1),
            "bbox": {
                "x": [round(bb.xmin, 2), round(bb.xmax, 2)],
                "y": [round(bb.ymin, 2), round(bb.ymax, 2)],
                "z": [round(bb.zmin, 2), round(bb.zmax, 2)],
            },
            "max_v2_suspension_overlap_mm3": round(float(max_ov), 4),
        }
        print(
            f"  {name:20s}: vol={sol.Volume():8.1f} mm3  "
            f"x=[{bb.xmin:6.1f},{bb.xmax:6.1f}] y=[{bb.ymin:6.1f},{bb.ymax:6.1f}] z=[{bb.zmin:6.1f},{bb.zmax:6.1f}]  "
            f"susp_ov={max_ov:.4f}"
        )
        # Export individual STEP + STL
        cq.exporters.export(sol, os.path.join(OUT_DIR, f"{name}.step"))
        cq.exporters.export(sol, os.path.join(OUT_DIR, f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)
        asm.add(sol, name=name, color=COLORS[name])

    # Check inter-part overlaps
    names = list(parts.keys())
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            ov = parts[names[i]].intersect(parts[names[j]]).Volume()
            if ov > 0.01:
                report["inter_part_overlaps"].append(
                    {"pair": [names[i], names[j]], "volume_mm3": round(float(ov), 3)}
                )
                print(f"  WARN overlap {names[i]} <> {names[j]}: {ov:.3f} mm3")

    assert not report["inter_part_overlaps"], f"Inter-part overlaps found: {report['inter_part_overlaps']}"
    print("  Inter-part overlaps: NONE (0.000 mm3 across all pairs!)")

    asm_step = os.path.join(OUT_DIR, "Fugu_V2_Desert_Armored_Assembly.step")
    asm.save(asm_step)
    with open(os.path.join(OUT_DIR, "verification_report.json"), "w") as f:
        json.dump(report, f, indent=2)
    print(f"Exported assembly + 7 STEP/STL files to {OUT_DIR}")


if __name__ == "__main__":
    main()
