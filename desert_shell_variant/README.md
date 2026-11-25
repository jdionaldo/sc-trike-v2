# Fugu V2 — Option 3 Armored (Exo-Cage + Sealed Inner-Corridor Armor)

Isolated front-shell & exo-cage variant for `/Users/johndionaldo/GitHub/sc-trike-v2`, built cleanly in its own folder without modifying any existing repo files.

## Design Goals
- **Zero front suspension cutouts:** Every front part stays strictly inside the dead-clear center corridor (`|x| <= 49.5 mm` for the exo-cage, `|x| <= 39.8 mm` for the nose tub & hood), and the side pods start cleanly behind the front arm/lever sweep (`y >= -55 mm`). Verified at **0.0000 mm³ overlap** against all 14 `swingarm_relief_tools()` (+43 mm single-wheel travel, ±20° steering lock).
- **Chamfered square tubing:** All front exo-cage & V-bullbar struts use `7.2–7.8 mm` chamfered square cross-sections (flat faces + 45° corner chamfers, Z-aligned) for clean, support-friendly FDM 3D printing.
- **Sealed internal protection:** Full-length removable faceted hood/cowl deck, inboard nose side-walls, closed faceted chin/skid plate, **hexagonal honeycomb mesh intake grille** beneath the cyan LED light bar, and full-depth louvered side-pod flanks sealing the motor/ESC/battery bay.

## Parts (`exports/`)
| Part | File(s) | Role |
|---|---|---|
| `Front_Exo_Cage` | `.step` / `.stl` | Chamfered-square forward exo-cage & V-bullbar tying into Fugu V2's existing cage nodes (`(±45, -140, 57/112/139.3)` and `(±45, -215, 128)`) |
| `Armored_Nose_Tub` | `.step` / `.stl` | Inboard nose side-walls, raked front fascia (LED groove + hex intake opening), and closed faceted lower chin/skid plate |
| `Hex_Mesh_Grille` | `.step` / `.stl` | Separate printable hexagonal honeycomb intake mesh (`6.4 mm` cells, `1.9 mm` ribs) with rear retention flange (`0.3 mm` clearance) |
| `Nose_Light_Bar` | `.step` / `.stl` | Flush cyan LED light bar with twin 3 mm LED bore pegs on the rear |
| `Armored_Hood_Deck` | `.step` / `.stl` | Removable multi-faceted top hood/cowl armor with 6 M3 mounting holes |
| `Side_Pod_Right` / `Side_Pod_Left` | `.step` / `.stl` | Full-depth angular side-pod flank armor with 3 angled cyberpunk cooling louvers and M3 cage mounting holes |
| `Fugu_V2_Desert_Armored_Assembly.step` | `.step` | Full color-coded STEP assembly ready to drop straight into SolidCraft or SOMA |

## Rebuild & Render
```bash
cd /Users/johndionaldo/GitHub/sc-trike-v2
/Applications/SOMA.app/Contents/Resources/python/bin/python3.12 desert_shell_variant/build.py
/Applications/Blender.app/Contents/MacOS/Blender --background --python desert_shell_variant/render_cad.py
```
