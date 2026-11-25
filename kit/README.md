# Fugu kit pipeline

Gate chain (run from the project root; exit code of every stage must be 0):

1. `soma build` then `python tools/collide.py` (SOMA python: /Applications/SOMA.app/Contents/Resources/python/bin)
2. `soma export --all --format stl --detail fine --out <dir>`
3. kit venv (trimesh, shapely, manifold3d, numpy, scipy, networkx, rtree, mapbox_earcut, lxml): `python kit/build_kit.py <dir> --out kit/print` (repair, shell mesh polish via `polish_stls.py` (`--no-polish` skips), split to bed with pins, orient, oriented STLs, parts.json)
4. `python kit/mesh_audit.py kit/print/stl kit/print/mesh_audit.json`
5. `python kit/make_manifest.py` then `cd kit/print && python ../bambu_kit.py kit_manifest.json --check` (Bambu Studio CLI slice of every plate)
6. `python kit/make_docs.py` (bom.md, assembly_manual.md)

`kit/registry.py` is the part registry (every printed STL stem must resolve, step 3 fails otherwise). `kit/p2s_profile.config` is the printer/process profile extracted from the user's saved P2S project.

## Version 2 suspension + drivetrain kit (November 24)

`kit/registry.py` now lists the Version 2 parts (leading swing arms; the wishbone registry is in the git history); the rest of the car is marked
optional there (full kit outputs are on hold). Bought parts: `kit/hardware.py`, sourcing `kit/sourcing_v2_2026-11-24.md`. Kit: `kit/susp_drive/README.md`.

1. `soma export --all --format stl --detail fine --out renders/kit_export`, copy the kit's stems (registry: SU, ST, DT and CH5) to `renders/kit_subset`
2. `python kit/build_kit.py renders/kit_subset --out kit/susp_drive --no-polish`, then `python kit/mesh_audit.py kit/susp_drive/stl kit/susp_drive/mesh_audit.json`
3. `python kit/make_susp_drive.py`, `cd kit/susp_drive && python ../bambu_kit.py kit_manifest.json --check`, `python kit/make_susp_drive.py --docs`
Python: the trimesh/manifold3d environment of the Version 1 checkout (`../sc-trike/Project_Fugu/.soma/env/bin/python`); this checkout has none.

## Version 2 desert shell: install and removal order (November 25)

Checked by `tools/assembly_path.py` (every part moves in 1 mm steps and must not touch anything that is already on).

1. Side pods (left and right): slide each pod in along x from outside (its last 30 mm straight in), then the M3 screws: from outside through the
   pod skin into the cage bosses, and the two lower-edge screws into the rear deck brackets.
2. Side covers: hold the cover 3 mm above its seat and 40 mm outboard, slide it straight in along x over the pod's top edge, lower it the
   3 mm onto the pod top, then the three M3 screws from above (y -100, -20, 30) into the bosses inside the pod's top edge.
3. Hood deck and roof panel: straight down from 30 mm above.

Removal is the reverse: hood and roof first, then the cover screws and each cover up 3 mm and out 40 mm, and only then the pods.
A pod cannot come off with its cover on: sliding out it hits the cover after 6 mm. A cover lifted 6 mm or more before it slides out hits the
cage at the front upright (y -138 .. -133), so keep it at about 3 mm.
