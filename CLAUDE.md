# CLAUDE.md

## Routine

After a **significant change** — one that makes something in this file wrong or incomplete (files added/renamed/reorganized, a new convention adopted, git/LFS setup changed) — say which section is affected and ask:

> Would you like to modify `CLAUDE.md` accordingly?

Do not ask on turns that change nothing here (questions, inspection, small edits that fit existing conventions). Keep this file lean — only what's needed to understand the repo.

## Repository nature

3D-printable coin + mechanical-key holder for a Kia Sorento. The source is a downloaded SolidWorks 2024 STEP export (no native CAD file). It holds two solids: the **tray** (larger) and the **lid**. They are hinged along X, with the axis at Y 26.42, Z −20.04 mm.

The original hinge needs two Ø2.41 mm metal rods. `make_rodless_hinge.py` (CadQuery) fills those bores and generates three rod-free variants:
- **keyed:** flatted lid pins drop into slotted knuckles at 45° open.
- **snap:** round pins in C-sockets.
- **print_in_place:** cone pins, printed assembled with the lid open 90°.

All variants also get the lid's underside pocket (67 × 39 × 3.7 mm) filled in, which the script calls `shrink_pocket`. Only a strip at the hinge edge stays open, because the tray's back wall swings through it. The fill has a 45° rear face so it prints without support. `POCKET_FILL_Z` sets where the fill starts.

The lid's flat top (plane at Y 29.85) gets a debossed leather texture in two styles: **grain** (Voronoi pebbles) and **stitched** (grain + a dashed stitch line 3 mm inside the edge). The texture is applied to the mesh only, so the STLs and GLBs carry it and the STEP stays smooth. It is cut in only, never raised, so hinge clearances are unaffected; a 2 mm border stays flat because the lid is only 1.07 mm thick at its front edge. The texture parameters are the `TEX_*` / `STITCH_*` constants.

Every geometric check the script makes prints PASS/FAIL. All fit dimensions are constants at the top of the script.

- The STEP is in **inches**; CadQuery imports it in mm.
- Run the script with the PowerShell `python`, which has cadquery, trimesh and matplotlib. Do not pass `-I`, because it hides the user site-packages. The Bash shell's python has no cadquery.
- Don't call `.clean()` on booleans with the imported tray: it makes the solid invalid.

## Layout

- `Original/coin.STEP` — original model; never modified.
- `make_rodless_hinge.py` — generator + checks. `python make_rodless_hinge.py [keyed|snap|print_in_place ...]`.
- `CoinBox/<variant>/` — generated output (`<style>` is `grain` or `stitched`):
  - `coin_<variant>.step` — both bodies, smooth.
  - STLs — keyed/snap: `coin_<variant>_tray.stl` + `coin_<variant>_lid_<style>.stl`; print_in_place: one assembled `coin_print_in_place_<style>.stl` (lid at 90°).
  - `coin_<variant>_<style>.glb` — animated assembly with the textured lid.
  - `coin_<variant>_hinge.png` (hinge close-up) and `coin_<variant>_leather.png` (texture relief) previews.
- `README.md` — GitHub-facing description; embeds the `keyed` hinge and leather PNGs. Update it when variants, styles or output names change.
- `.codex/` — another tool's local config; git-ignored, not part of the model.

## Git

Remote: `github.com/shivamchudasama/KiaSorentoCoinBox_3DPrinting`, branch `main`.

All generated output in `CoinBox/` is committed, about 310 MB. The STL, STEP (`*.step` and `*.STEP`), GLB, 3MF and PDF files go to Git LFS through `.gitattributes`. PNGs are ordinary blobs. LFS free tier is 1 GB storage / 1 GB/month bandwidth, so every regeneration that changes the STLs/GLBs costs about 300 MB of storage. Regenerate and commit outputs deliberately, not on every tweak.
