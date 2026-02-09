# GRScenes-test1 remaining `Material/mdl` missing — existence check

Input validate report:
- `check_reports/test1_category_merge_underscore_v3_validate_after_clear_missing_door_refs.json`

## Summary
- Missing entries (all `Material/mdl`): `47`
- Unique target paths (deduped by `Material/mdl/...`): `13`

Checked locations:
- Test0 subset: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0/Material`
- Source home: `/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/home_scenes/Materials`
- Source commercial: `/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/commercial_scenes/Materials`

Results:
- Exists in `GRScenes-test0/Material`: `0 / 13`
- Exists in source **home** `Materials` (found under `Materials/textures/...`): `4 / 13`
- Exists in source **commercial** `Materials`: `0 / 13`
- Missing in all three locations above: `9 / 13`

## Found in source home `Materials`
These four texture files exist in `home_scenes/Materials/textures/...` (but are missing in test1):
- `Material/mdl/textures/1c0/224/4efe3c3ce4f3f2c6857dff5b4fb8f88d694762f.jpg`
- `Material/mdl/textures/467/4fc/216362e4afa8b0b739586a6d76150b275118426.jpg`
- `Material/mdl/textures/704/609/d0b9ba417d09baab851ce1a5a5c1a152eba2f78.jpg`
- `Material/mdl/textures/bd8/8a5/0eef4d60d0831d83575a6762161b4f5650494eb.jpg`

## Missing in all checked locations
These 9 files were not found under test0 material nor either upstream `Materials` trees (search by exact path + basename):
- `Material/mdl/Base/Glass/Tinted_Glass_R50.mdl`
- `Material/mdl/Base/Glass/Tinted_Glass_R75.mdl`
- `Material/mdl/Base/Wood/Ash_Planks.mdl`
- `Material/mdl/Graphical_White_Jacquard_FL_3075963.mdl`
- `Material/mdl/textures/eyes03A.jpg`
- `Material/mdl/vMaterials_2/Concrete/Mortar.mdl`
- `Material/mdl/vMaterials_2/Glass/Glass_Glazing_Spandrel.mdl`
- `Material/mdl/vMaterials_2/Ground/Ground_Leaves.mdl`
- `Material/mdl/vMaterials_2/Other/Retroreflective/Retroreflective_Material.mdl`

## Raw data
- JSON report v1: `check_reports/test1_mdl_missing_existence_check.json`
- JSON report v2 (checks both `Materials/<tail>` and `Materials/mdl/<tail>` layouts): `check_reports/test1_mdl_missing_existence_check_v2.json`
