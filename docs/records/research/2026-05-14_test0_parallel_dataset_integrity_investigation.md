---
title: Test0 Parallel Dataset Asset and Material Integrity Investigation
doc_class: record
code_reference:
  - scripts/check_usd_external_assets.py
  - scripts/sync_missing_mdl_textures.py
  - scripts/scan_usd_for_asset_paths.py
created_at: 2026-05-14
updated_at: 2026-05-14
maintainer: Codex
status: investigation-complete
---

# Test0 Parallel Dataset Asset and Material Integrity Investigation

## Scope

调查对象：

`/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset`

本次只做调查和记录，没有修改 dataset 内容。覆盖两个问题：

1. `GRScenes_assets` 中缺失 `usd/<uid>.usd` 和四视角 PNG 的资产壳。
2. `Material/mdl` 相关依赖缺失，包括 layout 直接引用的 MDL/贴图，以及 MDL 文件内部引用的贴图。

## Executive Summary

### 缺 USD/PNG 的资产壳

最终 dataset 的 `GRScenes_assets` 共有 79 个 category、53171 个二级资产目录。全量扫描确认只有 4 个资产目录同时缺失预期 USD、缺失 `front.png`/`left.png`/`back.png`/`right.png`，并且都是 annotation-only 目录：

| asset | state |
| --- | --- |
| `cabinet/b98d6ccbeb75dfdeb60e27649a5b055a` | only `*_annotation.json`; no USD; no PNG |
| `other/d41d8cd98f00b204e9800998ecf8427e` | only `*_annotation.json`; no USD; no PNG |
| `person/351316cbb083f9f4df0cccd60cbfa848` | only `*_annotation.json`; no USD; no PNG |
| `person/d41d8cd98f00b204e9800998ecf8427e` | only `*_annotation.json`; no USD; no PNG |

这些目录在上游 workspace `test0_transitive_apply_20260421_103046` 中已经是同样状态，因此不是这次 OSS 上传造成的，也不是当前 render recovery/copy-back 丢失。

更重要的是，99 个 `layout.usd` 中仍有 15 处 authored references 指向这 4 个缺失 USD。仅把它们排除出 render-missing 队列不能完全修复 dataset；发布前需要恢复 USD 或清理 scene reference。

### Material/MDL 缺失

Material 问题分两层：

| layer | result |
| --- | ---: |
| layout 直接引用的缺失 `Material/mdl` 路径 | 9 个唯一路径，58 次属性引用，影响 9 个 layout |
| MDL 文件内部引用的缺失贴图 | 171 个唯一 `.jpg` 路径，290 次引用，影响 102 个 `.mdl` 文件 |

这些缺失在 current dataset 和上游 source workspace 的 dataset 中都不存在。已按已知 legacy Materials 根目录做候选搜索：

- `/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/home_scenes/Materials`
- `/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/commercial_scenes/Materials`

结果没有找到可直接补齐的候选文件。

## Evidence

### Workspace Lineage

`test0_transitive_apply_parallel/workspace_manifest.json` 记录的 source workspace：

`/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046`

关键 manifest 字段：

| field | value |
| --- | --- |
| `type` | `parallel_c1_pipeline` |
| `source_workspace` | `/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046` |
| `dedup_mode` | `geom_only` |
| `step6_mode` | `apply` |
| `completed_categories` | 24 |
| `remaining_categories` | 59 |

### Asset Directory Integrity

全量扫描结论：

| check | count |
| --- | ---: |
| categories under `GRScenes_assets` | 79 |
| asset directories under `GRScenes_assets/<category>/<uid>` | 53171 |
| missing expected `usd/<uid>.usd` | 4 |
| missing at least one of four required PNGs | 4 |
| annotation-only direct directories | 4 |

缺失清单：

```text
cabinet/b98d6ccbeb75dfdeb60e27649a5b055a
other/d41d8cd98f00b204e9800998ecf8427e
person/351316cbb083f9f4df0cccd60cbfa848
person/d41d8cd98f00b204e9800998ecf8427e
```

每个目录当前只有一个 annotation JSON：

| asset | files |
| --- | --- |
| `cabinet/b98d6ccbeb75dfdeb60e27649a5b055a` | `b98d6ccbeb75dfdeb60e27649a5b055a_annotation.json` |
| `other/d41d8cd98f00b204e9800998ecf8427e` | `d41d8cd98f00b204e9800998ecf8427e_annotation.json` |
| `person/351316cbb083f9f4df0cccd60cbfa848` | `351316cbb083f9f4df0cccd60cbfa848_annotation.json` |
| `person/d41d8cd98f00b204e9800998ecf8427e` | `d41d8cd98f00b204e9800998ecf8427e_annotation.json` |

上游 source workspace 中同样只有 annotation JSON，且缺少预期 USD：

| asset | upstream exists | upstream expected USD |
| --- | --- | --- |
| `cabinet/b98d6ccbeb75dfdeb60e27649a5b055a` | yes | no |
| `other/d41d8cd98f00b204e9800998ecf8427e` | yes | no |
| `person/351316cbb083f9f4df0cccd60cbfa848` | yes | no |
| `person/d41d8cd98f00b204e9800998ecf8427e` | yes | no |

Annotation 摘要：

| asset | `asset_type` | `usd_size` | metadata quality |
| --- | --- | ---: | --- |
| `cabinet/b98d6ccbeb75dfdeb60e27649a5b055a` | `articulation` | 0.0021810531616210938 | `description`/`material`/`dimensions`/`mass` empty |
| `other/d41d8cd98f00b204e9800998ecf8427e` | `rigid` | 0.0014247894287109375 | `description`/`material`/`dimensions`/`mass` empty |
| `person/351316cbb083f9f4df0cccd60cbfa848` | `rigid` | 0.0013332366943359375 | `description`/`material`/`dimensions`/`mass` empty |
| `person/d41d8cd98f00b204e9800998ecf8427e` | `articulation` | 0.001537322998046875 | `description`/`material`/`dimensions`/`mass` empty |

`d41d8cd98f00b204e9800998ecf8427e` 是空字符串的 MD5，说明它更像上游空 ID 或占位输入产物。

### Scene References To Missing USD

使用 USD API 打开 99 个 `layout.usd` 并检查 authored references，发现 15 处引用仍指向上述缺失 USD：

| missing asset USD | occurrences | scene count | scenes |
| --- | ---: | ---: | --- |
| `GRScenes_assets/cabinet/b98d6ccbeb75dfdeb60e27649a5b055a/usd/b98d6ccbeb75dfdeb60e27649a5b055a.usd` | 6 | 2 | `GRScenes100/home/MVUCSQAKTKJ5EAABAAAAABY8_usd/layout.usd`, `GRScenes100/home/MWAX5JYKTKJZ2AABAAAAADI8_usd/layout.usd` |
| `GRScenes_assets/other/d41d8cd98f00b204e9800998ecf8427e/usd/d41d8cd98f00b204e9800998ecf8427e.usd` | 7 | 1 | `GRScenes100/commercial/MVSYCXYKTKJ66AABAAAAAAA8_usd/layout.usd` |
| `GRScenes_assets/person/351316cbb083f9f4df0cccd60cbfa848/usd/351316cbb083f9f4df0cccd60cbfa848.usd` | 1 | 1 | `GRScenes100/commercial/MVSGSAIKTKJ66AABAAAAADY8_usd/layout.usd` |
| `GRScenes_assets/person/d41d8cd98f00b204e9800998ecf8427e/usd/d41d8cd98f00b204e9800998ecf8427e.usd` | 1 | 1 | `GRScenes100/commercial/MVSYCXYKTKJ66AABAAAAAAA8_usd/layout.usd` |

Stage open stderr also emitted 15 `Could not open asset` warnings, matching the 15 authored references.

The three UID strings were not found in the `c1_bulk/{cabinet,other,person}` mapping/audit records, which supports the conclusion that these are not C1 replacement outputs. They were inherited as incomplete upstream asset records.

## Layout-Level Missing Material Dependencies

Using `scripts/check_usd_external_assets.py` logic over all 99 `layout.usd` files:

| metric | value |
| --- | ---: |
| layouts scanned | 99 |
| failed layouts | 0 |
| unique missing `Material/mdl` paths | 9 |
| total missing material occurrences | 58 |
| scenes with at least one missing material path | 9 |

Missing paths:

| missing path | occurrences | scene count |
| --- | ---: | ---: |
| `Material/mdl/Base/Glass/Tinted_Glass_R50.mdl` | 2 | 2 |
| `Material/mdl/Base/Glass/Tinted_Glass_R75.mdl` | 1 | 1 |
| `Material/mdl/Base/Wood/Ash_Planks.mdl` | 5 | 2 |
| `Material/mdl/Graphical_White_Jacquard_FL_3075963.mdl` | 1 | 1 |
| `Material/mdl/textures/eyes03A.jpg` | 1 | 1 |
| `Material/mdl/vMaterials_2/Concrete/Mortar.mdl` | 5 | 1 |
| `Material/mdl/vMaterials_2/Glass/Glass_Glazing_Spandrel.mdl` | 2 | 2 |
| `Material/mdl/vMaterials_2/Ground/Ground_Leaves.mdl` | 33 | 7 |
| `Material/mdl/vMaterials_2/Other/Retroreflective/Retroreflective_Material.mdl` | 8 | 1 |

Affected scenes:

```text
GRScenes100/commercial/MV5M25QKTKJZ2AABAAAAAAY8_usd/layout.usd
GRScenes100/commercial/MV7J6NIKTKJZ2AABAAAAAAA8_usd/layout.usd
GRScenes100/commercial/MVJWVGYKTLDAYAABAAAAAAQ8_usd/layout.usd
GRScenes100/commercial/MVSGSAIKTKJ66AABAAAAAEA8_usd/layout.usd
GRScenes100/commercial/MVSYCXYKTKJ66AABAAAAACY8_usd/layout.usd
GRScenes100/commercial/MWF4WLIKTIFZIAABAAAAABY8_usd/layout.usd
GRScenes100/commercial/MWF4WLIKTIFZIAABAAAAACQ8_usd/layout.usd
GRScenes100/commercial/MWF4WLIKTIFZIAABAAAAADI8_usd/layout.usd
GRScenes100/commercial/MWHLEPQKTIFZIAABAAAAAAA8_usd/layout.usd
```

These 9 paths are absent from both:

- current dataset: `test0_transitive_apply_parallel/dataset`
- upstream dataset: `test0_transitive_apply_20260421_103046/dataset`

Exact filename search under the two known legacy Materials roots also returned no hits.

## MDL-Internal Missing Texture Dependencies

Using `scripts/sync_missing_mdl_textures.py` over the current dataset's `Material/mdl` tree:

| metric | value |
| --- | ---: |
| total `.mdl` files | 1721 |
| texture references found in `.mdl` files | 1836 |
| missing texture reference occurrences | 290 |
| unique missing texture paths | 171 |
| file extensions | all `.jpg` |
| impacted `.mdl` files | 102 |

Search against known legacy Materials roots found no direct candidates:

| search roots | found candidates |
| --- | ---: |
| `home_scenes/Materials`, `commercial_scenes/Materials` | 0 |

Top impacted MDL files:

| MDL file | missing occurrences | unique missing textures |
| --- | ---: | ---: |
| `Material/mdl/652553e329931a0001ad9024.mdl` | 11 | 6 |
| `Material/mdl/652553e4b9b35600017dfa77.mdl` | 11 | 6 |
| `Material/mdl/5e1452a100e3a3000138e08b.mdl` | 5 | 5 |
| `Material/mdl/652553e38ffa490001faa5fd.mdl` | 5 | 5 |
| `Material/mdl/652553e41ea6be000148420f.mdl` | 5 | 5 |
| `Material/mdl/652553e3f14b7400016a2aa8.mdl` | 9 | 4 |
| `Material/mdl/652553e4e2d2b80001b31c56.mdl` | 9 | 4 |
| `Material/mdl/5e1452a15ea89f0001b3f47f.mdl` | 8 | 4 |
| `Material/mdl/5e1452a123bc840001d4ca23.mdl` | 4 | 4 |
| `Material/mdl/5e1452a1b1d8ad0001dae23f.mdl` | 4 | 4 |

Complete unique missing texture list is in Appendix A.

## Root Cause Assessment

### Missing USD/PNG

Root cause is upstream dataset assembly retaining four metadata-only asset shells. The current parallel workspace inherited these directories from `test0_transitive_apply_20260421_103046`.

Evidence:

- The four directories already exist upstream as annotation-only records.
- Current render discovery expects `Category/UID/usd/UID.usd`; these four records do not have that path and cannot be enqueued for rendering.
- `layout.usd` files still reference the missing USD files, so this is a dataset integrity issue, not only a render-thumbnail queue issue.
- `d41d8cd98f00b204e9800998ecf8427e` is an empty-string MD5 placeholder and appears in two categories.

### Material/MDL

Root cause is incomplete Material packaging in the source/current dataset lineage:

- 9 layout-level material paths are authored in scenes but absent from current and upstream dataset roots.
- 171 MDL-internal texture files are referenced by packaged MDL files but absent from `Material/mdl/textures`.
- Candidate search in the known legacy Materials roots found no directly copyable files for the MDL-internal missing textures.

This likely predates the OSS upload and should be handled as a dataset completeness issue before the next release or re-upload.

## Recommended Handling

### For the 4 missing USD assets

Choose one policy before publishing:

1. Restore the original source USDs and four render PNGs for the four assets, then rerun scene/material validation.
2. If the assets are invalid placeholders, delete the four asset records and remove or rewrite the 15 scene references that point to them.
3. If only render completeness reporting is being cleaned, exclude the four rows from render-missing queues, but record that scene references still fail until option 1 or 2 is done.

For release quality, option 1 or 2 is required. Option 3 only suppresses render retry noise.

### For layout-level missing Material/MDL paths

Recover or vendor the 9 missing files into `Material/mdl` while preserving the authored relative paths. Most are standard-looking MDL library files under `Base/*` or `vMaterials_2/*`; they may need to come from the original Omniverse/vMaterials source rather than the GRScenes legacy Materials roots.

After copying, rerun the 99-layout external dependency check and confirm `unique_missing_material_paths=0` for layout-level material paths.

### For MDL-internal missing textures

The 171 missing `.jpg` files are not fixed by copying from the two known legacy Materials roots. Next source search should be against the original material export cache or the package that generated the hash-sharded `Material/mdl/textures` tree.

After recovery, rerun:

```bash
python scripts/sync_missing_mdl_textures.py \
  --mdl-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/Material/mdl \
  --report /tmp/test0_parallel_missing_mdl_textures_report.json
```

Expected success condition: exit code 0, `missing_count=0`, `unique_missing_paths=0`.

### For future upload/release gates

Add pre-upload checks before OSS publication:

1. Asset directory gate:
   - every `GRScenes_assets/<category>/<uid>` must have `usd/<uid>.usd`
   - every rendered asset expected in final package must have `front.png`, `left.png`, `back.png`, `right.png`
   - no annotation-only asset shell unless explicitly allowlisted
2. Scene reference gate:
   - every authored USD reference in every `layout.usd` must resolve locally
3. Material gate:
   - run `scripts/check_usd_external_assets.py` across all layouts and fail on missing `Material/mdl` paths
   - run `scripts/sync_missing_mdl_textures.py` and fail on MDL-internal missing textures

## Appendix A: Unique Missing MDL Texture Paths

```text
Material/mdl/textures/007/7f1/6e0f68f3e0fe013bfc8f197eedd5452ebfd0726.jpg
Material/mdl/textures/013/499/5df415c8b78b590207f8fbc3e4517dabd2ed3b4.jpg
Material/mdl/textures/013/560/40c15f2f516275d7ba6e5daf0d6efc4b5d2526.jpg
Material/mdl/textures/020/f1c/9bbfe31f9150c5cf9180a550d52e35e585138ad.jpg
Material/mdl/textures/061/b12/b8f44bb5914fb833c1d602339aa4cb86227536.jpg
Material/mdl/textures/063/f50/949a92bb3eb8901c83ce736e43fbec89657d95f.jpg
Material/mdl/textures/065/6c3/9c21bd7c8108d6f62d1142628c979619cddb5c.jpg
Material/mdl/textures/087/3b6/e9d604903285ff588a185d0ce2aad3c18a18857.jpg
Material/mdl/textures/0a6/a8d/85b5f1d56ddf4d18e51c45221bf7db5b123567.jpg
Material/mdl/textures/0af/6b4/413f3033fa668b6b7a0cab1f8cb750f7bc7cf38.jpg
Material/mdl/textures/0b0/4aa/4b82a6e9c3790d26cdc9a0f51cdc28005e1e02f.jpg
Material/mdl/textures/11c/669/88570d9ef92b9f9725a6b6feb1ed360aabedf68.jpg
Material/mdl/textures/13f/bff/39a89466e0fe8c7471e742861fa850c85776c91.jpg
Material/mdl/textures/144/699/cbb8cfa9a19125d24014a69fa83e5bd8321c3d.jpg
Material/mdl/textures/14e/b85/d6047f51d9b64f9f532e3a7beb2b961cdd8dfed.jpg
Material/mdl/textures/18d/8e4/3f952330849359cb88a73f6fcffccaf118f6ea3.jpg
Material/mdl/textures/18e/a52/098b88836a2f049100aae09186eb8734138873e.jpg
Material/mdl/textures/1b3/8dd/64c67d5eee197ba2e3bdb121522f2abae79f7d5.jpg
Material/mdl/textures/1c0/194/9ea9dbdcf44dea72beb834bf7e5403f9c63c15.jpg
Material/mdl/textures/1cf/9ec/91936884b7257c13df634373fc6df8d93f102c4.jpg
Material/mdl/textures/1d8/940/0f69a5c2d0d811e439e7d2b808aa2fca34a2d0.jpg
Material/mdl/textures/1fa/091/b1cce72dbf923d123b6cb53112e38b2721335b5.jpg
Material/mdl/textures/211/a39/3915088bd7a45b2f1fe37cb447d94e66b618f18.jpg
Material/mdl/textures/23d/745/db65a8d565c87b31d1e07e1107f10160ad96fcb.jpg
Material/mdl/textures/23d/7d4/373579abc13247cda19e567e95a319cac23d6a.jpg
Material/mdl/textures/266/0e7/cd2ba8de911e88bbdc2b46145557f5e99788592.jpg
Material/mdl/textures/275/1fc/58a9807511c00d283fb87a0527c2b2d2474da78d.jpg
Material/mdl/textures/284/7a5/794ee26c50385b01b7a0d48b6af66e836fa049.jpg
Material/mdl/textures/2c7/1c4/12b7fb1f7e890df5806bc3ddeeaa71339a6e2bb.jpg
Material/mdl/textures/2cc/5ca/377592102a98731478c4730251b09c2aea21f0ed.jpg
Material/mdl/textures/2fa/fac/78cb65bbfb38afdcb16adb4b743e03fb341aa6.jpg
Material/mdl/textures/318/160/298081f661299ac217fa114e3ba99846ba2aa15.jpg
Material/mdl/textures/346/8d0/3cc3f491997198e069443566faa24d1bf47b59f8.jpg
Material/mdl/textures/347/cbe/6e41e3cfee8733383e7dcb83b2edf3dc85933e7.jpg
Material/mdl/textures/353/177/47039aecf007d04971b7fab74a34e5a23d3e1a.jpg
Material/mdl/textures/353/6be/6a2351e28c6a1a8d6dfd8b6840760172bb2d137.jpg
Material/mdl/textures/35d/374/2ac16206dff2eb42915ea2082248937413cc54.jpg
Material/mdl/textures/366/376/09e3a83eb6e5f00f058d8462d39b927632a1ff8.jpg
Material/mdl/textures/396/2bf/990a2e91829d6d4e089667025857877dba7b523.jpg
Material/mdl/textures/39e/257/9f431296a40f1959ee28807f765f0dfa7c1656eb.jpg
Material/mdl/textures/3a8/41e/8386d7c84b715b2a102ee32c25b6121d2c18222.jpg
Material/mdl/textures/3ac/2ab/bb8c0a66b2936c09cfbb64a6047fb8e22a8ecad.jpg
Material/mdl/textures/3dc/769/93ff19a68125bdd5ecb1f261372acc27836eb70.jpg
Material/mdl/textures/404/6b2/0194038f08ebeb74f16a63ca81adba93aa84597.jpg
Material/mdl/textures/40d/c46/e5afc36ee56c98740b68110ad30b2c99ae31fcc.jpg
Material/mdl/textures/415/bc5/cd1727e6b652f9a5f5e1cff92206530810c51cb.jpg
Material/mdl/textures/41b/dee/bb2a598dc598022aa506c1980f9d15743a2971f.jpg
Material/mdl/textures/439/910/7971d929085742c610be4ded359f41460017f1c.jpg
Material/mdl/textures/442/e43/2534a8f16293c12de51208485f9cde8cbcc570.jpg
Material/mdl/textures/45e/960/95b32809f3be0558839c0d4be90e4d6d09ae5a5.jpg
Material/mdl/textures/471/1d9/80ce566a95c92451f67101c117f16c74f12b353.jpg
Material/mdl/textures/47d/11d/d7d351c5b3ad85081dbb28a1173abbd52c2d0ee1.jpg
Material/mdl/textures/47f/100/76ccb3467b48dafcc43ea58a76d5eb0a49cdec.jpg
Material/mdl/textures/488/22d/7a8ab3e0e3143a78311caf7f85905921ac25fa7.jpg
Material/mdl/textures/4ba/c72/dae840b56cdb607fb132731fb0618371a239efa.jpg
Material/mdl/textures/4c7/60f/f3cafeac217f233bebe48a50ca097e1e022259f.jpg
Material/mdl/textures/4cc/c5c/49c46af8afe588342b1d277fbdc801f09432f95.jpg
Material/mdl/textures/4e6/a4d/bd1606afef8f5e1eccb1a52e4ce97d0409188c6.jpg
Material/mdl/textures/520/a18/cebbc1ca039b9d68102b5277ab71ca668fb87fd.jpg
Material/mdl/textures/524/bc9/ec2f5eb3ec18c885056f723e4a01b925589c21e.jpg
Material/mdl/textures/52b/0e3/94079db8c052db9db01026559cda933071fb2b.jpg
Material/mdl/textures/53d/3df/4abc636093eb9025dc568720534a98ad9f4091.jpg
Material/mdl/textures/542/737/ea8e1ea73c86772694c9fcd0510430bfb73dd3ed.jpg
Material/mdl/textures/551/46a/d031bf7ce1231c4e6fbddf24096c6760db309f3.jpg
Material/mdl/textures/55c/264/7f058d824eca77e0528824fee5e7cdd44c10ef65.jpg
Material/mdl/textures/563/8c7/7c09749576dcc23e36acf100624b4f969fb4be6.jpg
Material/mdl/textures/566/c90/d5c84f0e298c516eb729aca4eb093efd1c7fc8e4.jpg
Material/mdl/textures/582/c01/923347c10d1eb399d8cbcb6aef30b2b86f1da38.jpg
Material/mdl/textures/590/c75/66a1811b0f4f99344658a09dda4ffa96fdbd7ec.jpg
Material/mdl/textures/5a0/8ee/e58d9828bef14b6b2b26ff27f949d3e49112806.jpg
Material/mdl/textures/5ac/598/faf2eb1b99e4f0e10c96a5bac2bf336606d13734.jpg
Material/mdl/textures/5b1/5b4/e32c585170cd2fcb1ed9f22dd644e7aac27b46.jpg
Material/mdl/textures/5d7/1ce/05080ccaca48cd3d30eee04d47dbc690dd1d62d.jpg
Material/mdl/textures/601/ecd/eaaf55e1689e5efdfc6747fb3f90e03076608d1.jpg
Material/mdl/textures/604/f75/1cf8da1ae4c32ad1bec273c036257331bb315d8.jpg
Material/mdl/textures/62a/29e/2e6e9c1f9e1c263b83c75cafb318ec123e2a3464.jpg
Material/mdl/textures/64d/026/f8f7fdb35157371023916312b36cf9f88ecc748.jpg
Material/mdl/textures/652/99c/dedaa92340e77418a20269fff0b038673ed180a.jpg
Material/mdl/textures/65b/498/8ff475119af2f21b75b664fff794339aa61dc1.jpg
Material/mdl/textures/671/239/2b7ad3341245624dcbb8dcc37dc357b8f8204174.jpg
Material/mdl/textures/684/363/bdd0f1b65e0cd87730bd62ce45e6dca72d30500.jpg
Material/mdl/textures/6a5/09b/639bd4bbefeee9ed8717845b6586e08ad0eb0d.jpg
Material/mdl/textures/6af/21b/28d4b47acc019f44969b5fe8afec5f937021652.jpg
Material/mdl/textures/6b4/9fb/8c3ce5db72112d2eafe9cfb63b2b4055ea81a8b.jpg
Material/mdl/textures/6e7/b2b/388f1716ff733cda1dbe32f053c319b27d17033e.jpg
Material/mdl/textures/719/018/89ca887aad0aaeee28d937d7df16b41ebf12c53.jpg
Material/mdl/textures/729/a87/a8cdfdc245a97c64a559e7986b658cbee8828e.jpg
Material/mdl/textures/740/f66/39b69a221d8a3028eed420e8929799542ddc3f.jpg
Material/mdl/textures/74a/77e/0f1488fd91cf286cfc1586d9c23bc1b3f1a983.jpg
Material/mdl/textures/74c/426/ef7c5e8b53ccfbc01fdd0b43cd91c28da6191c.jpg
Material/mdl/textures/79a/841/778f3c8cf49e88e5c66ea99a2e09cd7ce25690db.jpg
Material/mdl/textures/7b4/1e8/77d42aabba7ccc60e177457f9b1991af872a508b.jpg
Material/mdl/textures/7d5/f20/f874a9cc9a0e2702dacc39b6d5a3f8463012bff9.jpg
Material/mdl/textures/7dc/10f/ab172bda5fd2b974a6621c40fcbd6d27ca362ad.jpg
Material/mdl/textures/7e0/f36/d83d509abfe878e88e5259cd5c91b1071e2839f.jpg
Material/mdl/textures/7e1/403/aa7c5516113d7f7fbe52a0d87e0e778e1d3898a.jpg
Material/mdl/textures/7fc/5ab/da84aafff1b52220e19a8aae3667ef9706fb54.jpg
Material/mdl/textures/801/bb5/049d4e57db982d6ef50f41bce81f2c9dc857507.jpg
Material/mdl/textures/819/015/677a8cfc02225065d01e25a800a2f639ba34323.jpg
Material/mdl/textures/81f/8ab/49211e197ac400c68bfab7ba773268125452cd6.jpg
Material/mdl/textures/81f/a28/ea84fd3e2942816577650324eeeb25655438c5.jpg
Material/mdl/textures/840/42c/040d620308beb38e16d7b35cbe29f1cc7688b5d.jpg
Material/mdl/textures/85e/4ea/58e587c6517330cb335b65b40bd5ef3d457b96a.jpg
Material/mdl/textures/88a/4ca/d3b7cb268cff63f216b27ecc19276edcf984bf6.jpg
Material/mdl/textures/89f/9ad/c3e49cdbcbd4cf8282aac4dae2341223705550.jpg
Material/mdl/textures/8b7/4bb/4fda1126392af022489ecb4ab853e1eafb59a64.jpg
Material/mdl/textures/915/859/a649a8d8e12cc5f6edb964cb5065ea206e18012.jpg
Material/mdl/textures/941/4a1/522359cf800cf453421136726821518b737540.jpg
Material/mdl/textures/94f/9a7/9eafb73fe8a394bde1dae874f6f24775dd6d4b.jpg
Material/mdl/textures/961/9f7/6a992a70d4021dc8f269b068189f561ae025955.jpg
Material/mdl/textures/987/e0b/d60703b540841fdd0f9bd8e742684ffa6ebe51.jpg
Material/mdl/textures/9a3/f48/3ea23b0237597fef4f2616468d4c40a3071fcf0.jpg
Material/mdl/textures/9b4/c7b/36887e6d6a078c97112a8e4ef2d7fcdec729f32.jpg
Material/mdl/textures/9d0/c58/f8c4d00a8d91bed1d760e4a6eaf6ce1c045ef9e.jpg
Material/mdl/textures/9d2/a1b/dbe6285d85a422364ea309f163a93991c522da.jpg
Material/mdl/textures/9f6/95d/3890db0d19e69e94f6f7e14cd902eadec516f6.jpg
Material/mdl/textures/a01/285/47f2d7174956d8da2d621ea6fff02eca4733077c.jpg
Material/mdl/textures/a0e/56b/e45802196391dc8ea5b7537e202b815fd2107de.jpg
Material/mdl/textures/a15/3e5/438adda6d0a5e48784d28c83e6afdbf48519f8f.jpg
Material/mdl/textures/a3f/f6d/7d7148201418dafa049466fd90fb88d1cbaafa6.jpg
Material/mdl/textures/a4e/03f/617b218a431fa60078defdcc6600d8a97fab451.jpg
Material/mdl/textures/a72/3b4/a0a81cb24d6aec0f4da72caaabf4f23e5f2062d.jpg
Material/mdl/textures/a9a/72e/0314dcc1a45ecd3de981fe704d6d746b49185fd.jpg
Material/mdl/textures/a9f/f17/c28ca3fd4144757b910045b6df98221bb61f5a.jpg
Material/mdl/textures/aaa/fd6/93f5dea03fc9c4122c117d3048194293301af2ab.jpg
Material/mdl/textures/ab2/66b/a3945b7803430cf2eb37e4c226ef5a4a0022e806.jpg
Material/mdl/textures/ae2/44b/e9276ea19b31693ace6ef362e6dd502a3933ed8.jpg
Material/mdl/textures/ae4/cae/b23c3374531582a49d2eb01d5bfd482cbcc806.jpg
Material/mdl/textures/af5/d0f/9ddb6cf925aebb5f298c289ce9b2cbd68119674.jpg
Material/mdl/textures/b08/d5a/5c3122c7698e3b79c3b8f6603560129e641e6ee.jpg
Material/mdl/textures/b2f/553/53f3e19f4e4d925628eec7259b9ceb9fee6c0a4.jpg
Material/mdl/textures/b7c/426/42c3af35c17cedceaae59ec33c890bdd74a1daa.jpg
Material/mdl/textures/b92/75d/c4c7b916d2853b4e74b295f53b69e36fad166ca1.jpg
Material/mdl/textures/ba6/207/a2d1bfacc0d62a7963f0c0aba21ed606fd2c266e.jpg
Material/mdl/textures/bb2/e07/2e2b0f0f8caaffba0a2eaaedf954ac808d4232a.jpg
Material/mdl/textures/bd2/9cc/b6ae6a68c9a2ba3dcdba250fe06a7ee143adfda.jpg
Material/mdl/textures/bef/d70/b7a478529b8e358d988ff4dd4106f618d74d243.jpg
Material/mdl/textures/c6e/75b/d52b05aae84908a9e5a9aaf8eb5aebee363dae2.jpg
Material/mdl/textures/c70/1ff/406db4870c3bb4f739845508c3108038a318f92.jpg
Material/mdl/textures/c70/d7c/47d862f414341efa8342e612570729e4a4a2df5.jpg
Material/mdl/textures/c73/48b/4ce4df0f149a9cd8b1d30a3ca570c1c18d2f5a43.jpg
Material/mdl/textures/c78/8c8/aae8f654eb3c6affb443bd03a62a15b2744593c.jpg
Material/mdl/textures/c92/d23/8a1f4c0f954add6a9a7b3746bafb0fe1ca3dc31.jpg
Material/mdl/textures/caa/347/62b2e1babadc33623709e4c3a9e812f4cd3d898.jpg
Material/mdl/textures/caf/c36/55d8e606830b25b7670af42c93d5b148383d03.jpg
Material/mdl/textures/cb0/bb1/b3cf7e6b4f59d78ab633fa2b9f7cfb460cf9c8.jpg
Material/mdl/textures/cc0/88f/db246edf2a1f6972a3ceafeb3264ec7c2e42920.jpg
Material/mdl/textures/cee/499/dce542eca9c9aaa18f9a5651c3ef31b4933d9aa4.jpg
Material/mdl/textures/d09/293/0f5d28d0e6523a0db43b2d7bf0d224017826c09.jpg
Material/mdl/textures/d0a/b75/6aa505e6f23290472b41c931658d16ba061fcf1.jpg
Material/mdl/textures/d24/848/d94870d0e5cb5f5e35f56d67872b56519858432.jpg
Material/mdl/textures/d7f/23e/04efe911f98477988bad7fce1a208908cf8a3ee.jpg
Material/mdl/textures/d8a/470/92d03a55f9bec0f74368d1d210cdfa93b8b2b9.jpg
Material/mdl/textures/dfd/203/e743f37afa0c9b6a7aa9807d74af5c7f6114827.jpg
Material/mdl/textures/e07/a77/f1176b325cd7578f4710c760182178996e6f45.jpg
Material/mdl/textures/e23/505/1fc9ee64a3967243505b62f1362401db2ab789.jpg
Material/mdl/textures/e27/08c/6130bc3406cb7329814fc45e21227877193c00f.jpg
Material/mdl/textures/e2d/ff8/106b900285628539db024119e8835a80a84284a.jpg
Material/mdl/textures/e3c/ce0/3841d436ee4410cf35b3e906f6c7276f4728e71b.jpg
Material/mdl/textures/e49/996/f705ff955ed368014d6f8db89def3553d721015.jpg
Material/mdl/textures/e79/b31/6bce43566c0a6b269ad8fb4e6457698b2720833.jpg
Material/mdl/textures/e80/6aa/ac0ff29df68ab611fcf16e4ab9fb2283bb58a26.jpg
Material/mdl/textures/e87/b26/f73dc62cd4a00e76eb340bb18463e7b8e9224d7.jpg
Material/mdl/textures/ef7/13f/a62fdd5e78076810502c900ef11f2e12e749d45.jpg
Material/mdl/textures/f39/7f5/fbbaa9039f630a79123762ab3b3edb300045f13.jpg
Material/mdl/textures/f3d/b41/f13789861960893b40b35f5a451b2fd6cad856.jpg
Material/mdl/textures/f4c/c46/c259af12caf81a319e7e0072be03619aa8b768.jpg
Material/mdl/textures/f60/bce/0faffda1952ad0ea1368fd3b043374755e27db7.jpg
Material/mdl/textures/f64/7e6/cb9165eacf841f37c86ad1f967db63eed9b0b0.jpg
Material/mdl/textures/f65/a2f/d3c4e2288306dd5e8301d84e8de46263692b441.jpg
Material/mdl/textures/fa8/a6e/3767a3ca9dd71f128d6e52ce87a4bbd0b012492.jpg
```
