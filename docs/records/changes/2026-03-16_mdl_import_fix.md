---
title: MDL 绝对导入修复 (KooPbr → 相对导入)
code_reference: scripts/fix_mdl_absolute_imports.py
created_at: 2026-03-16
updated_at: 2026-03-16
maintainer: claude
status: active
---

# MDL 绝对导入修复 (KooPbr → 相对导入)

## Problem

1,566 MDL files in `GRScenes-test0-rebuilt/Material/mdl/` use absolute imports
(`import ::KooPbr::*`), causing Isaac Sim GUI to render materials as solid red.
This affects approximately 1.2% of assets across 18 categories (electriccooker,
faucet, etc.).

## Root Cause

`KooPbr.mdl` is a custom material module bundled with the dataset, not an
Isaac Sim built-in. Absolute imports (`import ::KooPbr::KooMtl;`) require the
containing directory to be registered in MDL search paths. Isaac Sim's default
search paths do not include `Material/mdl/`, so the imports fail at material
compilation time.

By contrast, `Num*.mdl` files in the same directory already use relative imports
(`using .::OmniUe4Base import *;`) and render correctly.

## Solution

Convert all absolute KooPbr/KooPbr_maps imports to strict relative imports
(`using .::Module import Symbol;`), which resolve from the same directory as the
importing `.mdl` file. This requires no runtime search path configuration.

**11 import patterns replaced** (~3,683 total replacements across 1,566 files):

| Original | Replacement |
|----------|-------------|
| `import ::KooPbr::KooMtl;` | `using .::KooPbr import KooMtl;` |
| `import ::KooPbr::KooTranslucentMtl;` | `using .::KooPbr import KooTranslucentMtl;` |
| `import ::KooPbr::KooLightMtl;` | `using .::KooPbr import KooLightMtl;` |
| `import ::KooPbr::KooMtl2Sided;` | `using .::KooPbr import KooMtl2Sided;` |
| `import ::KooPbr_maps::KooPbr_bitmap;` | `using .::KooPbr_maps import KooPbr_bitmap;` |
| `import ::KooPbr_maps::KooPbr_rgb_output;` | `using .::KooPbr_maps import KooPbr_rgb_output;` |
| `import ::KooPbr_maps::KooPbr_mono_output;` | `using .::KooPbr_maps import KooPbr_mono_output;` |
| `import ::KooPbr_maps::KooPbr_alpha_source;` | `using .::KooPbr_maps import KooPbr_alpha_source;` |
| `import ::KooPbr_maps::KooPbr_bitmap_bump;` | `using .::KooPbr_maps import KooPbr_bitmap_bump;` |
| `import ::KooPbr_maps::KooPbr_falloff;` | `using .::KooPbr_maps import KooPbr_falloff;` |
| `import ::KooPbr_maps::NormalMap_bump;` | `using .::KooPbr_maps import NormalMap_bump;` |

## Scope

- **Target directory**: `GRScenes-test0-rebuilt/Material/mdl/`
- **Files modified**: 1,353 `MI_*.mdl` + 213 hex-hash `.mdl` = 1,566 files
- **Files untouched**: `KooPbr.mdl`, `KooPbr_maps.mdl`, `Num*.mdl`, `OmniUe4*.mdl`
  (these import only MDL standard library modules or already use relative imports)
- **Body code unchanged**: Material body calls (`= KooPbr::KooMtl(...)`) are
  compatible with both `import` and `using` import styles

## Execution

```bash
# Dry-run (report only, no file changes)
python scripts/fix_mdl_absolute_imports.py \
  --mdl-dir GRScenes-test0-rebuilt/Material/mdl \
  --dry-run

# Apply fix
python scripts/fix_mdl_absolute_imports.py \
  --mdl-dir GRScenes-test0-rebuilt/Material/mdl
```

## Verification

```bash
# 1. Confirm no residual absolute KooPbr imports
grep -r "import ::KooPbr" GRScenes-test0-rebuilt/Material/mdl/*.mdl
# Expected: no output

# 2. Confirm relative imports are in place
grep -c "using .::KooPbr" GRScenes-test0-rebuilt/Material/mdl/*.mdl | grep -v ":0$" | wc -l
# Expected: 1,566

# 3. Isaac Sim GUI: open a layout.usd containing affected assets
#    (e.g., electriccooker, faucet) — materials should render correctly
#    without setting MDL_SYSTEM_PATH or carb.settings
```

## Reference

- Proposal and technical analysis: `docs/tmp/mdl-import-fix-proposal.md`
- MDL import semantics: absolute (`import ::M::S;`) requires search path
  registration; strict relative (`using .::M import S;`) resolves from the
  same directory (MDL 1.3+, fully supported by Isaac Sim 4.x)
