# DLC Normalize Mode Failure Analysis / DLC Normalize 模式故障分析

**Job ID (Failed)**: `dlcp6js3t2v2d71w`
**Job ID (Fix)**: `dlc93ecqxamobs8z`
**Date**: 2026-03-04
**Status**: RESOLVED

---

## 1. Error Description / 错误描述

The DLC job `dlcp6js3t2v2d71w` failed immediately at import time with the following traceback:

```
Running Normalize (specs_normalizer)...
Traceback (most recent call last):
  File "<frozen runpy>", line 198, in _run_module_as_main
  File "<frozen runpy>", line 88, in _run_code
  File "specs_normalizer/__main__.py", line 1, in <module>
    from .normalize import main
  File "specs_normalizer/normalize.py", line 16, in <module>
    from .exporters.assets import export_assets
  File "specs_normalizer/exporters/assets.py", line 17, in <module>
    from ..utils.mdl_rewrite import rewrite_usd_mdl_paths
  File "specs_normalizer/utils/mdl_rewrite.py", line 18, in <module>
    from pxr import Usd, Sdf
ModuleNotFoundError: No module named 'pxr'
```

The command that triggered this failure was:

```bash
python3 -m specs_normalizer --help
```

This is the command that was used by `scripts/dlc/run_task.sh` in `normalize` mode — it called system `python3` directly, NOT the `isaac_python.sh` wrapper that provides the Isaac Sim Python environment.

### Import Chain Explanation / 导入链说明

Each line in the traceback corresponds to one step in the Python module import chain, triggered at process startup before any user code runs:

| Traceback Line | Explanation |
|---|---|
| `specs_normalizer/__main__.py:1` | Entry point for `python3 -m specs_normalizer`; immediately imports `main` from `normalize.py` |
| `specs_normalizer/normalize.py:16` | Top-level import of `export_assets` from `exporters/assets.py` |
| `specs_normalizer/exporters/assets.py:17` | Top-level import of `rewrite_usd_mdl_paths` from `utils/mdl_rewrite.py` |
| `specs_normalizer/utils/mdl_rewrite.py:18` | **`from pxr import Usd, Sdf`** — unconditional top-level import. Python tries to load the `pxr` extension module here and fails. |
| `ModuleNotFoundError: No module named 'pxr'` | System `python3` has no `pxr` on its `sys.path`; the entire import cascade fails. |

Because all imports in this chain are **unconditional and top-level**, the failure happens before `main()` is ever called — even running `python3 -m specs_normalizer --help` would crash.

---

## 2. Root Cause Analysis / 根因分析

### What is `pxr`? / `pxr` 是什么？

`pxr` (also known as `usd-core`) is the **OpenUSD Python bindings** — the official Python API for reading, writing, and manipulating Universal Scene Description (USD) files. It provides key classes such as `Usd.Stage`, `Usd.Prim`, `Sdf.AssetPath`, `Sdf.ValueTypeNames`, `Gf.Vec3f`, `Vt.Vec3fArray`, etc.

In this project's DLC container, `pxr` is **not installed as a standalone pip package**. It is bundled as part of the **Isaac Sim 4.5.0 source installation** located at `/isaac-sim/`. The `pxr` Python package physically resides at:

```
/isaac-sim/kit/python/lib/python3.10/site-packages/pxr/
```

This path is NOT on the system Python's `sys.path` by default.

### Why System `python3` Does Not Have `pxr` / 为什么系统 `python3` 找不到 `pxr`

The DLC container runs an Alibaba Cloud image based on the Isaac Sim Docker image (`isaacsim450-vnc-v8`). The system `python3` interpreter at `/usr/bin/python3` or `/usr/local/bin/python3` is a plain Python installation without any NVIDIA/Omniverse extensions. It cannot find `pxr` because the package is not in any standard system site-packages location.

The Isaac Sim Python environment is accessed through the `isaac_python.sh` wrapper script (`scripts/isaac_python.sh`), which:
1. Locates the Isaac Sim installation (defaulting to `/isaac-sim/`)
2. Finds all `pxr` directories under the `extscache/` subtree and prepends them to `PYTHONPATH`
3. Finds USD shared libraries (`libtf.so`, etc.) and extends `LD_LIBRARY_PATH`
4. Delegates to `/isaac-sim/python.sh` (the Isaac Sim Python runner)

When `run_task.sh` dispatched the `normalize` mode using plain `python3`, none of this environment setup happened, causing the immediate `ModuleNotFoundError`.

### Why `normalize` Was (Incorrectly) Marked as "No Isaac Sim Needed" / 为什么 `normalize` 被错误地标记为"不需要 Isaac Sim"

The `normalize` mode was documented and wired in `run_task.sh` as a "no Isaac Sim needed" task due to the following historical reasons:

1. **Original design intent**: The `specs_normalizer` package was designed as a pure file-copy and JSON export tool that does not interact with Isaac Sim physics APIs (`omni.*`, `isaacsim.*`). On that basis, it appeared not to require Isaac Sim.

2. **Incremental `pxr` dependency**: The `pxr` dependency was added later when `mdl_rewrite.py` was written to rewrite MDL asset paths inside USD files. This change made `pxr` a hard dependency, but the DLC dispatch configuration and documentation were not updated to reflect the new requirement.

3. **Misleading graceful degradation**: The `render_size.py` file wraps its `pxr` import in a `try/except` block, which may have given the false impression that all `pxr` usage in the package was optional. In reality, `mdl_rewrite.py` has an unconditional top-level import that makes `pxr` a hard requirement for the entire package.

---

## 3. Import Dependency Analysis / 依赖链分析

### 3.1 `specs_normalizer` pxr Imports

A full audit of `pxr` imports across the `specs_normalizer` package:

| File | Line(s) | Import Style | Notes |
|---|---|---|---|
| `specs_normalizer/utils/mdl_rewrite.py` | 18 | **Top-level, unconditional** | `from pxr import Usd, Sdf` — **primary failure point**; makes `pxr` a hard dependency for the entire package |
| `specs_normalizer/utils/scene_rewrite.py` | 60 | Inside a function body | `from pxr import Usd, Sdf` inside `rewrite_scene_refs_inplace()`; would fail at call time, not at import time |
| `specs_normalizer/utils/render_size.py` | 17 | `try/except` block | `from pxr import Sdf, Usd` guarded; sets `Sdf = None` and `Usd = None` if import fails — gracefully degraded |
| `specs_normalizer/exporters/scenes.py` | 39, 98 | Inside function bodies | Lines 39: `from pxr import Usd` inside `export_scenes()`; line 98: `from pxr import Sdf` in fallback branch — would fail at call time |

The key insight: `mdl_rewrite.py:18` is an unconditional top-level import. Because `exporters/assets.py` imports `rewrite_usd_mdl_paths` from `mdl_rewrite` unconditionally at the module level, and `normalize.py` imports `export_assets` from `exporters/assets` at the module level, the entire `specs_normalizer` package fails to load without `pxr` — even for `--help`.

### 3.2 `clean_data.py` pxr Imports

The `clean_data.py` entry point (scene splitting / data cleaning) also depends on `pxr` through its primary import:

```
clean_data.py
  └─ import set_physics.pxr_utils.data_clean
      └─ set_physics/pxr_utils/data_clean.py, line 1:
             from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, Vt, UsdShade
```

This is an unconditional top-level import. Running `clean_data.py` with system `python3` would produce the same `ModuleNotFoundError: No module named 'pxr'`. The `clean` mode in `run_task.sh` had the same underlying bug.

---

## 4. Isaac Sim Python Environment Analysis / Isaac Sim Python 环境分析

### How `isaac_python.sh` Sets Up the Python Environment

The `scripts/isaac_python.sh` wrapper performs the following environment setup before delegating to the Isaac Sim Python interpreter:

1. **Locate Isaac Sim**: Searches in order:
   - `$ISAAC_SIM_ROOT` environment variable (if set)
   - `/isaac-sim/` (standard DLC Docker container path)
   - `~/.local/share/ov/pkg/isaac_sim-*` (user-local installs)
   - `/opt/nvidia/isaac-sim`, `/opt/NVIDIA/isaac-sim`, `/opt/omniverse/isaac-sim`

2. **Configure `PYTHONPATH`**: Finds all `pxr/` directories under `${ISAAC_ROOT}/extscache/` (up to 4 levels deep) and prepends their parent directories to `PYTHONPATH`. This makes `pxr` importable.

3. **Configure `LD_LIBRARY_PATH`**: Locates USD shared libraries (`libtf.so` inside `omni.usd.libs/`) and adds relevant lib directories (`kit/lib`, `kit/plugins`) to `LD_LIBRARY_PATH`.

4. **Execute**: Delegates to `${ISAAC_ROOT}/python.sh "$@"`, which is the Isaac Sim Python runner that invokes the correct Python 3.10 interpreter.

### Where `pxr` Lives in the Container

```
/isaac-sim/
  kit/
    python/
      lib/
        python3.10/
          site-packages/
            pxr/          ← pxr Python bindings
              __init__.py
              Usd.so
              Sdf.so
              Gf.so
              Vt.so
              UsdGeom.so
              UsdPhysics.so
              UsdShade.so
              ...
```

The system `python3` in a clean DLC shell does NOT have `/isaac-sim/kit/python/lib/python3.10/site-packages` on its `sys.path`. Only the `isaac_python.sh` → `/isaac-sim/python.sh` execution path provides access to `pxr`.

### Environment Comparison

| Property | System `python3` | `isaac_python.sh` |
|---|---|---|
| Python binary | `/usr/bin/python3` (or similar) | `/isaac-sim/python.sh` → Isaac Sim Python 3.10 |
| `pxr` available | **No** — not in system site-packages | **Yes** — added to `PYTHONPATH` by the wrapper |
| `omni.*` modules | No | Yes (Isaac Sim extensions) |
| `LD_LIBRARY_PATH` | Standard system libs only | Extended with USD shared libs |
| EULA accepted | N/A | `OMNI_KIT_ACCEPT_EULA=YES` set by `run_task.sh` |

---

## 5. Impact Assessment / 影响评估

### Modes Affected

Both `normalize` and `clean` modes in `run_task.sh` were broken:

| Mode | Bug | Impact |
|---|---|---|
| `normalize` | Called `python3 -m specs_normalizer` | All normalize DLC jobs fail at startup with `ModuleNotFoundError: No module named 'pxr'` |
| `clean` | Called `python3 "$CODE_ROOT/clean_data.py"` | All clean/scene-splitting DLC jobs fail at startup with the same error |

All other modes (`interaction`, `navigation`, `simready`, `prep_root_scene`, `custom`, `batch`) already correctly used `isaac_python.sh` and were unaffected.

### Documentation Affected

Approximately **4 files and ~15 locations** contained incorrect claims that `normalize` mode "does not require Isaac Sim":

| File | Type of Error |
|---|---|
| `scripts/dlc/run_task.sh` | Wrong executor (`python3` instead of `isaac_python.sh`) for `normalize` and `clean` |
| `docs/dlc/README.md` | Section 5.5: stated normalize "does not need Isaac Sim"; call chain diagram was incorrect; dependency table was incorrect |
| `CLAUDE.md` | Modes table listed `normalize` as not needing Isaac Sim |
| `.claude/agents/dlc-operator.md` | Call chain and CLI command sections showed wrong invocations |

---

## 6. Fix Summary / 修复汇总

### Files Changed

| File | Changes Made |
|---|---|
| `scripts/dlc/run_task.sh` | `normalize` mode: `python3 -m specs_normalizer` → `"$ISAAC_PYTHON" -m specs_normalizer`; `clean` mode: `python3 "$CODE_ROOT/clean_data.py"` → `"$ISAAC_PYTHON" "$CODE_ROOT/clean_data.py"`; added Isaac Sim requirement comments |
| `docs/dlc/README.md` | Section 5.5: updated to say "Requires Isaac Sim Python"; updated underlying call; Section 5.6: updated similarly; call chain diagram corrected; dependency table updated (~9 locations total) |
| `CLAUDE.md` | Modes table corrected; normalize and clean both now listed as requiring Isaac Sim Python |
| `.claude/agents/dlc-operator.md` | Call chain updated to show `isaac_python.sh` for normalize and clean; CLI commands section updated with correct invocations and Isaac Sim notes |

### Specific Code Change in `run_task.sh`

**Before (broken)**:
```bash
elif [ "$1" == "normalize" ]; then
    shift
    echo "Running Normalize (specs_normalizer)..."
    python3 -m specs_normalizer "$@"

elif [ "$1" == "clean" ]; then
    shift
    echo "Running Clean (scene splitting)..."
    python3 "$CODE_ROOT/clean_data.py" "$@"
```

**After (fixed)**:
```bash
elif [ "$1" == "normalize" ]; then
    # 注意: 需要 Isaac Sim Python (pxr 依赖) / Requires Isaac Sim Python (pxr dependency)
    shift
    echo "Running Normalize (specs_normalizer)..."
    "$ISAAC_PYTHON" -m specs_normalizer "$@"

elif [ "$1" == "clean" ]; then
    # 注意: 需要 Isaac Sim Python (pxr 依赖) / Requires Isaac Sim Python (pxr dependency)
    shift
    echo "Running Clean (scene splitting)..."
    "$ISAAC_PYTHON" "$CODE_ROOT/clean_data.py" "$@"
```

---

## 7. Verification / 验证结果

### Local Tests

Before resubmitting, the following local validation steps were performed:

- [x] Shell syntax check: `bash -n scripts/dlc/run_task.sh` — passed
- [x] Mode dispatch review: all 7 named modes and the batch default confirmed correct
- [x] `isaac_python.sh` present and executable in expected location
- [x] `ISAAC_PYTHON` variable referencing correct path
- [x] All documentation updated and consistent with actual script behavior

### Resubmitted DLC Job

A new DLC test job was submitted with the corrected `run_task.sh`:

- **Job ID**: `dlc93ecqxamobs8z`
- **Mode tested**: `normalize`
- **Result**: The job started successfully; `pxr` was importable; `specs_normalizer` loaded without errors

---

## 8. Lessons Learned / 经验总结

1. **All pipeline scripts in this project depend on `pxr` (OpenUSD)**. There is NO mode in `run_task.sh` that can run without the Isaac Sim Python environment in the DLC container. Even "pure file processing" tools like `specs_normalizer` transitively depend on `pxr` through their USD manipulation utilities.

2. **`pxr` is a transitive dependency, not just a direct one**. Even if a top-level script does not directly import `pxr`, any utility module it imports (like `mdl_rewrite.py`) may have an unconditional top-level `pxr` import that makes the entire package unusable without it. Always trace the full import dependency tree before deciding whether Isaac Sim Python is needed.

3. **The assumption that "pure Python" scripts don't need Isaac Sim was wrong** in this context. "Pure Python" refers to not using Isaac Sim's extension APIs (`omni.*`, `isaacsim.*`), but the project's data processing tools use `pxr` (OpenUSD) extensively for USD file manipulation. In the DLC container, `pxr` is only available through Isaac Sim — so all scripts that use USD are Isaac Sim Python scripts.

4. **When adding new DLC modes, always trace import dependencies** to verify whether Isaac Sim Python is required. The rule of thumb for this project: if the script or any of its imports touch any file with `from pxr import ...` or `import pxr`, it needs `isaac_python.sh`.

5. **Graceful degradation (`try/except` around `pxr` imports) does not mean the package is `pxr`-free**. The presence of guarded imports in some files (e.g., `render_size.py`) can give a misleading picture. Always check for unconditional top-level imports in transitive dependencies.

6. **Documentation and dispatch scripts must be updated together**. When a new dependency is added to a module, the following should be updated atomically: the module docstring, the DLC dispatch script (`run_task.sh`), all user-facing documentation, and any agent memory/context files.

---

## Appendix: Full Import Trace / 附录：完整导入路径

```
python3 -m specs_normalizer
  ├─ specs_normalizer/__main__.py
  │   └─ from .normalize import main
  │       └─ specs_normalizer/normalize.py
  │           ├─ from .exporters.assets import export_assets    ← triggers failure
  │           │   └─ specs_normalizer/exporters/assets.py
  │           │       └─ from ..utils.mdl_rewrite import rewrite_usd_mdl_paths
  │           │           └─ specs_normalizer/utils/mdl_rewrite.py
  │           │               └─ from pxr import Usd, Sdf      ← ModuleNotFoundError
  │           ├─ from .exporters.materials import export_materials  (never reached)
  │           └─ from .exporters.scenes import export_scenes        (never reached)
  │
  └─ (Additional pxr usages — only reached if import succeeds)
      ├─ specs_normalizer/utils/scene_rewrite.py:60
      │   └─ from pxr import Usd, Sdf  (inside function)
      ├─ specs_normalizer/utils/render_size.py:17
      │   └─ try: from pxr import Sdf, Usd  (guarded, degrades gracefully)
      └─ specs_normalizer/exporters/scenes.py:39,98
          └─ from pxr import Usd / Sdf  (inside functions)

python3 clean_data.py
  └─ (project root) clean_data.py
      └─ from set_physics.pxr_utils.data_clean import ...
          └─ set_physics/pxr_utils/data_clean.py, line 1:
              └─ from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, Vt, UsdShade
                                                                    ← ModuleNotFoundError
```
