---
title: GRScenes Test0 Promoted Clone Implementation Plan
code_reference:
  - scripts/prepare_promoted_clone_workspace.py
  - tests/test_prepare_promoted_clone_workspace.py
  - docs/records/changes/2026-04-21_promoted_clone_workspace.md
  - docs/superpowers/specs/2026-04-21-test0-promoted-clone-design.md
created_at: 2026-04-21
updated_at: 2026-04-21
maintainer: OpenCode
status: active
doc_class: record
---

# GRScenes Test0 Promoted Clone Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a repeatable helper that creates a user-side promoted-clone workspace for GRScenes-test0 by cloning the dataset, seeding only reusable `01_cert` artifacts, and emitting the exact follow-up autorun command for `Step6=apply`.

**Architecture:** Add one focused Python script that prepares the workspace and writes a manifest plus runnable command wrapper, then cover it with fixture-driven tests. After the helper is green, use it once against the real test0 inputs under `/cpfs/user/zhuzihou/dedup_workspaces/` and record the smoke verification in docs.

**Tech Stack:** Python 3, pathlib, json, shutil, pytest

---

## File Structure

- Create: `scripts/prepare_promoted_clone_workspace.py`
  Responsibility: create a fresh workspace root, clone the dataset, seed only `01_cert/` category artifacts into a fresh `c1_bulk/`, and write a manifest plus `run_promoted_clone.sh`.
- Create: `tests/test_prepare_promoted_clone_workspace.py`
  Responsibility: verify workspace creation, `01_cert` seeding, refusal to overwrite existing workspace, and emitted autorun command content.
- Create: `docs/records/changes/2026-04-21_promoted_clone_workspace.md`
  Responsibility: record the new helper, real workspace creation command, and smoke verification results.

### Task 1: Write Failing Tests For Workspace Preparation

**Files:**
- Create: `tests/test_prepare_promoted_clone_workspace.py`
- Test: `tests/test_prepare_promoted_clone_workspace.py`

- [ ] **Step 1: Write the failing test file**

Create `tests/test_prepare_promoted_clone_workspace.py` with this content:

```python
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import prepare_promoted_clone_workspace as mod


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _fixture_roots(tmp_path: Path):
    src_dataset = tmp_path / "src_dataset"
    src_c1_bulk = tmp_path / "src_c1_bulk"
    workspace = tmp_path / "workspace"

    _write(src_dataset / "GRScenes100" / "home" / "SCENE1" / "layout.usd", "LAYOUT")
    _write(src_dataset / "GRScenes_assets" / "book" / "keep" / "usd" / "keep.usd", "#usda 1.0\n")
    _write(src_dataset / "Material" / "mdl" / "example.mdl", "mdl")

    cat_root = src_c1_bulk / "book_bbox_primary_rmse_observe_v1"
    _write(cat_root / "01_cert" / "filtered_mapping.json", json.dumps({
        "GRScenes_assets/book/old/usd/old.usd": "GRScenes_assets/book/keep/usd/keep.usd"
    }, indent=2))
    _write(cat_root / "01_cert" / "filtered_mapping.stats.json", json.dumps({"mapping_pairs": 1}, indent=2))
    _write(cat_root / "01_cert" / "pair_certificates.jsonl", "{}\n")
    _write(cat_root / "01_cert" / "certified_graph.json", json.dumps({"components": 1}, indent=2))
    _write(cat_root / "02_apply" / "stale.json", "stale")

    empty_root = src_c1_bulk / "rug_bbox_primary_rmse_observe_v1"
    _write(empty_root / "01_cert" / "filtered_mapping.stats.json", json.dumps({"mapping_pairs": 0}, indent=2))

    return src_dataset, src_c1_bulk, workspace


def test_prepare_workspace_copies_dataset_and_seeds_only_01_cert(tmp_path):
    src_dataset, src_c1_bulk, workspace = _fixture_roots(tmp_path)

    result = mod.prepare_promoted_clone_workspace(
        source_dataset_root=src_dataset,
        source_c1_bulk_root=src_c1_bulk,
        workspace_root=workspace,
        group_label="test0_transitive_apply_seeded",
    )

    assert (workspace / "dataset" / "GRScenes100" / "home" / "SCENE1" / "layout.usd").exists()
    assert (workspace / "dataset" / "Material" / "mdl" / "example.mdl").exists()
    assert (workspace / "bak").is_dir()
    assert (workspace / "notes").is_dir()
    assert (workspace / "c1_bulk" / "book_bbox_primary_rmse_observe_v1" / "01_cert" / "filtered_mapping.json").exists()
    assert not (workspace / "c1_bulk" / "book_bbox_primary_rmse_observe_v1" / "02_apply").exists()
    assert result["seeded_categories"] == ["book"]


def test_prepare_workspace_writes_manifest_and_run_script(tmp_path):
    src_dataset, src_c1_bulk, workspace = _fixture_roots(tmp_path)

    result = mod.prepare_promoted_clone_workspace(
        source_dataset_root=src_dataset,
        source_c1_bulk_root=src_c1_bulk,
        workspace_root=workspace,
        group_label="test0_transitive_apply_seeded",
    )

    manifest = json.loads((workspace / "workspace_manifest.json").read_text(encoding="utf-8"))
    run_script = (workspace / "run_promoted_clone.sh").read_text(encoding="utf-8")

    assert manifest["group_label"] == "test0_transitive_apply_seeded"
    assert manifest["seeded_category_count"] == 1
    assert "--step6-mode apply" in run_script
    assert "--bbox-gated" in run_script
    assert str(workspace / "dataset") in run_script
    assert str(workspace / "bak") in run_script
    assert str(workspace / "c1_bulk") in run_script
    assert result["run_script_path"] == str(workspace / "run_promoted_clone.sh")


def test_prepare_workspace_refuses_existing_root(tmp_path):
    src_dataset, src_c1_bulk, workspace = _fixture_roots(tmp_path)
    workspace.mkdir()

    try:
        mod.prepare_promoted_clone_workspace(
            source_dataset_root=src_dataset,
            source_c1_bulk_root=src_c1_bulk,
            workspace_root=workspace,
            group_label="test0_transitive_apply_seeded",
        )
    except FileExistsError as exc:
        assert str(workspace) in str(exc)
    else:
        raise AssertionError("expected FileExistsError")
```

- [ ] **Step 2: Run the test file and confirm it fails**

Run:

```bash
python -m pytest tests/test_prepare_promoted_clone_workspace.py -q
```

Expected:

- FAIL with `ModuleNotFoundError` for `prepare_promoted_clone_workspace`

- [ ] **Step 3: Add the minimal import stub**

Create `scripts/prepare_promoted_clone_workspace.py` with this content:

```python
#!/usr/bin/env python3

def prepare_promoted_clone_workspace(**kwargs):
    raise NotImplementedError
```

- [ ] **Step 4: Re-run the test file and confirm the failure shifts**

Run:

```bash
python -m pytest tests/test_prepare_promoted_clone_workspace.py -q
```

Expected:

- FAIL with `NotImplementedError`

### Task 2: Implement The Workspace Preparation Helper

**Files:**
- Create: `scripts/prepare_promoted_clone_workspace.py`
- Test: `tests/test_prepare_promoted_clone_workspace.py`

- [ ] **Step 1: Replace the stub with the implementation skeleton**

Replace `scripts/prepare_promoted_clone_workspace.py` with this implementation:

```python
#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import shlex
import shutil
from pathlib import Path
from typing import Dict, List, Tuple


POLICY_TAG = "bbox_primary_rmse_observe"
OUT_VERSION = "v1"


def _category_name_from_root_name(root_name: str, *, policy_tag: str, out_version: str) -> str:
    suffix = f"_{policy_tag}_{out_version}"
    if not root_name.endswith(suffix):
        raise ValueError(f"unexpected category root: {root_name}")
    return root_name[: -len(suffix)]


def _iter_seedable_cert_dirs(source_c1_bulk_root: Path, *, policy_tag: str, out_version: str) -> List[Tuple[str, Path]]:
    out: List[Tuple[str, Path]] = []
    for category_root in sorted(source_c1_bulk_root.glob(f"*_{policy_tag}_{out_version}")):
        cert_dir = category_root / "01_cert"
        mapping_json = cert_dir / "filtered_mapping.json"
        if not mapping_json.exists():
            continue
        category = _category_name_from_root_name(category_root.name, policy_tag=policy_tag, out_version=out_version)
        out.append((category, cert_dir))
    return out


def _copytree(src: Path, dst: Path) -> None:
    shutil.copytree(src, dst, symlinks=True)


def _build_run_command(*, workspace_root: Path, group_label: str, source_report: Path, mode_reports_dir: Path) -> str:
    dataset_root = workspace_root / "dataset"
    bak_root = workspace_root / "bak"
    c1_bulk_root = workspace_root / "c1_bulk"
    repo_root = Path(__file__).resolve().parents[1]
    cmd = [
        str(repo_root / "scripts" / "isaac_python.sh"),
        str(repo_root / "scripts" / "c1_autorun_categories.py"),
        "--dataset-root", str(dataset_root),
        "--bak-root", str(bak_root),
        "--report", str(source_report),
        "--c1-bulk-dir", str(c1_bulk_root),
        "--group-label", group_label,
        "--bbox-gated",
        "--bbox-policy", POLICY_TAG,
        "--step6-mode", "apply",
        "--dedup-mode", "geom_only",
        "--v-matrix-mode", "auto",
        "--mode-reports-dir", str(mode_reports_dir),
        "--scene-files", "layout.usd",
    ]
    return " ".join(shlex.quote(part) for part in cmd)


def prepare_promoted_clone_workspace(
    *,
    source_dataset_root: Path,
    source_c1_bulk_root: Path,
    workspace_root: Path,
    group_label: str,
    source_report: Path | None = None,
    mode_reports_dir: Path | None = None,
    policy_tag: str = POLICY_TAG,
    out_version: str = OUT_VERSION,
) -> Dict[str, object]:
    source_dataset_root = Path(source_dataset_root).resolve()
    source_c1_bulk_root = Path(source_c1_bulk_root).resolve()
    workspace_root = Path(workspace_root).resolve()
    source_report = (Path(source_report).resolve() if source_report else (source_c1_bulk_root.parents[0] / "v8_prededup" / "union_3way" / "all_categories_union_merged.json"))
    mode_reports_dir = (Path(mode_reports_dir).resolve() if mode_reports_dir else (source_c1_bulk_root.parents[0] / "v8_prededup"))

    if workspace_root.exists():
        raise FileExistsError(f"workspace already exists: {workspace_root}")
    if not source_dataset_root.is_dir():
        raise FileNotFoundError(f"missing source dataset root: {source_dataset_root}")
    if not source_c1_bulk_root.is_dir():
        raise FileNotFoundError(f"missing source c1 bulk root: {source_c1_bulk_root}")

    dataset_dst = workspace_root / "dataset"
    bak_dst = workspace_root / "bak"
    c1_bulk_dst = workspace_root / "c1_bulk"
    notes_dst = workspace_root / "notes"

    _copytree(source_dataset_root, dataset_dst)
    bak_dst.mkdir(parents=True, exist_ok=False)
    c1_bulk_dst.mkdir(parents=True, exist_ok=False)
    notes_dst.mkdir(parents=True, exist_ok=False)

    seeded_categories: List[str] = []
    for category, cert_dir in _iter_seedable_cert_dirs(source_c1_bulk_root, policy_tag=policy_tag, out_version=out_version):
        dst_cert_dir = c1_bulk_dst / f"{category}_{policy_tag}_{out_version}" / "01_cert"
        dst_cert_dir.parent.mkdir(parents=True, exist_ok=True)
        _copytree(cert_dir, dst_cert_dir)
        seeded_categories.append(category)

    run_command = _build_run_command(
        workspace_root=workspace_root,
        group_label=group_label,
        source_report=source_report,
        mode_reports_dir=mode_reports_dir,
    )
    run_script_path = workspace_root / "run_promoted_clone.sh"
    run_script_path.write_text("#!/usr/bin/env bash\nset -euo pipefail\n" + run_command + "\n", encoding="utf-8")
    run_script_path.chmod(0o755)

    manifest = {
        "source_dataset_root": str(source_dataset_root),
        "source_c1_bulk_root": str(source_c1_bulk_root),
        "workspace_root": str(workspace_root),
        "group_label": group_label,
        "policy_tag": policy_tag,
        "out_version": out_version,
        "source_report": str(source_report),
        "mode_reports_dir": str(mode_reports_dir),
        "seeded_categories": seeded_categories,
        "seeded_category_count": len(seeded_categories),
        "run_script_path": str(run_script_path),
    }
    (workspace_root / "workspace_manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return {**manifest, "run_command": run_command}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source-dataset-root", required=True)
    ap.add_argument("--source-c1-bulk-root", required=True)
    ap.add_argument("--workspace-root", required=True)
    ap.add_argument("--group-label", required=True)
    ap.add_argument("--source-report", default=None)
    ap.add_argument("--mode-reports-dir", default=None)
    args = ap.parse_args()

    result = prepare_promoted_clone_workspace(
        source_dataset_root=Path(args.source_dataset_root),
        source_c1_bulk_root=Path(args.source_c1_bulk_root),
        workspace_root=Path(args.workspace_root),
        group_label=args.group_label,
        source_report=Path(args.source_report) if args.source_report else None,
        mode_reports_dir=Path(args.mode_reports_dir) if args.mode_reports_dir else None,
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 2: Run the new test file and make sure it passes**

Run:

```bash
python -m pytest tests/test_prepare_promoted_clone_workspace.py -q
```

Expected:

- PASS with `3 passed`

- [ ] **Step 3: Add one regression test for path content in the run script**

Append this test to `tests/test_prepare_promoted_clone_workspace.py`:

```python
def test_run_script_uses_apply_mode_and_layout_usd_only(tmp_path):
    src_dataset, src_c1_bulk, workspace = _fixture_roots(tmp_path)

    mod.prepare_promoted_clone_workspace(
        source_dataset_root=src_dataset,
        source_c1_bulk_root=src_c1_bulk,
        workspace_root=workspace,
        group_label="test0_transitive_apply_seeded",
    )

    run_script = (workspace / "run_promoted_clone.sh").read_text(encoding="utf-8")
    assert "--step6-mode apply" in run_script
    assert "--scene-files layout.usd" in run_script
    assert "--dedup-mode geom_only" in run_script
```

- [ ] **Step 4: Re-run the test file and confirm all tests pass**

Run:

```bash
python -m pytest tests/test_prepare_promoted_clone_workspace.py -q
```

Expected:

- PASS with `4 passed`

### Task 3: Record The Helper And Create The Real User-Side Workspace

**Files:**
- Create: `docs/records/changes/2026-04-21_promoted_clone_workspace.md`
- Modify: `/cpfs/user/zhuzihou/dedup_workspaces/test0_transitive_apply_<stamp>/...` (runtime output, not repo-tracked)
- Test: `tests/test_prepare_promoted_clone_workspace.py`

- [ ] **Step 1: Write the change record**

Create `docs/records/changes/2026-04-21_promoted_clone_workspace.md` with this structure:

```md
---
title: "Promoted Clone Workspace Preparation"
code_reference:
  - scripts/prepare_promoted_clone_workspace.py
  - tests/test_prepare_promoted_clone_workspace.py
created_at: 2026-04-21
updated_at: 2026-04-21
maintainer: OpenCode
status: active
---

# Promoted Clone Workspace Preparation

## Summary

Added `scripts/prepare_promoted_clone_workspace.py` to create a user-side clean workspace for the GRScenes-test0 promoted-clone pass.

The helper:
- clones the source dataset into `workspace_root/dataset`
- creates `bak/`, `c1_bulk/`, and `notes/`
- seeds only reusable `01_cert/` artifacts from the prior dry-run workspace
- writes `workspace_manifest.json`
- writes `run_promoted_clone.sh` with the exact `c1_autorun_categories.py --step6-mode apply` command

## Verification

```bash
python -m pytest tests/test_prepare_promoted_clone_workspace.py -q
python scripts/prepare_promoted_clone_workspace.py \
  --source-dataset-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup-transitive-rerun-20260413 \
  --source-c1-bulk-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout_transitive_full_dlc_20260415 \
  --workspace-root /cpfs/user/zhuzihou/dedup_workspaces/<stamp> \
  --group-label test0_transitive_apply_seeded
```
```

- [ ] **Step 2: Run tests before the real workspace creation**

Run:

```bash
python -m pytest tests/test_prepare_promoted_clone_workspace.py -q
```

Expected:

- PASS with `4 passed`

- [ ] **Step 3: Create the real user-side workspace**

Run this command with the actual timestamp substituted into `<stamp>`:

```bash
python scripts/prepare_promoted_clone_workspace.py \
  --source-dataset-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup-transitive-rerun-20260413 \
  --source-c1-bulk-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout_transitive_full_dlc_20260415 \
  --workspace-root /cpfs/user/zhuzihou/dedup_workspaces/test0_transitive_apply_<stamp> \
  --group-label test0_transitive_apply_seeded
```

Expected:

- JSON output with `seeded_category_count: 74`
- `workspace_manifest.json` created under the new workspace root
- `run_promoted_clone.sh` created under the new workspace root

- [ ] **Step 4: Smoke-check the real workspace layout**

Run:

```bash
python - <<'PY'
import json
from pathlib import Path
root = Path('/cpfs/user/zhuzihou/dedup_workspaces/test0_transitive_apply_<stamp>')
manifest = json.loads((root / 'workspace_manifest.json').read_text())
print((root / 'dataset' / 'GRScenes100').is_dir())
print((root / 'bak').is_dir())
print((root / 'c1_bulk').is_dir())
print((root / 'notes').is_dir())
print(manifest['seeded_category_count'])
print((root / 'run_promoted_clone.sh').exists())
print((root / 'c1_bulk' / 'book_bbox_primary_rmse_observe_v1' / '01_cert' / 'filtered_mapping.json').exists())
print((root / 'c1_bulk' / 'book_bbox_primary_rmse_observe_v1' / '02_apply').exists())
PY
```

Expected:

- `True`
- `True`
- `True`
- `True`
- `74`
- `True`
- `True`
- `False`

- [ ] **Step 5: Validate docs after recording the change**

Run:

```bash
python scripts/doc_manager.py --validate && python scripts/doc_manager.py --gen-index
```

Expected:

- `All documents look good!`
- `Index generated .../docs/INDEX.md`
