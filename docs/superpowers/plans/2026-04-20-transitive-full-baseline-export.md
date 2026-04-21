---
title: Transitive Full Baseline Export Implementation Plan
code_reference:
  - scripts/export_transitive_full_baseline_dataset.py
  - tests/test_export_transitive_full_baseline_dataset.py
  - docs/records/changes/2026-04-20_transitive_full_baseline_export.md
created_at: 2026-04-20
updated_at: 2026-04-20
maintainer: OpenCode
status: approved
doc_class: record
---

# Transitive Full Baseline Export Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a repeatable exporter that derives a slim delivery dataset from the successful transitive full rerun, keeping only scene-referenced assets while copying `Material/` wholesale.

**Architecture:** A single export script reads final rerun layout outputs from the rerun dataset root (`GRScenes100/**/layout.<suffix>.usd`), writes them into a new delivery root as `layout.usd`, computes the referenced asset closure, copies only those `GRScenes_assets`, copies `Material/` whole, and emits manifest/summary files. The full rerun result root is used for metrics/provenance, not as the physical source of scene layouts. Tests use small filesystem fixtures and monkeypatched reference extraction so the export logic is verified without requiring the real 149G dataset.

**Tech Stack:** Python 3, pathlib, json, shutil, pytest, optional pxr/USD for reference extraction in the production script

---

## File Structure

- Create: `scripts/export_transitive_full_baseline_dataset.py`
  Purpose: export final rerun layouts into a new delivery root, copy only referenced assets, copy `Material/` whole, and emit manifest/summary files.
- Create: `tests/test_export_transitive_full_baseline_dataset.py`
  Purpose: verify scene layout selection, asset closure trimming, manifest generation, and dangling reference reporting.
- Create: `docs/records/changes/2026-04-20_transitive_full_baseline_export.md`
  Purpose: record the delivery export feature, commands, and verification results.

### Task 1: Build Fixture-Driven Export Tests

**Files:**
- Create: `tests/test_export_transitive_full_baseline_dataset.py`
- Test: `tests/test_export_transitive_full_baseline_dataset.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_export_transitive_full_baseline_dataset.py` with the fixture helpers and three failing tests below.

```python
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import export_transitive_full_baseline_dataset as mod


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _fixture_roots(tmp_path: Path):
    dataset_root = tmp_path / "dataset"
    rerun_root = tmp_path / "full_rerun"
    output_root = tmp_path / "delivery"

    _write(dataset_root / "Material" / "mdl" / "example.mdl", "mdl")
    _write(
        dataset_root / "GRScenes_assets" / "bottle" / "keep_a" / "usd" / "keep_a.usd",
        "#usda 1.0\n",
    )
    _write(
        dataset_root / "GRScenes_assets" / "bottle" / "keep_b" / "usd" / "keep_b.usd",
        "#usda 1.0\n",
    )
    _write(
        dataset_root / "GRScenes_assets" / "bottle" / "drop_c" / "usd" / "drop_c.usd",
        "#usda 1.0\n",
    )
    _write(dataset_root / "GRScenes100" / "home" / "SCENE1" / "layout.usd", "ORIGINAL")
    _write(dataset_root / "GRScenes100" / "home" / "SCENE2" / "layout.usd", "ORIGINAL")

    suffix = "layout.c1_v8_tier2_rollout_transitive_full_dlc_bbox_primary_rmse_observe_v1.usd"
    _write(dataset_root / "GRScenes100" / "home" / "SCENE1" / suffix, "FINAL_SCENE1")
    _write(dataset_root / "GRScenes100" / "home" / "SCENE2" / suffix, "FINAL_SCENE2")
    _write(rerun_root / "bottle_bbox_primary_rmse_observe_v1" / "01_cert" / "filtered_mapping.stats.json", "{}")
    return dataset_root, rerun_root, output_root


def test_export_uses_final_rerun_layouts_as_layout_usd(tmp_path, monkeypatch):
    dataset_root, rerun_root, output_root = _fixture_roots(tmp_path)
    monkeypatch.setattr(
        mod,
        "collect_layout_asset_refs",
        lambda path: {
            "GRScenes_assets/bottle/keep_a/usd/keep_a.usd",
            "GRScenes_assets/bottle/keep_b/usd/keep_b.usd",
        },
    )

    mod.export_transitive_full_baseline_dataset(
        dataset_root=dataset_root,
        full_rerun_root=rerun_root,
        output_root=output_root,
        rerun_layout_name="layout.c1_v8_tier2_rollout_transitive_full_dlc_bbox_primary_rmse_observe_v1.usd",
        full_rerun_job_id="job-123",
    )

    assert (output_root / "GRScenes100" / "home" / "SCENE1" / "layout.usd").read_text() == "FINAL_SCENE1"
    assert (output_root / "GRScenes100" / "home" / "SCENE2" / "layout.usd").read_text() == "FINAL_SCENE2"


def test_export_keeps_only_referenced_assets(tmp_path, monkeypatch):
    dataset_root, rerun_root, output_root = _fixture_roots(tmp_path)
    refs = {
        dataset_root / "GRScenes100" / "home" / "SCENE1" / "layout.c1_v8_tier2_rollout_transitive_full_dlc_bbox_primary_rmse_observe_v1.usd": {
            "GRScenes_assets/bottle/keep_a/usd/keep_a.usd",
        },
        dataset_root / "GRScenes100" / "home" / "SCENE2" / "layout.c1_v8_tier2_rollout_transitive_full_dlc_bbox_primary_rmse_observe_v1.usd": {
            "GRScenes_assets/bottle/keep_b/usd/keep_b.usd",
        },
    }
    monkeypatch.setattr(mod, "collect_layout_asset_refs", lambda path: refs[Path(path)])

    mod.export_transitive_full_baseline_dataset(
        dataset_root=dataset_root,
        full_rerun_root=rerun_root,
        output_root=output_root,
        rerun_layout_name="layout.c1_v8_tier2_rollout_transitive_full_dlc_bbox_primary_rmse_observe_v1.usd",
        full_rerun_job_id="job-123",
    )

    assert (output_root / "GRScenes_assets" / "bottle" / "keep_a" / "usd" / "keep_a.usd").exists()
    assert (output_root / "GRScenes_assets" / "bottle" / "keep_b" / "usd" / "keep_b.usd").exists()
    assert not (output_root / "GRScenes_assets" / "bottle" / "drop_c").exists()


def test_export_writes_manifest_and_dangling_refs(tmp_path, monkeypatch):
    dataset_root, rerun_root, output_root = _fixture_roots(tmp_path)
    monkeypatch.setattr(
        mod,
        "collect_layout_asset_refs",
        lambda path: {
            "GRScenes_assets/bottle/keep_a/usd/keep_a.usd",
            "GRScenes_assets/bottle/missing_x/usd/missing_x.usd",
        },
    )

    mod.export_transitive_full_baseline_dataset(
        dataset_root=dataset_root,
        full_rerun_root=rerun_root,
        output_root=output_root,
        rerun_layout_name="layout.c1_v8_tier2_rollout_transitive_full_dlc_bbox_primary_rmse_observe_v1.usd",
        full_rerun_job_id="job-123",
    )

    manifest = json.loads((output_root / "MANIFEST.json").read_text())
    dangling = json.loads((output_root / "dangling_references.json").read_text())
    summary = json.loads((output_root / "asset_pruning_summary.json").read_text())
    assert manifest["full_rerun_job_id"] == "job-123"
    assert manifest["dangling_reference_count"] == 1
    assert dangling["dangling_reference_count"] == 1
    assert summary["retained_asset_count"] == 1
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
python -m pytest tests/test_export_transitive_full_baseline_dataset.py -q
```

Expected:

- FAIL because `scripts/export_transitive_full_baseline_dataset.py` does not exist yet.

- [ ] **Step 3: Implement minimal test scaffolding file import**

Create a tiny placeholder `scripts/export_transitive_full_baseline_dataset.py` just to make the failure shift to missing functions.

```python
#!/usr/bin/env python3

def export_transitive_full_baseline_dataset(**kwargs):
    raise NotImplementedError


def collect_layout_asset_refs(path):
    raise NotImplementedError
```

- [ ] **Step 4: Re-run tests to verify the failure is now about missing behavior**

Run:

```bash
python -m pytest tests/test_export_transitive_full_baseline_dataset.py -q
```

Expected:

- FAIL on `NotImplementedError`, proving the tests are targeting the intended behavior.

### Task 2: Implement Scene Export, Asset Closure, And Summary Files

**Files:**
- Create: `scripts/export_transitive_full_baseline_dataset.py`
- Test: `tests/test_export_transitive_full_baseline_dataset.py`

- [ ] **Step 1: Implement the minimal export script**

Replace the placeholder script with the implementation skeleton below.

```python
#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import shutil
from collections import defaultdict
from pathlib import Path


FINAL_LAYOUT_NAME = "layout.c1_v8_tier2_rollout_transitive_full_dlc_bbox_primary_rmse_observe_v1.usd"


def collect_layout_asset_refs(path: str) -> set[str]:
    from pxr import Sdf, Usd

    stage = Usd.Stage.Open(path)
    if stage is None:
        raise RuntimeError(f"Failed to open layout stage: {path}")
    refs = set()
    for prim in stage.Traverse():
        for spec in prim.GetPrimStack():
            for ref in spec.referenceList.prependedItems:
                asset = (ref.assetPath or "").strip()
                if asset.startswith("GRScenes_assets/"):
                    refs.add(asset)
    return refs


def _iter_final_layouts(dataset_root: Path, rerun_layout_name: str):
    return sorted((dataset_root / "GRScenes100").glob(f"*/*/{rerun_layout_name}"))


def export_transitive_full_baseline_dataset(
    *,
    dataset_root: Path,
    full_rerun_root: Path,
    output_root: Path,
    rerun_layout_name: str = FINAL_LAYOUT_NAME,
    full_rerun_job_id: str | None = None,
) -> None:
    if output_root.exists():
        raise RuntimeError(f"Output root already exists: {output_root}")
    output_root.mkdir(parents=True)

    all_refs: set[str] = set()
    dangling: list[str] = []
    scene_count = 0
        for layout in _iter_final_layouts(dataset_root, rerun_layout_name):
            scene_count += 1
            scene_rel = layout.relative_to(dataset_root / "GRScenes100")
            dst = output_root / "GRScenes100" / scene_rel.parent / "layout.usd"
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(layout, dst)
        refs = collect_layout_asset_refs(str(layout))
        all_refs.update(refs)

    retained = 0
    per_category = defaultdict(lambda: {"retained": 0, "omitted": 0})
    source_assets_root = dataset_root / "GRScenes_assets"
    dest_assets_root = output_root / "GRScenes_assets"
    for ref in sorted(all_refs):
        src = dataset_root / ref
        if src.exists():
            dst = output_root / ref
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
            retained += 1
            per_category[Path(ref).parts[1]]["retained"] += 1
        else:
            dangling.append(ref)

    for asset_usd in source_assets_root.glob("*/*/usd/*.usd"):
        category = asset_usd.relative_to(source_assets_root).parts[0]
        rel = asset_usd.relative_to(dataset_root).as_posix()
        if rel not in all_refs:
            per_category[category]["omitted"] += 1

    shutil.copytree(dataset_root / "Material", output_root / "Material")

    manifest = {
        "dataset_root": str(dataset_root),
        "full_rerun_root": str(full_rerun_root),
        "full_rerun_job_id": full_rerun_job_id,
        "exported_scene_count": scene_count,
        "exported_asset_count": retained,
        "dangling_reference_count": len(dangling),
        "material_mode": "whole_copy",
        "rerun_layout_name": rerun_layout_name,
    }
    (output_root / "MANIFEST.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    (output_root / "dangling_references.json").write_text(
        json.dumps({"dangling_reference_count": len(dangling), "references": dangling}, indent=2),
        encoding="utf-8",
    )
    (output_root / "asset_pruning_summary.json").write_text(
        json.dumps(
            {
                "retained_asset_count": retained,
                "omitted_asset_count": sum(v["omitted"] for v in per_category.values()),
                "per_category": per_category,
            },
            indent=2,
            default=dict,
        ),
        encoding="utf-8",
    )
    (output_root / "README.md").write_text(
        "# Transitive Full Baseline Export\n\nThis delivery dataset exposes the final rerun layout outputs as layout.usd.\n",
        encoding="utf-8",
    )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset-root", required=True)
    ap.add_argument("--full-rerun-root", required=True)
    ap.add_argument("--output-root", required=True)
    ap.add_argument("--rerun-layout-name", default=FINAL_LAYOUT_NAME)
    ap.add_argument("--full-rerun-job-id", default=None)
    args = ap.parse_args()
    export_transitive_full_baseline_dataset(
        dataset_root=Path(args.dataset_root),
        full_rerun_root=Path(args.full_rerun_root),
        output_root=Path(args.output_root),
        rerun_layout_name=args.rerun_layout_name,
        full_rerun_job_id=args.full_rerun_job_id,
    )


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run the focused tests to verify they pass**

Run:

```bash
python -m pytest tests/test_export_transitive_full_baseline_dataset.py -q
```

Expected:

- PASS

- [ ] **Step 3: Run a second safety test for existing output root behavior**

Append this extra test to the same file:

```python
def test_export_refuses_existing_output_root(tmp_path):
    dataset_root = tmp_path / "dataset"
    rerun_root = tmp_path / "rerun"
    output_root = tmp_path / "delivery"
    output_root.mkdir()
    with pytest.raises(RuntimeError, match="Output root already exists"):
        mod.export_transitive_full_baseline_dataset(
            dataset_root=dataset_root,
            full_rerun_root=rerun_root,
            output_root=output_root,
        )
```

- [ ] **Step 4: Run the full test file again**

Run:

```bash
python -m pytest tests/test_export_transitive_full_baseline_dataset.py -q
```

Expected:

- PASS

### Task 3: Add Lightweight Real-Data Verification And Delivery Docs

**Files:**
- Modify: `scripts/export_transitive_full_baseline_dataset.py`
- Create: `docs/records/changes/2026-04-20_transitive_full_baseline_export.md`

- [ ] **Step 1: Add summary fields needed for real delivery verification**

Extend the script so `MANIFEST.json` and `asset_pruning_summary.json` include the final rerun metrics and source counts expected by the design.

```python
def _count_source_asset_usds(dataset_root: Path) -> int:
    return sum(1 for _ in (dataset_root / "GRScenes_assets").glob("*/*/usd/*.usd"))


def _load_full_rerun_metrics(full_rerun_root: Path) -> dict:
    candidate_pairs = mapping_pairs = eligible_pairs = 0
    for p in full_rerun_root.glob('*_bbox_primary_rmse_observe_v1/01_cert/filtered_mapping.stats.json'):
        d = json.loads(p.read_text())
        candidate_pairs += d.get('candidate_pairs', 0)
        mapping_pairs += d.get('mapping_pairs', 0)
    for p in full_rerun_root.glob('*_bbox_primary_rmse_observe_v1/01_cert/pair_certificate_summary.json'):
        d = json.loads(p.read_text())
        eligible_pairs += d.get('eligible_pairs', d.get('eligible_count', 0))
    return {
        'candidate_pairs': candidate_pairs,
        'mapping_pairs': mapping_pairs,
        'eligible_pairs': eligible_pairs,
    }
```

Wire those into `MANIFEST.json` and `asset_pruning_summary.json`.

- [ ] **Step 2: Run the focused tests again**

Run:

```bash
python -m pytest tests/test_export_transitive_full_baseline_dataset.py -q
```

Expected:

- PASS

- [ ] **Step 3: Write the change record**

Create `docs/records/changes/2026-04-20_transitive_full_baseline_export.md` with:

```markdown
---
title: "Transitive Full Baseline Export"
code_reference:
  - scripts/export_transitive_full_baseline_dataset.py
created_at: 2026-04-20
updated_at: 2026-04-20
maintainer: OpenCode
status: active
doc_class: record
---

# Transitive Full Baseline Export

## Summary

Added a repeatable exporter that derives a slim delivery dataset from the
transitive-capable full rerun outputs, keeps only scene-referenced assets, and
copies `Material/` as a whole.

## Verification

- `python -m pytest tests/test_export_transitive_full_baseline_dataset.py -q`

## Notes

- delivery `layout.usd` is sourced from the final rerun layout output rather
  than the original `layout.usd`
- dangling references are recorded, not rewritten away, in this phase
```

- [ ] **Step 4: Run doc validation**

Run:

```bash
python scripts/doc_manager.py --validate
python scripts/doc_manager.py --gen-index
```

Expected:

- Both commands succeed.

### Task 4: Run One Real Export Smoke Pass

**Files:**
- Modify: `scripts/export_transitive_full_baseline_dataset.py` (only if smoke test reveals a real bug)

- [ ] **Step 1: Run a real export smoke pass into a temporary output root**

Run:

```bash
python scripts/export_transitive_full_baseline_dataset.py \
  --dataset-root GRScenes-test0-rebuilt-normalize-prededup-transitive-rerun-20260413 \
  --full-rerun-root check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout_transitive_full_dlc_20260415 \
  --output-root /tmp/grscenes-test0-transitive-full-baseline-smoke \
  --full-rerun-job-id dlcve680agoitv7g
```

Expected:

- command exits 0
- smoke output contains `MANIFEST.json`, `asset_pruning_summary.json`, `dangling_references.json`

- [ ] **Step 2: Verify the smoke export shape**

Run:

```bash
python - <<'PY'
import json
from pathlib import Path
root = Path('/tmp/grscenes-test0-transitive-full-baseline-smoke')
print((root / 'MANIFEST.json').exists())
print((root / 'asset_pruning_summary.json').exists())
print((root / 'dangling_references.json').exists())
manifest = json.loads((root / 'MANIFEST.json').read_text())
print(manifest['full_rerun_job_id'])
print(manifest['exported_scene_count'])
PY
```

Expected:

- `True`
- `True`
- `True`
- `dlcve680agoitv7g`
- scene count greater than `0`

- [ ] **Step 3: If smoke export succeeds, stop and report the result before any large copy into `/cpfs/user/zhuzihou/assets`**

Do not perform the final large export to `/cpfs/user/zhuzihou/assets` in the same task batch unless explicitly directed after the smoke result is reviewed.

## Self-Review Checklist

### Spec Coverage

- final rerun layouts exported as `layout.usd`: covered in Task 2
- only referenced `GRScenes_assets` retained: covered in Task 2
- `Material/` copied whole: covered in Task 2
- manifest/summary/dangling outputs: covered in Task 3
- real smoke export: covered in Task 4

### Placeholder Scan

- No `TODO`, `TBD`, or deferred placeholders remain.
- Each task contains concrete commands and code snippets.

### Type And Naming Consistency

- `export_transitive_full_baseline_dataset` is the single top-level export entrypoint.
- `collect_layout_asset_refs` is the single layout-closure hook.
- `FINAL_LAYOUT_NAME` defines the authoritative rerun layout filename used for export.
