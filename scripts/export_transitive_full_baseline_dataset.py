#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path


FINAL_LAYOUT_NAME = (
    "layout.c1_v8_tier2_rollout_transitive_full_dlc_bbox_primary_rmse_observe_v1.usd"
)
_ASCII_REF_RE = re.compile(r"@([^@]+)@")


def _can_import_pxr() -> bool:
    try:
        import pxr  # noqa: F401
    except (ModuleNotFoundError, ImportError, OSError):
        return False
    return True


def _maybe_reexec_with_isaac_python(
    argv: list[str],
    *,
    script_path: Path | None = None,
    wrapper_path: Path | None = None,
) -> int | None:
    try:
        pxr_available = _can_import_pxr()
    except Exception:
        pxr_available = False

    if (
        pxr_available
        or os.environ.get("EXPORT_TRANSITIVE_FULL_BASELINE_UNDER_ISAAC") == "1"
    ):
        return None

    script_path = script_path or Path(__file__).resolve()
    wrapper_path = wrapper_path or script_path.with_name("isaac_python.sh")
    if not wrapper_path.exists():
        return None

    env = os.environ.copy()
    env["EXPORT_TRANSITIVE_FULL_BASELINE_UNDER_ISAAC"] = "1"
    result = subprocess.run(
        [str(wrapper_path), str(script_path), *argv], check=False, env=env
    )
    return result.returncode


def _collect_ascii_layout_asset_refs(path: str | Path) -> set[str]:
    text = Path(path).read_text(encoding="utf-8")
    refs = set()
    for asset in _ASCII_REF_RE.findall(text):
        normalized = _normalize_asset_ref(path, asset)
        if normalized is not None:
            refs.add(normalized)
    return refs


def _normalize_asset_ref(layout_path: str | Path, asset_path: str) -> str | None:
    del layout_path
    normalized = asset_path.strip().replace("\\", "/")
    if not normalized:
        return None
    if normalized.startswith("GRScenes_assets/"):
        return normalized
    marker = "/GRScenes_assets/"
    if marker in normalized:
        return "GRScenes_assets/" + normalized.split(marker, 1)[1]
    return None


def collect_layout_asset_refs(path: str | Path) -> set[str]:
    if not _can_import_pxr():
        return _collect_ascii_layout_asset_refs(path)

    from pxr import Usd

    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f"Failed to open layout stage: {path}")

    refs: set[str] = set()
    for prim in stage.Traverse():
        for spec in prim.GetPrimStack():
            for ref in spec.referenceList.prependedItems:
                asset_path = (ref.assetPath or "").strip()
                normalized = _normalize_asset_ref(path, asset_path)
                if normalized is None and spec.layer is not None:
                    normalized = _normalize_asset_ref(
                        path, spec.layer.ComputeAbsolutePath(asset_path) or ""
                    )
                if normalized is not None:
                    refs.add(normalized)
    return refs


def _iter_final_layouts(dataset_root: Path, rerun_layout_name: str):
    return sorted((dataset_root / "GRScenes100").glob(f"*/*/{rerun_layout_name}"))


def _asset_dir_from_ref(ref: str) -> str:
    parts = Path(ref).parts
    if len(parts) < 3 or parts[0] != "GRScenes_assets":
        raise ValueError(f"Unsupported asset ref: {ref}")
    return Path(*parts[:3]).as_posix()


def _iter_source_asset_dirs(dataset_root: Path):
    source_assets_root = dataset_root / "GRScenes_assets"
    if not source_assets_root.exists():
        return []
    return sorted(path for path in source_assets_root.glob("*/*") if path.is_dir())


def _collect_full_rerun_metrics(full_rerun_root: Path) -> dict:
    stats_files = sorted(full_rerun_root.glob("**/*.stats.json"))
    numeric_totals: dict[str, int | float] = {}

    for stats_path in stats_files:
        try:
            payload = json.loads(stats_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            continue
        if not isinstance(payload, dict):
            continue
        for key, value in payload.items():
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                numeric_totals[key] = numeric_totals.get(key, 0) + value

    return {
        "stats_file_count": len(stats_files),
        "stats_files": [
            path.relative_to(full_rerun_root).as_posix() for path in stats_files
        ],
        "numeric_totals": numeric_totals,
    }


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _write_readme(path: Path, manifest: dict) -> None:
    lines = [
        "# Transitive Full Baseline Delivery Dataset",
        "",
        "This is a slim delivery dataset derived from the transitive-capable full rerun.",
        "",
        "- `GRScenes100/**/layout.usd` is copied from the final rerun layout outputs in the source dataset root.",
        "- `GRScenes_assets/` keeps only asset directories referenced by those exported layouts.",
        "- `Material/` is copied wholesale for this phase.",
        "- Repo-side baseline and rerun roots remain the provenance source of truth.",
        "- Dangling references are recorded, not rewritten away.",
        "",
        "## Export Summary",
        "",
        f"- Exported scenes: {manifest['exported_scene_count']}",
        f"- Retained assets: {manifest['retained_asset_count']}",
        f"- Dangling references: {manifest['dangling_reference_count']}",
        f"- Full rerun stats files scanned: {manifest['full_rerun_metrics']['stats_file_count']}",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def export_transitive_full_baseline_dataset(
    *,
    dataset_root: Path,
    full_rerun_root: Path,
    output_root: Path,
    rerun_layout_name: str = FINAL_LAYOUT_NAME,
    full_rerun_job_id: str | None = None,
) -> None:
    dataset_root = Path(dataset_root)
    full_rerun_root = Path(full_rerun_root)
    output_root = Path(output_root)

    if output_root.exists():
        raise RuntimeError(f"Output root already exists: {output_root}")

    layout_paths = _iter_final_layouts(dataset_root, rerun_layout_name)
    if not layout_paths:
        raise RuntimeError(
            f"No final rerun layouts found under {dataset_root} with name {rerun_layout_name}"
        )

    material_root = dataset_root / "Material"
    if not material_root.exists():
        raise RuntimeError(f"Material root does not exist: {material_root}")

    output_root.mkdir(parents=True)

    all_refs: set[str] = set()
    dangling_refs: set[str] = set()
    for layout_path in layout_paths:
        scene_rel = layout_path.relative_to(dataset_root / "GRScenes100")
        dst_layout = output_root / "GRScenes100" / scene_rel.parent / "layout.usd"
        dst_layout.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(layout_path, dst_layout)
        all_refs.update(collect_layout_asset_refs(layout_path))

    source_asset_dirs = _iter_source_asset_dirs(dataset_root)
    source_asset_dir_map = {
        path.relative_to(dataset_root).as_posix(): path for path in source_asset_dirs
    }

    refs_by_asset_dir: dict[str, set[str]] = defaultdict(set)
    for ref in all_refs:
        refs_by_asset_dir[_asset_dir_from_ref(ref)].add(ref)

    referenced_asset_dirs = set(refs_by_asset_dir)
    retained_asset_dirs: list[str] = []
    retained_asset_dir_set: set[str] = set()
    for asset_dir_rel in sorted(referenced_asset_dirs):
        src_dir = source_asset_dir_map.get(asset_dir_rel)
        if src_dir is None:
            dangling_refs.update(refs_by_asset_dir[asset_dir_rel])
            continue
        resolvable_refs = [
            ref
            for ref in refs_by_asset_dir[asset_dir_rel]
            if (dataset_root / ref).exists()
        ]
        dangling_refs.update(
            ref
            for ref in refs_by_asset_dir[asset_dir_rel]
            if ref not in resolvable_refs
        )
        if not resolvable_refs:
            continue
        dst_dir = output_root / asset_dir_rel
        shutil.copytree(src_dir, dst_dir)
        retained_asset_dirs.append(asset_dir_rel)
        retained_asset_dir_set.add(asset_dir_rel)

    shutil.copytree(material_root, output_root / "Material")

    per_category: dict[str, dict[str, int]] = {}
    for path in source_asset_dirs:
        category = path.relative_to(dataset_root / "GRScenes_assets").parts[0]
        counts = per_category.setdefault(category, {"retained": 0, "omitted": 0})
        rel = path.relative_to(dataset_root).as_posix()
        if rel in retained_asset_dir_set:
            counts["retained"] += 1
        else:
            counts["omitted"] += 1

    full_rerun_metrics = _collect_full_rerun_metrics(full_rerun_root)
    manifest = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "dataset_root": str(dataset_root),
        "full_rerun_root": str(full_rerun_root),
        "full_rerun_job_id": full_rerun_job_id,
        "rerun_layout_name": rerun_layout_name,
        "exported_scene_count": len(layout_paths),
        "retained_asset_count": len(retained_asset_dirs),
        "copied_material_mode": "whole_copy",
        "dangling_reference_count": len(dangling_refs),
        "full_rerun_metrics": full_rerun_metrics,
    }
    summary = {
        "original_asset_count": len(source_asset_dirs),
        "retained_asset_count": len(retained_asset_dirs),
        "omitted_asset_count": len(source_asset_dirs) - len(retained_asset_dirs),
        "per_category": per_category,
        "full_rerun_metrics": full_rerun_metrics,
    }
    dangling = {
        "dangling_reference_count": len(dangling_refs),
        "dangling_references": sorted(dangling_refs),
        "full_rerun_metrics": full_rerun_metrics,
    }

    _write_json(output_root / "MANIFEST.json", manifest)
    _write_json(output_root / "asset_pruning_summary.json", summary)
    _write_json(output_root / "dangling_references.json", dangling)
    _write_readme(output_root / "README.md", manifest)


def main() -> None:
    exit_code = _maybe_reexec_with_isaac_python(sys.argv[1:])
    if exit_code is not None:
        raise SystemExit(exit_code)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--full-rerun-root", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--rerun-layout-name", default=FINAL_LAYOUT_NAME)
    parser.add_argument("--full-rerun-job-id")
    args = parser.parse_args()

    export_transitive_full_baseline_dataset(
        dataset_root=args.dataset_root,
        full_rerun_root=args.full_rerun_root,
        output_root=args.output_root,
        rerun_layout_name=args.rerun_layout_name,
        full_rerun_job_id=args.full_rerun_job_id,
    )


if __name__ == "__main__":
    main()
