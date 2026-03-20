#!/usr/bin/env python3
"""Rebuild GRScenes-test0 from legacy home/commercial packages.

This script orchestrates the restore plan for rebuilding a clean GRScenes-test0
from the external legacy dataset root:

  /cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100

It is intentionally split into explicit stages:

1. ``submit-dlc`` submits two DLC jobs in parallel, one for ``home_scenes`` and
   one for ``commercial_scenes``. Each job runs ``specs_normalizer`` into its
   own temporary output root to avoid write races.
2. ``merge`` merges the two temporary outputs into one final dataset root,
   verifying that any overlapping files are byte-identical.
3. ``validate`` checks the merged output against the legacy source inventory and
   spot-checks that exported layouts can be opened with pxr when available.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    from pxr import Usd
except Exception:
    Usd = None  # type: ignore[assignment]

from specs_normalizer.utils.scene_rewrite import rewrite_scene_refs_inplace


def _utc_run_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def _abs_path(path: str) -> Path:
    return Path(path).expanduser().resolve()


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _ensure_empty_or_absent(path: Path, label: str) -> None:
    if not path.exists():
        return
    if path.is_dir() and not any(path.iterdir()):
        return
    raise RuntimeError(f"{label} already exists and is not empty: {path}")


def _iter_files(root: Path) -> Iterable[Path]:
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames.sort()
        filenames.sort()
        base = Path(dirpath)
        for name in filenames:
            yield base / name


def _iter_scene_usd_files(dataset_root: Path) -> Iterable[Path]:
    scenes_root = dataset_root / "GRScenes100"
    if not scenes_root.is_dir():
        return
    for category_dir in sorted(path for path in scenes_root.iterdir() if path.is_dir()):
        for scene_dir in sorted(path for path in category_dir.iterdir() if path.is_dir()):
            for usd_path in sorted(scene_dir.iterdir()):
                if usd_path.is_file() and usd_path.suffix.lower() in {".usd", ".usda", ".usdc"}:
                    yield usd_path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _scene_dir_count(path: Path) -> int:
    if not path.is_dir():
        return 0
    return sum(1 for child in path.iterdir() if child.is_dir())


def _scan_legacy_model_refs_in_usd(usd_path: Path) -> Tuple[int, List[str]]:
    if Usd is None:
        return 0, []

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        return 0, []

    legacy_refs: List[str] = []
    for prim in stage.Traverse():
        for metadata_name in ("references", "payload"):
            entries = prim.GetMetadata(metadata_name)
            if entries is None or not hasattr(entries, "GetAddedOrExplicitItems"):
                continue
            for item in entries.GetAddedOrExplicitItems():
                asset_path = getattr(item, "assetPath", "") or ""
                if asset_path.startswith("models/"):
                    legacy_refs.append(asset_path)
    return len(legacy_refs), legacy_refs[:10]


def _repair_scene_refs_under_root(dataset_root: Path) -> Dict[str, Any]:
    materials_root = dataset_root / "Material" / "mdl"
    assets_root = dataset_root / "GRScenes_assets"
    if not materials_root.is_dir():
        raise FileNotFoundError(f"Missing materials root: {materials_root}")
    if not assets_root.is_dir():
        raise FileNotFoundError(f"Missing assets root: {assets_root}")

    scene_files = list(_iter_scene_usd_files(dataset_root))
    summary: Dict[str, Any] = {
        "dataset_root": str(dataset_root),
        "scene_file_count": len(scene_files),
        "files_changed": 0,
        "rewrite_count": 0,
        "remaining_legacy_model_ref_files": [],
    }

    for usd_path in scene_files:
        changed = rewrite_scene_refs_inplace(
            str(usd_path),
            str(materials_root),
            str(assets_root),
            relative_base=str(usd_path.parent),
        )
        if changed:
            summary["files_changed"] += 1
            summary["rewrite_count"] += changed

    remaining_files: List[Dict[str, Any]] = []
    remaining_count = 0
    if Usd is not None:
        for usd_path in scene_files:
            legacy_ref_count, preview = _scan_legacy_model_refs_in_usd(usd_path)
            if legacy_ref_count:
                remaining_count += legacy_ref_count
                remaining_files.append(
                    {
                        "path": str(usd_path),
                        "legacy_model_ref_count": legacy_ref_count,
                        "legacy_model_ref_preview": preview,
                    }
                )

    summary["remaining_legacy_model_ref_count"] = remaining_count
    summary["remaining_legacy_model_ref_files"] = remaining_files[:50]
    summary["status"] = "ok" if remaining_count == 0 else "failed_remaining_legacy_refs"
    return summary


def _build_default_paths(out_root: Path) -> Dict[str, Path]:
    parent = out_root.parent
    stem = out_root.name
    return {
        "home_tmp_root": parent / f"{stem}_home_tmp",
        "commercial_tmp_root": parent / f"{stem}_commercial_tmp",
    }


def _legacy_package_root(legacy_root: Path, split_name: str) -> Path:
    return legacy_root / split_name


def _collect_legacy_inventory(legacy_root: Path) -> Dict[str, Any]:
    home_root = _legacy_package_root(legacy_root, "home_scenes")
    commercial_root = _legacy_package_root(legacy_root, "commercial_scenes")
    for label, root in (("home_scenes", home_root), ("commercial_scenes", commercial_root)):
        for child in ("Materials", "models", "scenes"):
            path = root / child
            if not path.is_dir():
                raise FileNotFoundError(f"Legacy {label} missing directory: {path}")

    home_count = _scene_dir_count(home_root / "scenes")
    commercial_count = _scene_dir_count(commercial_root / "scenes")
    inventory = {
        "legacy_root": str(legacy_root),
        "home_source_root": str(home_root),
        "commercial_source_root": str(commercial_root),
        "home_scene_count": home_count,
        "commercial_scene_count": commercial_count,
        "total_scene_count": home_count + commercial_count,
    }
    return inventory


def _build_normalize_command_args(
    *,
    src_target: Path,
    dst_root: Path,
    scene_category: str,
) -> str:
    cmd = [
        "normalize",
        "--src-target",
        str(src_target),
        "--dst-root",
        str(dst_root),
        "--asset-name",
        "GRScenes_assets",
        "--scene-name",
        "GRScenes100",
        "--scene-category",
        scene_category,
        "--with-annotations",
    ]
    return shlex.join(cmd)


@dataclass(frozen=True)
class JobSpec:
    label: str
    source_root: Path
    tmp_root: Path
    scene_category: str
    task_name: str
    command_args: str


def _build_job_specs(
    *,
    legacy_root: Path,
    out_root: Path,
    run_id: str,
) -> List[JobSpec]:
    defaults = _build_default_paths(out_root)
    home_src = _legacy_package_root(legacy_root, "home_scenes")
    commercial_src = _legacy_package_root(legacy_root, "commercial_scenes")
    return [
        JobSpec(
            label="home",
            source_root=home_src,
            tmp_root=defaults["home_tmp_root"],
            scene_category="home",
            task_name=f"rebuild_test0_{run_id}_home",
            command_args=_build_normalize_command_args(
                src_target=home_src,
                dst_root=defaults["home_tmp_root"],
                scene_category="home",
            ),
        ),
        JobSpec(
            label="commercial",
            source_root=commercial_src,
            tmp_root=defaults["commercial_tmp_root"],
            scene_category="commercial",
            task_name=f"rebuild_test0_{run_id}_commercial",
            command_args=_build_normalize_command_args(
                src_target=commercial_src,
                dst_root=defaults["commercial_tmp_root"],
                scene_category="commercial",
            ),
        ),
    ]


def _manifest_path(report_root: Path) -> Path:
    return report_root / "run_manifest.json"


def _build_manifest(
    *,
    legacy_root: Path,
    out_root: Path,
    report_root: Path,
    run_id: str,
    inventory: Dict[str, Any],
    jobs: Sequence[JobSpec],
    data_sources: Optional[str],
) -> Dict[str, Any]:
    return {
        "run_id": run_id,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "legacy_root": str(legacy_root),
        "out_root": str(out_root),
        "report_root": str(report_root),
        "inventory": inventory,
        "data_sources": data_sources,
        "jobs": [
            {
                "label": job.label,
                "scene_category": job.scene_category,
                "source_root": str(job.source_root),
                "tmp_root": str(job.tmp_root),
                "task_name": job.task_name,
                "job_name": f"{job.task_name}_0_1",
                "command_args": job.command_args,
            }
            for job in jobs
        ],
        "stages": {},
    }


def _submit_job(
    *,
    repo_root: Path,
    job: JobSpec,
    data_sources: Optional[str],
    dry_run: bool,
) -> Dict[str, Any]:
    launch_script = repo_root / "scripts" / "dlc" / "launch_job.sh"
    if not launch_script.is_file():
        raise FileNotFoundError(f"launch script not found: {launch_script}")

    cmd = ["bash", str(launch_script), job.task_name, "0", "1", data_sources or "", job.command_args]
    if dry_run:
        return {
            "status": "dry_run",
            "command": cmd,
            "job_name": f"{job.task_name}_0_1",
        }

    subprocess.run(cmd, check=True, cwd=str(repo_root))
    return {
        "status": "submitted",
        "command": cmd,
        "job_name": f"{job.task_name}_0_1",
    }


def cmd_submit_dlc(args: argparse.Namespace) -> int:
    repo_root = Path(__file__).resolve().parent.parent
    legacy_root = _abs_path(args.legacy_root)
    out_root = _abs_path(args.out_root)
    run_id = args.run_id or _utc_run_id()
    report_root = _abs_path(args.report_root or str(repo_root / "check_reports" / "rebuild_test0" / run_id))

    inventory = _collect_legacy_inventory(legacy_root)
    if inventory["home_scene_count"] != 69 or inventory["commercial_scene_count"] != 30:
        raise RuntimeError(
            "Legacy scene inventory mismatch: "
            f"observed home={inventory['home_scene_count']} commercial={inventory['commercial_scene_count']}"
        )

    _ensure_empty_or_absent(out_root, "Final output root")
    defaults = _build_default_paths(out_root)
    _ensure_empty_or_absent(defaults["home_tmp_root"], "Home temp root")
    _ensure_empty_or_absent(defaults["commercial_tmp_root"], "Commercial temp root")
    _ensure_empty_or_absent(report_root, "Report root")

    jobs = _build_job_specs(legacy_root=legacy_root, out_root=out_root, run_id=run_id)
    manifest = _build_manifest(
        legacy_root=legacy_root,
        out_root=out_root,
        report_root=report_root,
        run_id=run_id,
        inventory=inventory,
        jobs=jobs,
        data_sources=args.data_sources,
    )

    report_root.mkdir(parents=True, exist_ok=False)
    submissions: Dict[str, Any] = {}
    for job in jobs:
        submissions[job.label] = _submit_job(
            repo_root=repo_root,
            job=job,
            data_sources=args.data_sources,
            dry_run=args.dry_run,
        )

    manifest["stages"]["submit_dlc"] = {
        "status": "dry_run" if args.dry_run else "submitted",
        "submissions": submissions,
    }
    _write_json(_manifest_path(report_root), manifest)

    print(f"Manifest: {_manifest_path(report_root)}")
    for label, info in submissions.items():
        print(f"{label}: {info['status']} -> {' '.join(info['command'])}")
    print("Next step after both DLC jobs finish: run `merge` and then `validate`.")
    return 0


def _copy_with_conflict_check(
    *,
    src_root: Path,
    dst_root: Path,
    skip_relpaths: Optional[set[str]] = None,
) -> Dict[str, Any]:
    copied = 0
    identical = 0
    conflicts: List[Dict[str, str]] = []
    skip_relpaths = skip_relpaths or set()

    for src_path in _iter_files(src_root):
        rel_path = src_path.relative_to(src_root).as_posix()
        if rel_path in skip_relpaths:
            continue

        dst_path = dst_root / rel_path
        if not dst_path.exists():
            dst_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src_path, dst_path)
            copied += 1
            continue

        if not dst_path.is_file():
            conflicts.append(
                {
                    "path": rel_path,
                    "reason": "destination_exists_but_is_not_file",
                }
            )
            continue

        if _sha256(src_path) == _sha256(dst_path):
            identical += 1
            continue

        conflicts.append(
            {
                "path": rel_path,
                "reason": "content_mismatch",
                "src": str(src_path),
                "dst": str(dst_path),
            }
        )

    return {
        "copied": copied,
        "identical": identical,
        "conflicts": conflicts,
    }


def _regenerate_asset_annotation(out_root: Path) -> Dict[str, Any]:
    assets_root = out_root / "GRScenes_assets"
    payload: Dict[str, Any] = {
        "asset_name": "GRScenes_assets",
        "total_count": 0,
        "categories": {},
    }
    if not assets_root.is_dir():
        raise FileNotFoundError(f"Missing assets root: {assets_root}")

    total = 0
    for category_dir in sorted(path for path in assets_root.iterdir() if path.is_dir()):
        uids = sorted(child.name for child in category_dir.iterdir() if child.is_dir())
        payload["categories"][category_dir.name] = {
            "count": len(uids),
            "uids": uids,
        }
        total += len(uids)
    payload["total_count"] = total

    target = assets_root / "Asset_annotation.json"
    _write_json(target, payload)
    return {
        "path": str(target),
        "category_count": len(payload["categories"]),
        "total_count": total,
    }


def cmd_merge(args: argparse.Namespace) -> int:
    manifest = _load_json(_abs_path(args.manifest))
    out_root = _abs_path(manifest["out_root"])
    report_root = _abs_path(manifest["report_root"])
    home_tmp = _abs_path(next(job["tmp_root"] for job in manifest["jobs"] if job["label"] == "home"))
    commercial_tmp = _abs_path(next(job["tmp_root"] for job in manifest["jobs"] if job["label"] == "commercial"))

    _ensure_empty_or_absent(out_root, "Final output root")
    for label, path in (("home tmp", home_tmp), ("commercial tmp", commercial_tmp)):
        if not path.is_dir():
            raise FileNotFoundError(f"Missing {label} root: {path}")

    repair_summary = {
        "home": _repair_scene_refs_under_root(home_tmp),
        "commercial": _repair_scene_refs_under_root(commercial_tmp),
    }
    _write_json(report_root / "repair_temp_refs_summary.json", repair_summary)
    for label, result in repair_summary.items():
        if result["status"] != "ok":
            raise RuntimeError(
                f"Temp root scene ref repair failed for {label}; see {report_root / 'repair_temp_refs_summary.json'}"
            )

    out_root.mkdir(parents=True, exist_ok=False)
    skip_relpaths = {"GRScenes_assets/Asset_annotation.json"}
    summary = {
        "out_root": str(out_root),
        "sources": {
            "home_tmp": str(home_tmp),
            "commercial_tmp": str(commercial_tmp),
        },
        "skip_relpaths": sorted(skip_relpaths),
        "repair_temp_refs_summary_path": str(report_root / "repair_temp_refs_summary.json"),
        "passes": {},
    }

    for label, src_root in (("home", home_tmp), ("commercial", commercial_tmp)):
        result = _copy_with_conflict_check(src_root=src_root, dst_root=out_root, skip_relpaths=skip_relpaths)
        summary["passes"][label] = {
            "copied": result["copied"],
            "identical": result["identical"],
            "conflict_count": len(result["conflicts"]),
            "conflicts_preview": result["conflicts"][:50],
        }
        if result["conflicts"]:
            summary["status"] = "failed_conflicts"
            _write_json(report_root / "merge_summary.json", summary)
            raise RuntimeError(f"Merge conflicts detected during {label} pass; see merge_summary.json")

    annotation = _regenerate_asset_annotation(out_root)
    summary["asset_annotation"] = annotation
    summary["status"] = "ok"
    _write_json(report_root / "merge_summary.json", summary)

    manifest.setdefault("stages", {})["merge"] = {
        "status": "completed",
        "summary_path": str(report_root / "merge_summary.json"),
        "repair_summary_path": str(report_root / "repair_temp_refs_summary.json"),
        "out_root": str(out_root),
    }
    _write_json(_manifest_path(report_root), manifest)

    print(f"Merged dataset: {out_root}")
    print(f"Merge summary: {report_root / 'merge_summary.json'}")
    return 0


def _validate_asset_layout(out_root: Path) -> Dict[str, Any]:
    assets_root = out_root / "GRScenes_assets"
    bad_assets: List[str] = []
    asset_count = 0

    for category_dir in sorted(path for path in assets_root.iterdir() if path.is_dir()):
        for uid_dir in sorted(path for path in category_dir.iterdir() if path.is_dir()):
            uid = uid_dir.name
            asset_count += 1
            usd_path = uid_dir / "usd" / f"{uid}.usd"
            if not usd_path.is_file():
                bad_assets.append(uid_dir.as_posix())

    return {
        "asset_dir_count": asset_count,
        "bad_asset_count": len(bad_assets),
        "bad_assets_preview": bad_assets[:50],
    }


def _validate_layout_counts(out_root: Path) -> Dict[str, Any]:
    scenes_root = out_root / "GRScenes100"
    home = sum(1 for p in (scenes_root / "home").glob("*/layout.usd"))
    commercial = sum(1 for p in (scenes_root / "commercial").glob("*/layout.usd"))
    return {
        "home_layout_count": home,
        "commercial_layout_count": commercial,
        "total_layout_count": home + commercial,
    }


def _validate_layout_openability(out_root: Path, limit: Optional[int]) -> Dict[str, Any]:
    if Usd is None:
        return {
            "enabled": False,
            "reason": "pxr.Usd unavailable",
        }

    layouts = sorted((out_root / "GRScenes100").glob("*/*/layout.usd"))
    if limit is not None:
        layouts = layouts[:limit]

    failures: List[str] = []
    for layout in layouts:
        stage = Usd.Stage.Open(str(layout))
        if stage is None:
            failures.append(layout.as_posix())

    return {
        "enabled": True,
        "checked_count": len(layouts),
        "failure_count": len(failures),
        "failures_preview": failures[:50],
    }


def _validate_layout_refs(out_root: Path, limit: Optional[int]) -> Dict[str, Any]:
    if Usd is None:
        return {
            "enabled": False,
            "reason": "pxr.Usd unavailable",
        }

    layouts = sorted((out_root / "GRScenes100").glob("*/*/layout.usd"))
    if limit is not None:
        layouts = layouts[:limit]

    bad_layouts: List[Dict[str, Any]] = []
    total_legacy_refs = 0
    for layout in layouts:
        legacy_ref_count, preview = _scan_legacy_model_refs_in_usd(layout)
        if legacy_ref_count:
            total_legacy_refs += legacy_ref_count
            bad_layouts.append(
                {
                    "path": str(layout),
                    "legacy_model_ref_count": legacy_ref_count,
                    "legacy_model_ref_preview": preview,
                }
            )

    return {
        "enabled": True,
        "checked_count": len(layouts),
        "legacy_model_ref_count": total_legacy_refs,
        "bad_layout_count": len(bad_layouts),
        "bad_layouts_preview": bad_layouts[:50],
    }


def cmd_validate(args: argparse.Namespace) -> int:
    manifest = _load_json(_abs_path(args.manifest))
    legacy_root = _abs_path(manifest["legacy_root"])
    out_root = _abs_path(manifest["out_root"])
    report_root = _abs_path(manifest["report_root"])
    inventory = _collect_legacy_inventory(legacy_root)

    if not out_root.is_dir():
        raise FileNotFoundError(f"Missing merged output root: {out_root}")

    counts = _validate_layout_counts(out_root)
    assets = _validate_asset_layout(out_root)
    openability = _validate_layout_openability(out_root, args.open_check_limit)
    layout_refs = _validate_layout_refs(out_root, args.open_check_limit)

    failures: List[str] = []
    if counts["home_layout_count"] != inventory["home_scene_count"]:
        failures.append(
            f"home layout count mismatch: {counts['home_layout_count']} != {inventory['home_scene_count']}"
        )
    if counts["commercial_layout_count"] != inventory["commercial_scene_count"]:
        failures.append(
            f"commercial layout count mismatch: {counts['commercial_layout_count']} != {inventory['commercial_scene_count']}"
        )
    if assets["bad_asset_count"] != 0:
        failures.append(f"bad asset layout count: {assets['bad_asset_count']}")
    if openability.get("enabled") and openability.get("failure_count", 0) != 0:
        failures.append(f"layout open failures: {openability['failure_count']}")
    if layout_refs.get("enabled") and layout_refs.get("legacy_model_ref_count", 0) != 0:
        failures.append(f"layout legacy model refs remain: {layout_refs['legacy_model_ref_count']}")

    report = {
        "legacy_inventory": inventory,
        "out_root": str(out_root),
        "layout_counts": counts,
        "asset_layout": assets,
        "layout_openability": openability,
        "layout_refs": layout_refs,
        "status": "ok" if not failures else "failed",
        "failures": failures,
    }
    _write_json(report_root / "validation_summary.json", report)

    manifest.setdefault("stages", {})["validate"] = {
        "status": report["status"],
        "summary_path": str(report_root / "validation_summary.json"),
    }
    _write_json(_manifest_path(report_root), manifest)

    print(f"Validation summary: {report_root / 'validation_summary.json'}")
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    return 0


def build_parser() -> argparse.ArgumentParser:
    repo_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    submit = sub.add_parser("submit-dlc", help="Submit home/commercial rebuild jobs to DLC")
    submit.add_argument(
        "--legacy-root",
        default="/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100",
        help="Legacy dataset root containing home_scenes/ and commercial_scenes/",
    )
    submit.add_argument(
        "--out-root",
        default=str(repo_root / "GRScenes-test0-rebuilt"),
        help="Final merged output root. Temporary roots are derived from this path.",
    )
    submit.add_argument(
        "--report-root",
        default=None,
        help="Report directory. Defaults to check_reports/rebuild_test0/<run_id>/",
    )
    submit.add_argument("--run-id", default=None, help="Stable run id for temp roots and reports")
    submit.add_argument(
        "--data-sources",
        default=None,
        help="Optional comma-separated DLC data source ids passed through to launch_job.sh",
    )
    submit.add_argument("--dry-run", action="store_true", help="Print submission commands without submitting")
    submit.set_defaults(func=cmd_submit_dlc)

    merge = sub.add_parser("merge", help="Merge completed home/commercial tmp outputs into final root")
    merge.add_argument("--manifest", required=True, help="Path to run_manifest.json from submit-dlc stage")
    merge.set_defaults(func=cmd_merge)

    validate = sub.add_parser("validate", help="Validate the merged GRScenes-test0 output")
    validate.add_argument("--manifest", required=True, help="Path to run_manifest.json from submit-dlc stage")
    validate.add_argument(
        "--open-check-limit",
        type=int,
        default=None,
        help="Optional limit on layout.usd files opened via pxr. Default: check all layouts.",
    )
    validate.set_defaults(func=cmd_validate)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
