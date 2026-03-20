#!/usr/bin/env python3
"""Preflight checks for rebuilt GRScenes-test0 normalize runs.

This script validates a clean rebuilt source root before any smoke or full
normalize execution. It focuses on the failure modes that previously corrupted
`GRScenes-test0`:

- source inventory drift
- main asset USDs being symlinks instead of regular files
- planned output/report roots already existing
- optional spot checks that the rebuilt source differs from the polluted root
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence


EXPECTED_INVENTORY = {
    "category_count": 114,
    "scene_count": 99,
    "home_scene_count": 69,
    "commercial_scene_count": 30,
}

DEFAULT_SAMPLE_ASSETS = (
    {"category": "wall", "uid": "6eb70edb667976379efe987ac4608061"},
    {"category": "ground", "uid": "861585c1868b584326999f86c46c0844"},
    {"category": "other", "uid": "eb1addd1c3ad924b56dfd09c84770558"},
    {"category": "cabinet", "uid": "3399337a68a2bb41b4f4ebc20590fd94"},
)


def _abs_path(value: str) -> Path:
    return Path(value).expanduser().resolve()


def _count_immediate_dirs(path: Path) -> int:
    return sum(1 for child in path.iterdir() if child.is_dir())


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _find_main_usd(source_root: Path, category: str, uid: str) -> Path:
    base = source_root / "GRScenes_assets" / category / uid
    normalized = base / "usd" / f"{uid}.usd"
    flat = base / f"{uid}.usd"
    if normalized.is_file():
        return normalized
    if flat.is_file():
        return flat
    raise FileNotFoundError(f"main usd not found for {category}/{uid} under {source_root}")


def _load_compute_asset_center(repo_root: Path):
    script_path = repo_root / "scripts" / "normalize_asset_transforms.py"
    spec = importlib.util.spec_from_file_location("normalize_asset_transforms", script_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"failed to load normalize script: {script_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.compute_asset_center


def _collect_inventory(source_root: Path) -> Dict[str, int]:
    scenes_root = source_root / "GRScenes100"
    assets_root = source_root / "GRScenes_assets"
    home_root = scenes_root / "home"
    commercial_root = scenes_root / "commercial"
    return {
        "category_count": _count_immediate_dirs(assets_root),
        "scene_count": _count_immediate_dirs(home_root) + _count_immediate_dirs(commercial_root),
        "home_scene_count": _count_immediate_dirs(home_root),
        "commercial_scene_count": _count_immediate_dirs(commercial_root),
    }


def _sample_asset_report(
    *,
    source_root: Path,
    compare_root: Optional[Path],
    sample_assets: Sequence[Dict[str, str]],
    compute_centers: bool,
    compute_asset_center,
) -> List[Dict[str, Any]]:
    reports: List[Dict[str, Any]] = []
    for sample in sample_assets:
        category = sample["category"]
        uid = sample["uid"]
        entry: Dict[str, Any] = {
            "category": category,
            "uid": uid,
        }

        path = _find_main_usd(source_root, category, uid)
        entry["source_main_usd"] = str(path)
        entry["source_is_symlink"] = path.is_symlink()
        entry["source_is_file"] = path.is_file()
        entry["source_sha256"] = _sha256_file(path)

        if compute_centers and compute_asset_center is not None:
            center_info = compute_asset_center(str(path))
            entry["source_center"] = center_info["center"]

        if compare_root is not None:
            compare_entry: Dict[str, Any] = {}
            try:
                compare_path = _find_main_usd(compare_root, category, uid)
                compare_entry["main_usd"] = str(compare_path)
                compare_entry["is_symlink"] = compare_path.is_symlink()
                compare_entry["is_file"] = compare_path.is_file()
                compare_entry["sha256"] = _sha256_file(compare_path)
                if compute_centers and compute_asset_center is not None:
                    compare_center_info = compute_asset_center(str(compare_path))
                    compare_entry["center"] = compare_center_info["center"]
            except Exception as exc:
                compare_entry["error"] = str(exc)
            entry["compare_root"] = compare_entry

        reports.append(entry)
    return reports


def _render_markdown(report: Dict[str, Any]) -> str:
    lines = [
        "# rebuilt normalize preflight",
        "",
        f"- status: `{report['status']}`",
        f"- source_root: `{report['source_root']}`",
        f"- normalized_root: `{report['normalized_root']}`",
        f"- report_root: `{report['report_root']}`",
        f"- inventory: `{report['inventory']}`",
        "",
        "## sample assets",
    ]
    for item in report["sample_assets"]:
        lines.append(
            f"- `{item['category']}/{item['uid']}`: "
            f"source_symlink={item['source_is_symlink']} sha256={item['source_sha256']}"
        )
    if report["failures"]:
        lines.extend(["", "## failures"])
        for failure in report["failures"]:
            lines.append(f"- {failure}")
    return "\n".join(lines) + "\n"


def run_preflight(
    *,
    source_root: Path,
    normalized_root: Path,
    report_root: Path,
    compare_root: Optional[Path],
    sample_assets: Sequence[Dict[str, str]],
    compute_centers: bool,
) -> Dict[str, Any]:
    repo_root = Path(__file__).resolve().parent.parent
    compute_asset_center = None
    center_error: Optional[str] = None
    if compute_centers:
        try:
            compute_asset_center = _load_compute_asset_center(repo_root)
        except Exception as exc:
            center_error = str(exc)

    failures: List[str] = []

    required_dirs = {
        "source_root": source_root,
        "source_assets": source_root / "GRScenes_assets",
        "source_scenes": source_root / "GRScenes100",
        "source_material": source_root / "Material",
    }
    for label, path in required_dirs.items():
        if not path.is_dir():
            failures.append(f"missing required directory: {label} -> {path}")

    inventory = _collect_inventory(source_root) if not failures else {}
    if inventory and inventory != EXPECTED_INVENTORY:
        failures.append(
            f"inventory mismatch: observed={inventory} expected={EXPECTED_INVENTORY}"
        )

    if normalized_root.exists():
        failures.append(f"normalized root must not exist before run: {normalized_root}")
    if report_root.exists():
        failures.append(f"report root must not exist before run: {report_root}")

    sample_reports: List[Dict[str, Any]] = []
    if not failures:
        try:
            sample_reports = _sample_asset_report(
                source_root=source_root,
                compare_root=compare_root,
                sample_assets=sample_assets,
                compute_centers=compute_centers and compute_asset_center is not None,
                compute_asset_center=compute_asset_center,
            )
        except Exception as exc:
            failures.append(str(exc))

    for item in sample_reports:
        if item.get("source_is_symlink"):
            failures.append(
                f"sample source main usd is symlink: {item['category']}/{item['uid']}"
            )

    report = {
        "status": "pass" if not failures else "fail",
        "source_root": str(source_root),
        "normalized_root": str(normalized_root),
        "report_root": str(report_root),
        "compare_root": str(compare_root) if compare_root else None,
        "inventory": inventory,
        "expected_inventory": EXPECTED_INVENTORY,
        "compute_centers": compute_centers,
        "center_loader_error": center_error,
        "sample_assets": sample_reports,
        "failures": failures,
    }
    return report


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-root",
        default=str(repo_root / "GRScenes-test0-rebuilt"),
        help="Clean rebuilt source root",
    )
    parser.add_argument(
        "--normalized-root",
        required=True,
        help="Planned normalized output root, which must not exist yet",
    )
    parser.add_argument(
        "--report-root",
        required=True,
        help="Planned report root, which must not exist yet",
    )
    parser.add_argument(
        "--compare-root",
        default=str(repo_root / "GRScenes-test0"),
        help="Optional compare root used for informative sample comparisons",
    )
    parser.add_argument(
        "--skip-centers",
        action="store_true",
        help="Skip compute_asset_center spot checks",
    )
    parser.add_argument(
        "--json-out",
        default=None,
        help="Optional explicit JSON output path; defaults to <report-root>/baseline_verification.json",
    )
    parser.add_argument(
        "--md-out",
        default=None,
        help="Optional explicit Markdown output path; defaults to <report-root>/baseline_verification.md",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    source_root = _abs_path(args.source_root)
    normalized_root = _abs_path(args.normalized_root)
    report_root = _abs_path(args.report_root)
    compare_root = _abs_path(args.compare_root) if args.compare_root else None
    if compare_root == source_root:
        compare_root = None

    report = run_preflight(
        source_root=source_root,
        normalized_root=normalized_root,
        report_root=report_root,
        compare_root=compare_root,
        sample_assets=DEFAULT_SAMPLE_ASSETS,
        compute_centers=not args.skip_centers,
    )

    json_out = _abs_path(args.json_out) if args.json_out else report_root / "baseline_verification.json"
    md_out = _abs_path(args.md_out) if args.md_out else report_root / "baseline_verification.md"
    json_out.parent.mkdir(parents=True, exist_ok=True)
    md_out.parent.mkdir(parents=True, exist_ok=True)
    with json_out.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    md_out.write_text(_render_markdown(report), encoding="utf-8")

    print(f"Preflight status: {report['status']}")
    print(f"JSON: {json_out}")
    print(f"Markdown: {md_out}")
    if report["status"] != "pass":
        for failure in report["failures"]:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
