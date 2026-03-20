#!/usr/bin/env python3
"""Orchestrate rebuilt GRScenes-test0 normalize runs.

This script exists for the rebuilt clean baseline workflow:

- preflight the rebuilt source root
- run a one-scene smoke normalize-only pass locally
- submit full Phase 1 / Phase 2 to DLC
- wait for DLC completion and finalize normalize-only verification locally

The script is manifest-driven so long DLC waits can resume from a saved
`run_manifest.json` without recomputing prior steps.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
import shlex
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


EXPECTED_INVENTORY = {
    "category_count": 114,
    "scene_count": 99,
    "home_scene_count": 69,
    "commercial_scene_count": 30,
}

DEFAULT_SAMPLE_ASSETS = [
    "wall/6eb70edb667976379efe987ac4608061",
    "ground/861585c1868b584326999f86c46c0844",
    "other/eb1addd1c3ad924b56dfd09c84770558",
    "cabinet/3399337a68a2bb41b4f4ebc20590fd94",
]

TERMINAL_DLC_STATUSES = {"Succeeded", "Failed", "Stopped"}
SUCCESS_DLC_STATUSES = {"Succeeded"}


def _utc_run_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _abs_path(path: str) -> Path:
    return Path(path).expanduser().resolve()


def _ensure_absent_or_empty(path: Path, label: str) -> None:
    if not path.exists():
        return
    if path.is_dir() and not any(path.iterdir()):
        return
    raise RuntimeError(f"{label} already exists and is not empty: {path}")


def _require_dir(path: Path, label: str) -> None:
    if not path.is_dir():
        raise FileNotFoundError(f"{label} not found: {path}")


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _run_command(
    *,
    cmd: Sequence[str],
    cwd: Path,
    log_path: Optional[Path] = None,
    allow_exit_codes: Sequence[int] = (0,),
) -> Dict[str, Any]:
    command_text = shlex.join(str(part) for part in cmd)
    started = time.time()
    stdout_text = ""
    stderr_text = ""

    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)

    completed = subprocess.run(
        [str(part) for part in cmd],
        cwd=str(cwd),
        check=False,
        text=True,
        capture_output=True,
    )
    stdout_text = completed.stdout or ""
    stderr_text = completed.stderr or ""
    elapsed = round(time.time() - started, 1)

    if log_path is not None:
        with log_path.open("w", encoding="utf-8") as handle:
            handle.write(f"$ {command_text}\n\n")
            if stdout_text:
                handle.write(stdout_text)
            if stderr_text:
                if stdout_text and not stdout_text.endswith("\n"):
                    handle.write("\n")
                handle.write(stderr_text)

    if completed.returncode not in allow_exit_codes:
        detail = stderr_text.strip() or stdout_text.strip()
        raise RuntimeError(
            f"Command failed (exit {completed.returncode}): {command_text}\n{detail}"
        )

    return {
        "command": command_text,
        "cwd": str(cwd),
        "exit_code": completed.returncode,
        "elapsed_seconds": elapsed,
        "log_path": str(log_path) if log_path else None,
        "stdout": stdout_text,
        "stderr": stderr_text,
    }


def _count_immediate_dirs(path: Path) -> int:
    return sum(1 for child in path.iterdir() if child.is_dir())


def _collect_inventory(source_root: Path) -> Dict[str, int]:
    source_assets = source_root / "GRScenes_assets"
    source_scenes = source_root / "GRScenes100"
    home_dir = source_scenes / "home"
    commercial_dir = source_scenes / "commercial"
    return {
        "category_count": _count_immediate_dirs(source_assets),
        "scene_count": (
            (_count_immediate_dirs(home_dir) if home_dir.is_dir() else 0)
            + (_count_immediate_dirs(commercial_dir) if commercial_dir.is_dir() else 0)
        ),
        "home_scene_count": _count_immediate_dirs(home_dir) if home_dir.is_dir() else 0,
        "commercial_scene_count": _count_immediate_dirs(commercial_dir) if commercial_dir.is_dir() else 0,
    }


def _collect_source_inventory(source_root: Path) -> Dict[str, int]:
    return _collect_inventory(source_root)


def _resolve_main_usd(source_root: Path, asset_key: str) -> Path:
    category, uid = asset_key.split("/", 1)
    asset_dir = source_root / "GRScenes_assets" / category / uid
    normalized = asset_dir / "usd" / f"{uid}.usd"
    flat = asset_dir / f"{uid}.usd"
    if normalized.is_file() or normalized.is_symlink():
        return normalized
    if flat.is_file() or flat.is_symlink():
        return flat
    raise FileNotFoundError(f"main USD not found for asset {asset_key} under {source_root}")


def _resolve_main_usd_from_rel_path(source_root: Path, rel_path: str) -> Path:
    direct = source_root / rel_path
    if direct.is_file() or direct.is_symlink():
        return direct

    parts = Path(rel_path).parts
    if len(parts) >= 5 and parts[0] == "GRScenes_assets" and parts[3] == "usd":
        category = parts[1]
        uid = parts[2]
        fallback = source_root / "GRScenes_assets" / category / uid / f"{uid}.usd"
        if fallback.is_file() or fallback.is_symlink():
            return fallback
    return direct


def _discover_main_usd_files(assets_root: Path) -> List[Path]:
    results: List[Path] = []
    for category_dir in sorted(path for path in assets_root.iterdir() if path.is_dir()):
        for uid_dir in sorted(path for path in category_dir.iterdir() if path.is_dir()):
            uid = uid_dir.name
            normalized = uid_dir / "usd" / f"{uid}.usd"
            flat = uid_dir / f"{uid}.usd"
            if normalized.exists() or normalized.is_symlink():
                results.append(normalized)
            elif flat.exists() or flat.is_symlink():
                results.append(flat)
    return results


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _capture_hashes(paths: Iterable[Path]) -> Dict[str, str]:
    return {str(path): f"sha256:{_sha256(path)}" for path in paths}


def _capture_hash_baseline(source_root: Path, rel_paths: Iterable[str]) -> Dict[str, str]:
    baseline: Dict[str, str] = {}
    for rel_path in rel_paths:
        target = _resolve_main_usd_from_rel_path(source_root, rel_path)
        if not target.is_file():
            raise FileNotFoundError(f"hash sample file not found: {target}")
        baseline[rel_path] = _sha256(target)
    return baseline


def _compare_hashes(expected: Dict[str, str]) -> Dict[str, Any]:
    changed: List[Dict[str, str]] = []
    missing: List[str] = []
    for path_str, expected_hash in sorted(expected.items()):
        path = Path(path_str)
        if not path.is_file():
            missing.append(path_str)
            continue
        observed = f"sha256:{_sha256(path)}"
        if observed != expected_hash:
            changed.append(
                {
                    "path": path_str,
                    "expected": expected_hash,
                    "observed": observed,
                }
            )
    return {
        "checked_count": len(expected),
        "missing_count": len(missing),
        "changed_count": len(changed),
        "missing": missing,
        "changed": changed,
    }


def _compare_hash_baseline(source_root: Path, baseline: Dict[str, str]) -> Dict[str, Any]:
    mismatches: List[Dict[str, str]] = []
    missing_paths: List[str] = []
    for rel_path, expected_hash in sorted(baseline.items()):
        target = _resolve_main_usd_from_rel_path(source_root, rel_path)
        if not target.is_file():
            missing_paths.append(rel_path)
            continue
        observed_hash = _sha256(target)
        if observed_hash != expected_hash:
            mismatches.append(
                {
                    "path": rel_path,
                    "expected_sha256": expected_hash,
                    "observed_sha256": observed_hash,
                }
            )
    return {
        "checked_count": len(baseline),
        "missing_count": len(missing_paths),
        "mismatch_count": len(mismatches),
        "missing_paths": missing_paths,
        "mismatches": mismatches,
        "status": "ok" if not missing_paths and not mismatches else "fail",
    }


def _load_normalize_module() -> Optional[Any]:
    try:
        import pxr  # noqa: F401
    except Exception:
        return None

    script_path = _repo_root() / "scripts" / "normalize_asset_transforms.py"
    spec = importlib.util.spec_from_file_location(
        "normalize_asset_transforms_runtime",
        script_path,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load normalize script: {script_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _sample_asset_checks(
    *,
    source_root: Path,
    compare_root: Optional[Path],
    sample_assets: Sequence[str],
    include_centers: bool,
) -> Dict[str, Any]:
    checks: List[Dict[str, Any]] = []
    normalize_module = _load_normalize_module() if include_centers else None
    for asset_key in sample_assets:
        source_usd = _resolve_main_usd(source_root, asset_key)
        entry: Dict[str, Any] = {
            "asset": asset_key,
            "source_main_usd": str(source_usd),
            "source_exists": source_usd.exists(),
            "source_is_symlink": source_usd.is_symlink(),
        }
        if source_usd.is_file():
            entry["source_hash"] = f"sha256:{_sha256(source_usd)}"

        if compare_root is not None:
            try:
                compare_usd = _resolve_main_usd(compare_root, asset_key)
                entry["compare_main_usd"] = str(compare_usd)
                entry["compare_exists"] = compare_usd.exists()
                entry["compare_is_symlink"] = compare_usd.is_symlink()
                if compare_usd.is_file():
                    entry["compare_hash"] = f"sha256:{_sha256(compare_usd)}"
            except FileNotFoundError as exc:
                entry["compare_error"] = str(exc)

        if normalize_module is not None:
            try:
                source_center = normalize_module.compute_asset_center(str(source_usd))
                entry["source_center"] = source_center["center"]
            except Exception as exc:
                entry["source_center_error"] = str(exc)

            compare_main = entry.get("compare_main_usd")
            if compare_main:
                try:
                    compare_center = normalize_module.compute_asset_center(compare_main)
                    entry["compare_center"] = compare_center["center"]
                except Exception as exc:
                    entry["compare_center_error"] = str(exc)

        checks.append(entry)
    return {
        "center_checks_enabled": normalize_module is not None,
        "checks": checks,
    }


def _build_preflight_report(
    *,
    source_root: Path,
    compare_root: Optional[Path],
    normalized_root: Optional[Path],
    report_root: Path,
    sample_assets: Sequence[str],
    include_centers: bool,
) -> Dict[str, Any]:
    source_assets = source_root / "GRScenes_assets"
    source_scenes = source_root / "GRScenes100"
    material_root = source_root / "Material"

    _require_dir(source_root, "source root")
    _require_dir(source_assets, "source assets root")
    _require_dir(source_scenes, "source scenes root")
    _require_dir(material_root, "source material root")

    inventory = _collect_inventory(source_root)
    sample_checks = _sample_asset_checks(
        source_root=source_root,
        compare_root=compare_root,
        sample_assets=sample_assets,
        include_centers=include_centers,
    )

    freshness: Dict[str, Any] = {}
    if normalized_root is not None:
        freshness = {
            "normalized_root": str(normalized_root),
            "exists": normalized_root.exists(),
            "is_empty_or_absent": (
                (not normalized_root.exists())
                or (normalized_root.is_dir() and not any(normalized_root.iterdir()))
            ),
        }

    report = {
        "created_at_utc": _now_iso(),
        "source_root": str(source_root),
        "compare_root": str(compare_root) if compare_root else None,
        "report_root": str(report_root),
        "inventory": inventory,
        "expected_inventory": EXPECTED_INVENTORY,
        "inventory_matches_expected": inventory == EXPECTED_INVENTORY,
        "output_freshness": freshness,
        "sample_asset_checks": sample_checks,
        "status": "pass",
        "failures": [],
    }

    if inventory != EXPECTED_INVENTORY:
        report["failures"].append(
            f"inventory mismatch: observed={inventory} expected={EXPECTED_INVENTORY}"
        )

    if normalized_root is not None and not freshness["is_empty_or_absent"]:
        report["failures"].append(
            f"normalized root already exists and is not empty: {normalized_root}"
        )

    for item in sample_checks["checks"]:
        if item["source_is_symlink"]:
            report["failures"].append(
                f"sample main USD is symlink in rebuilt root: {item['asset']}"
            )

    if report["failures"]:
        report["status"] = "fail"
    return report


def _write_preflight_markdown(report: Dict[str, Any], path: Path) -> None:
    lines = [
        f"# Rebuilt Normalize Preflight: `{report['status']}`",
        "",
        f"- source_root: `{report['source_root']}`",
        f"- compare_root: `{report['compare_root']}`",
        f"- inventory: `{report['inventory']}`",
        f"- expected_inventory: `{report['expected_inventory']}`",
        "",
    ]
    freshness = report.get("output_freshness") or {}
    if freshness:
        lines.extend(
            [
                "## Output Freshness",
                "",
                f"- normalized_root: `{freshness.get('normalized_root')}`",
                f"- exists: `{freshness.get('exists')}`",
                f"- is_empty_or_absent: `{freshness.get('is_empty_or_absent')}`",
                "",
            ]
        )

    lines.append("## Sample Assets")
    lines.append("")
    for item in report["sample_asset_checks"]["checks"]:
        lines.append(f"- `{item['asset']}`")
        lines.append(f"  source_is_symlink: `{item.get('source_is_symlink')}`")
        if "source_center" in item:
            lines.append(f"  source_center: `{item['source_center']}`")
        if "compare_center" in item:
            lines.append(f"  compare_center: `{item['compare_center']}`")
    lines.append("")

    if report["failures"]:
        lines.append("## Failures")
        lines.append("")
        for failure in report["failures"]:
            lines.append(f"- {failure}")

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _parse_dlc_table(output: str) -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []
    header: Optional[List[str]] = None
    for raw_line in output.splitlines():
        line = raw_line.rstrip()
        if not line.startswith("|"):
            continue
        parts = [part.strip() for part in line.strip().strip("|").split("|")]
        if not parts or all(not part for part in parts):
            continue
        if header is None:
            header = parts
            continue
        if len(parts) != len(header):
            continue
        rows.append(dict(zip(header, parts)))
    return rows


def _parse_dlc_job_table(output: str) -> List[Dict[str, str]]:
    return _parse_dlc_table(output)


def _dlc_workspace_id() -> str:
    return os.environ.get("DLC_WORKSPACE_ID", "270969")


def _query_dlc_jobs(
    *,
    display_name_regex: Optional[str] = None,
    display_name: Optional[str] = None,
    start_time: Optional[str] = None,
    page_size: int = 200,
) -> List[Dict[str, str]]:
    repo_root = _repo_root()
    dlc_bin = repo_root / "dlc"
    rows: List[Dict[str, str]] = []
    page_num = 1
    effective_page_size = min(max(1, page_size), 100)

    while True:
        cmd = [
            str(dlc_bin),
            "get",
            "job",
            "--workspace_id",
            _dlc_workspace_id(),
            "--page_size",
            str(effective_page_size),
            "--page_num",
            str(page_num),
        ]
        if display_name:
            cmd.extend(["--display_name", display_name])
        elif display_name_regex:
            cmd.extend(["--display_name_regex", display_name_regex])
        else:
            raise ValueError("display_name or display_name_regex is required")
        if start_time:
            cmd.extend(["--start_time", start_time])
        completed = subprocess.run(
            cmd,
            cwd=str(repo_root),
            check=False,
            text=True,
            capture_output=True,
        )
        if completed.returncode != 0:
            raise RuntimeError(
                f"Failed to query DLC jobs: {shlex.join(cmd)}\n{completed.stderr.strip()}"
            )
        page_rows = _parse_dlc_table(completed.stdout)
        if not page_rows:
            break
        rows.extend(page_rows)
        if len(page_rows) < effective_page_size:
            break
        page_num += 1

    return rows


def _phase1_display_name(run_id: str, category: str) -> str:
    return f"norm_p1_{run_id}_{category}_0_1"


def _expected_phase1_display_names(categories: Iterable[str], run_id: str) -> List[str]:
    return sorted(_phase1_display_name(run_id, category) for category in categories)


def _phase2_display_name(run_id: str) -> str:
    return f"norm_p2_{run_id}_scenes_0_1"


def _wait_for_jobs(
    *,
    expected_names: Sequence[str],
    start_time: str,
    poll_seconds: int,
    status_path: Path,
) -> Dict[str, Any]:
    expected = set(expected_names)
    latest_snapshot: Dict[str, Any] = {}

    while True:
        latest_by_name: Dict[str, Dict[str, str]] = {}
        for expected_name in sorted(expected):
            rows = _query_dlc_jobs(
                display_name=expected_name,
                start_time=start_time,
                page_size=5,
            )
            if not rows:
                continue
            latest_row = max(rows, key=lambda row: row.get("CreateTime", ""))
            latest_by_name[expected_name] = latest_row

        pending = sorted(name for name in expected if name not in latest_by_name)
        terminal = {
            name: row.get("JobStatus", "")
            for name, row in latest_by_name.items()
            if row.get("JobStatus", "") in TERMINAL_DLC_STATUSES
        }
        all_known = not pending
        all_terminal = all_known and len(terminal) == len(expected)

        latest_snapshot = {
            "updated_at_utc": _now_iso(),
            "expected_job_count": len(expected),
            "observed_job_count": len(latest_by_name),
            "pending_jobs": pending,
            "jobs": latest_by_name,
            "all_terminal": all_terminal,
        }
        _write_json(status_path, latest_snapshot)

        if all_terminal:
            return latest_snapshot
        time.sleep(poll_seconds)


def _count_output_main_usd_symlinks(output_root: Path) -> Dict[str, Any]:
    assets_root = output_root / "GRScenes_assets"
    if not assets_root.is_dir():
        return {
            "checked_main_usd_count": 0,
            "symlink_main_usd_count": 0,
            "symlink_main_usd_preview": [],
            "assets_root_exists": False,
        }
    main_usds = _discover_main_usd_files(assets_root)
    symlink_paths = [str(path) for path in main_usds if path.is_symlink()]
    return {
        "checked_main_usd_count": len(main_usds),
        "symlink_main_usd_count": len(symlink_paths),
        "symlink_main_usd_preview": symlink_paths[:50],
        "assets_root_exists": True,
    }


def _load_phase2_report_summary(phase2_report_path: Path) -> Dict[str, Any]:
    if not phase2_report_path.is_file():
        raise FileNotFoundError(f"phase2 normalize report not found: {phase2_report_path}")
    data = _load_json(phase2_report_path)
    meta = data.get("meta", {})
    if meta.get("scenes_processed") != 99:
        raise RuntimeError(f"unexpected phase2 scenes_processed: {meta.get('scenes_processed')}")
    if meta.get("errors_count") != 0:
        raise RuntimeError(f"unexpected phase2 errors_count: {meta.get('errors_count')}")
    return data


def _build_manifest(
    *,
    run_id: str,
    source_root: Path,
    normalized_root: Path,
    report_root: Path,
    sample_hashes: Dict[str, str],
    sample_assets: Sequence[str],
) -> Dict[str, Any]:
    phase1_root = report_root / "normalize" / "phase1"
    phase2_root = report_root / "normalize" / "phase2"
    summary_root = report_root / "summary"
    categories = sorted(
        path.name
        for path in (source_root / "GRScenes_assets").iterdir()
        if path.is_dir()
    )
    return {
        "run_id": run_id,
        "created_at_utc": _now_iso(),
        "repo_root": str(_repo_root()),
        "source_root": str(source_root),
        "normalized_root": str(normalized_root),
        "report_root": str(report_root),
        "sample_assets": list(sample_assets),
        "source_hash_baseline": sample_hashes,
        "phase1_root": str(phase1_root),
        "phase2_root": str(phase2_root),
        "summary_root": str(summary_root),
        "categories": categories,
        "stages": {},
        "status": "planned",
    }


def _write_run_status(summary_root: Path, payload: Dict[str, Any]) -> None:
    _write_json(summary_root / "run_status.json", payload)


def cmd_preflight(args: argparse.Namespace) -> int:
    repo_root = _repo_root()
    source_root = _abs_path(args.source_root)
    compare_root = _abs_path(args.compare_root) if args.compare_root else None
    normalized_root = _abs_path(args.normalized_root) if args.normalized_root else None
    run_id = args.run_id or _utc_run_id()
    report_root = _abs_path(
        args.report_root
        or str(repo_root / "check_reports" / "preflight" / "test0_rebuilt" / run_id)
    )
    _ensure_absent_or_empty(report_root, "preflight report root")

    report = _build_preflight_report(
        source_root=source_root,
        compare_root=compare_root,
        normalized_root=normalized_root,
        report_root=report_root,
        sample_assets=args.sample_asset or DEFAULT_SAMPLE_ASSETS,
        include_centers=not args.skip_center_checks,
    )
    report_root.mkdir(parents=True, exist_ok=False)
    _write_json(report_root / "preflight_report.json", report)
    _write_preflight_markdown(report, report_root / "preflight_report.md")
    print(f"Preflight report: {report_root / 'preflight_report.json'}")
    return 0 if report["status"] == "pass" else 1


def cmd_smoke(args: argparse.Namespace) -> int:
    repo_root = _repo_root()
    run_id = args.run_id or _utc_run_id()
    source_root = _abs_path(args.source_root)
    smoke_root = _abs_path(
        args.smoke_root or str(repo_root / "GRScenes-test0-rebuilt-smoke-S1")
    )
    normalized_root = _abs_path(
        args.normalized_root
        or str(repo_root / "GRScenes-test0-rebuilt-smoke-S1-normalized")
    )
    report_root = _abs_path(
        args.report_root
        or str(repo_root / "check_reports" / "test0_rebuilt_smoke" / run_id)
    )
    logs_dir = report_root / "logs"
    summary_root = report_root / "summary"
    normalize_only_report_root = report_root / "normalize_only"

    _ensure_absent_or_empty(smoke_root, "smoke source root")
    _ensure_absent_or_empty(normalized_root, "smoke normalized root")
    _ensure_absent_or_empty(report_root, "smoke report root")
    _require_dir(source_root, "rebuilt source root")

    report_root.mkdir(parents=True, exist_ok=False)
    logs_dir.mkdir(parents=True, exist_ok=True)

    isaac_python = repo_root / "scripts" / "isaac_python.sh"
    build_cmd = [
        str(isaac_python),
        str(repo_root / "scripts" / "build_scene_uid_subset_package.py"),
        "--src",
        str(source_root),
        "--dst",
        str(smoke_root),
        "--scene-uid",
        args.scene_uid,
        "--scene-category",
        args.scene_category,
        "--write-manifest",
        "--verify",
    ]
    _run_command(
        cmd=build_cmd,
        cwd=repo_root,
        log_path=logs_dir / "build_subset.log",
    )

    source_hashes = _capture_hashes(
        _discover_main_usd_files(smoke_root / "GRScenes_assets")
    )
    normalize_cmd = [
        sys.executable,
        str(repo_root / "scripts" / "test0_full_normalize.py"),
        "--run-id",
        run_id,
        "--source-root",
        str(smoke_root),
        "--normalized-root",
        str(normalized_root),
        "--report-root",
        str(normalize_only_report_root),
        "--symlink-copy",
        "--skip-inventory-check",
    ]
    normalize_result = _run_command(
        cmd=normalize_cmd,
        cwd=repo_root,
        log_path=logs_dir / "smoke_normalize.log",
        allow_exit_codes=(0, 1),
    )

    hash_check = _compare_hashes(source_hashes)
    output_symlink_check = _count_output_main_usd_symlinks(normalized_root)
    verification = {
        "run_id": run_id,
        "source_root": str(smoke_root),
        "normalized_root": str(normalized_root),
        "normalize_exit_code": normalize_result["exit_code"],
        "source_hash_check": hash_check,
        "output_main_usd_symlink_check": output_symlink_check,
        "status": "pass",
        "failures": [],
    }
    if hash_check["missing_count"] or hash_check["changed_count"]:
        verification["failures"].append("smoke source hashes changed after normalize run")
    if output_symlink_check["symlink_main_usd_count"] != 0:
        verification["failures"].append("smoke output contains symlink main USD files")
    if not output_symlink_check.get("assets_root_exists", False):
        verification["failures"].append("smoke normalized assets root was not produced")
    if normalize_result["exit_code"] != 0:
        verification["failures"].append(
            f"smoke normalize wrapper exited nonzero: {normalize_result['exit_code']}"
        )
    if verification["failures"]:
        verification["status"] = "fail"
    _write_json(summary_root / "smoke_verification.json", verification)
    _write_run_status(
        summary_root,
        {
            "run_id": run_id,
            "status": verification["status"],
            "smoke_verification": str(summary_root / "smoke_verification.json"),
        },
    )
    print(f"Smoke verification: {summary_root / 'smoke_verification.json'}")
    return 0 if verification["status"] == "pass" else 1


def cmd_submit_full(args: argparse.Namespace) -> int:
    repo_root = _repo_root()
    run_id = args.run_id or _utc_run_id()
    source_root = _abs_path(args.source_root)
    normalized_root = _abs_path(
        args.normalized_root
        or str(repo_root / "GRScenes-test0-rebuilt-normalized")
    )
    report_root = _abs_path(
        args.report_root
        or str(repo_root / "check_reports" / "test0_rebuilt_full" / run_id)
    )
    phase1_root = report_root / "normalize" / "phase1"
    summary_root = report_root / "summary"
    compare_root = _abs_path(args.compare_root) if args.compare_root else None
    sample_assets = args.sample_asset or DEFAULT_SAMPLE_ASSETS

    _ensure_absent_or_empty(report_root, "full report root")
    _ensure_absent_or_empty(normalized_root, "full normalized root")
    preflight = _build_preflight_report(
        source_root=source_root,
        compare_root=compare_root,
        normalized_root=normalized_root,
        report_root=report_root,
        sample_assets=sample_assets,
        include_centers=not args.skip_center_checks,
    )
    if preflight["status"] != "pass":
        raise RuntimeError("rebuilt preflight failed; inspect failures in generated report")

    report_root.mkdir(parents=True, exist_ok=False)
    _write_json(report_root / "preflight_report.json", preflight)
    _write_preflight_markdown(preflight, report_root / "preflight_report.md")
    phase1_root.mkdir(parents=True, exist_ok=True)
    summary_root.mkdir(parents=True, exist_ok=True)

    source_hashes = {
        str(_resolve_main_usd(source_root, asset_key)): f"sha256:{_sha256(_resolve_main_usd(source_root, asset_key))}"
        for asset_key in sample_assets
    }
    manifest = _build_manifest(
        run_id=run_id,
        source_root=source_root,
        normalized_root=normalized_root,
        report_root=report_root,
        sample_hashes=source_hashes,
        sample_assets=sample_assets,
    )
    manifest["preflight_report"] = str(report_root / "preflight_report.json")
    manifest["status"] = "submitting_phase1"
    _write_json(report_root / "run_manifest.json", manifest)

    assets_root = source_root / "GRScenes_assets"
    scenes_root = source_root / "GRScenes100"
    phase1_jobs: Dict[str, Any] = {}
    for category in manifest["categories"]:
        cat_report_dir = phase1_root / category
        cat_report_dir.mkdir(parents=True, exist_ok=True)
        task_name = f"norm_p1_{run_id}_{category}"
        display_name = _phase1_display_name(run_id, category)
        cmd_args = (
            "normalize_assets "
            f"--assets-root {shlex.quote(str(assets_root))} "
            f"--scenes-root {shlex.quote(str(scenes_root))} "
            f"--output-root {shlex.quote(str(normalized_root))} "
            f"--category {shlex.quote(category)} "
            "--phase 1 --symlink-copy "
            f"--report-dir {shlex.quote(str(cat_report_dir))}"
        )
        submit_cmd = [
            "bash",
            str(repo_root / "scripts" / "dlc" / "launch_job.sh"),
            task_name,
            "0",
            "1",
            "",
            cmd_args,
        ]
        result = {
            "log_path": str(phase1_root / "_submit_logs" / f"{category}.log"),
        }
        if not args.dry_run:
            result = _run_command(
                cmd=submit_cmd,
                cwd=repo_root,
                log_path=phase1_root / "_submit_logs" / f"{category}.log",
            )
        phase1_jobs[category] = {
            "task_name": task_name,
            "display_name": display_name,
            "report_dir": str(cat_report_dir),
            "command_args": cmd_args,
            "submission_log": result["log_path"],
        }

    manifest["stages"]["phase1_submit"] = {
        "status": "planned" if args.dry_run else "submitted",
        "submitted_at_utc": _now_iso(),
        "jobs": phase1_jobs,
    }
    manifest["status"] = "planned" if args.dry_run else "phase1_submitted"
    _write_json(report_root / "run_manifest.json", manifest)
    _write_run_status(
        summary_root,
        {
            "run_id": run_id,
            "status": "planned" if args.dry_run else "phase1_submitted",
            "manifest": str(report_root / "run_manifest.json"),
        },
    )
    print(f"Manifest: {report_root / 'run_manifest.json'}")
    return 0


def cmd_wait_full(args: argparse.Namespace) -> int:
    repo_root = _repo_root()
    manifest_path = _abs_path(args.manifest)
    manifest = _load_json(manifest_path)
    report_root = _abs_path(manifest["report_root"])
    phase1_root = _abs_path(manifest["phase1_root"])
    phase2_root = _abs_path(manifest["phase2_root"])
    summary_root = _abs_path(manifest["summary_root"])
    source_root = _abs_path(manifest["source_root"])
    normalized_root = _abs_path(manifest["normalized_root"])
    run_id = str(manifest["run_id"])
    source_hash_baseline = manifest.get("source_hash_baseline", {})

    summary_root.mkdir(parents=True, exist_ok=True)
    _write_run_status(
        summary_root,
        {
            "run_id": run_id,
            "status": "waiting_phase1",
            "manifest": str(manifest_path),
        },
    )

    if "phase1_wait" not in manifest["stages"]:
        phase1_expected = [
            manifest["stages"]["phase1_submit"]["jobs"][category]["display_name"]
            for category in manifest["categories"]
        ]
        phase1_status = _wait_for_jobs(
            expected_names=phase1_expected,
            start_time=manifest["created_at_utc"],
            poll_seconds=args.poll_seconds,
            status_path=phase1_root / "phase1_dlc_status.json",
        )
        manifest["stages"]["phase1_wait"] = phase1_status
        _write_json(manifest_path, manifest)

    phase1_summary_json = phase1_root / "phase1_remaining_parallel_summary.json"
    phase1_summary_payload: Dict[str, Any] = {}
    jobs = manifest["stages"]["phase1_submit"]["jobs"]
    phase1_wait = manifest["stages"]["phase1_wait"]["jobs"]
    for category, info in jobs.items():
        centers_files = sorted(
            path.name for path in Path(info["report_dir"]).glob("centers_*.json")
        )
        status = phase1_wait.get(info["display_name"], {}).get("JobStatus", "")
        phase1_summary_payload[category] = {
            "job_name": info["display_name"],
            "job_status": status,
            "returncode": 0 if status in SUCCESS_DLC_STATUSES else 1,
            "centers_files": centers_files,
        }
    _write_json(phase1_summary_json, phase1_summary_payload)

    allowlist_result = _run_command(
        cmd=[
            sys.executable,
            str(repo_root / "scripts" / "check_test0_phase1_allowlist.py"),
            "--phase1-root",
            str(phase1_root),
            "--summary-json",
            str(phase1_summary_json),
            "--json-out",
            str(phase1_root / "phase1_allowlist_verdict.json"),
        ],
        cwd=repo_root,
        log_path=phase1_root / "phase1_allowlist.log",
    )
    manifest["stages"]["phase1_allowlist"] = {
        "status": "pass",
        "log_path": allowlist_result["log_path"],
        "verdict_json": str(phase1_root / "phase1_allowlist_verdict.json"),
    }

    merge_result = _run_command(
        cmd=[
            sys.executable,
            str(repo_root / "scripts" / "merge_phase1_centers.py"),
            "--phase1-root",
            str(phase1_root),
            "--out-dir",
            str(phase1_root / "centers_merged"),
        ],
        cwd=repo_root,
        log_path=phase1_root / "merge_phase1_centers.log",
    )
    manifest["stages"]["phase1_merge"] = {
        "status": "completed",
        "log_path": merge_result["log_path"],
        "centers_dir": str(phase1_root / "centers_merged"),
    }
    _write_json(manifest_path, manifest)

    if "phase2_submit" not in manifest["stages"]:
        phase2_root.mkdir(parents=True, exist_ok=True)
        cmd_args = (
            "normalize_assets "
            f"--assets-root {shlex.quote(str(source_root / 'GRScenes_assets'))} "
            f"--scenes-root {shlex.quote(str(source_root / 'GRScenes100'))} "
            f"--output-root {shlex.quote(str(normalized_root))} "
            "--phase 2 "
            f"--centers-dir {shlex.quote(str(phase1_root / 'centers_merged'))} "
            "--symlink-copy "
            f"--report-dir {shlex.quote(str(phase2_root))}"
        )
        submit_result = _run_command(
            cmd=[
                "bash",
                str(repo_root / "scripts" / "dlc" / "launch_job.sh"),
                f"norm_p2_{run_id}_scenes",
                "0",
                "1",
                "",
                cmd_args,
            ],
            cwd=repo_root,
            log_path=phase2_root / "phase2_submit.log",
        )
        manifest["stages"]["phase2_submit"] = {
            "status": "submitted",
            "display_name": _phase2_display_name(run_id),
            "command_args": cmd_args,
            "log_path": submit_result["log_path"],
        }
        _write_json(manifest_path, manifest)

    if "phase2_wait" not in manifest["stages"]:
        phase2_status = _wait_for_jobs(
            expected_names=[manifest["stages"]["phase2_submit"]["display_name"]],
            start_time=manifest["created_at_utc"],
            poll_seconds=args.poll_seconds,
            status_path=phase2_root / "phase2_dlc_status.json",
        )
        manifest["stages"]["phase2_wait"] = phase2_status
        _write_json(manifest_path, manifest)

    phase2_job_status = (
        manifest["stages"]["phase2_wait"]["jobs"]
        .get(manifest["stages"]["phase2_submit"]["display_name"], {})
        .get("JobStatus")
    )
    if phase2_job_status not in SUCCESS_DLC_STATUSES:
        raise RuntimeError(f"phase2 DLC job did not succeed: {phase2_job_status}")

    _load_phase2_report_summary(phase2_root / "normalize_report.json")

    isaac_python = repo_root / "scripts" / "isaac_python.sh"
    normalize_root = report_root / "normalize"
    pairwise_json = normalize_root / "test0_vs_normalized_pre_dedup.json"
    audit_json = normalize_root / "test0_normalize_phase2_audit.json"

    post_steps = [
        (
            "pre_c1_snapshot",
            [
                sys.executable,
                str(repo_root / "scripts" / "create_pre_c1_normalize_only_snapshots.py"),
                "--normalized-root",
                str(normalized_root),
                "--run-id",
                run_id,
                "--report-path",
                str(normalize_root / "pre_c1_snapshot_report.json"),
            ],
            normalize_root / "pre_c1_snapshot.log",
        ),
        (
            "pairwise",
            [
                str(isaac_python),
                str(repo_root / "scripts" / "placement_pairwise_compare.py"),
                "--left-root",
                str(source_root),
                "--right-root",
                str(normalized_root),
                "--left-mode",
                "current",
                "--right-mode",
                "current",
                "--label",
                f"test0_rebuilt_{run_id}",
                "--workers",
                str(args.workers),
                "--out",
                str(pairwise_json),
            ],
            normalize_root / "pairwise.log",
        ),
        (
            "audit",
            [
                str(isaac_python),
                str(repo_root / "scripts" / "audit_normalize_phase2.py"),
                "--source-root",
                str(source_root),
                "--normalized-root",
                str(normalized_root),
                "--centers-dir",
                str(phase1_root / "centers_merged"),
                "--workers",
                str(args.workers),
                "--out",
                str(audit_json),
            ],
            normalize_root / "audit.log",
        ),
        (
            "bundle",
            [
                sys.executable,
                str(repo_root / "scripts" / "assemble_normalize_gate_bundle.py"),
                "--summary-dir",
                str(summary_root),
                "--run-id",
                run_id,
                "--pairwise-report",
                str(pairwise_json),
                "--audit-report",
                str(audit_json),
                "--python",
                sys.executable,
            ],
            normalize_root / "bundle.log",
        ),
    ]

    for name, cmd, log_path in post_steps:
        if name in manifest["stages"]:
            continue
        allow_exit_codes = (0, 1) if name == "bundle" else (0,)
        result = _run_command(
            cmd=cmd,
            cwd=repo_root,
            log_path=log_path,
            allow_exit_codes=allow_exit_codes,
        )
        manifest["stages"][name] = {
            "status": "completed" if result["exit_code"] == 0 else "failed",
            "exit_code": result["exit_code"],
            "log_path": result["log_path"],
        }
        _write_json(manifest_path, manifest)

    hash_check = _compare_hashes(source_hash_baseline)
    output_symlink_check = _count_output_main_usd_symlinks(normalized_root)
    verification = {
        "source_hash_check": hash_check,
        "output_main_usd_symlink_check": output_symlink_check,
        "status": "pass",
        "failures": [],
    }
    if hash_check["missing_count"] or hash_check["changed_count"]:
        verification["failures"].append("source hash baseline changed during full normalize")
    if output_symlink_check["symlink_main_usd_count"] != 0:
        verification["failures"].append("normalized output contains symlink main USD files")
    if manifest["stages"]["bundle"]["exit_code"] != 0:
        verification["failures"].append("normalize-only hard gate failed")
    if verification["failures"]:
        verification["status"] = "fail"

    _write_json(summary_root / "safety_verification.json", verification)
    manifest["stages"]["safety_verification"] = verification
    manifest["status"] = "completed" if verification["status"] == "pass" else "failed"
    _write_json(manifest_path, manifest)

    _write_run_status(
        summary_root,
        {
            "run_id": run_id,
            "status": verification["status"],
            "manifest": str(manifest_path),
            "safety_verification": str(summary_root / "safety_verification.json"),
            "final_verdict": str(summary_root / "final_verdict.json"),
        },
    )
    print(f"Run manifest: {manifest_path}")
    print(f"Run status:   {summary_root / 'run_status.json'}")
    return 0 if verification["status"] == "pass" else 1


def build_parser() -> argparse.ArgumentParser:
    repo_root = _repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    preflight = sub.add_parser("preflight", help="Validate rebuilt source root before normalize")
    preflight.add_argument(
        "--source-root",
        default=str(repo_root / "GRScenes-test0-rebuilt"),
        help="Rebuilt clean baseline root",
    )
    preflight.add_argument(
        "--compare-root",
        default=str(repo_root / "GRScenes-test0"),
        help="Optional polluted root for spot-check comparison",
    )
    preflight.add_argument(
        "--normalized-root",
        default=None,
        help="Optional normalized output root to enforce fresh-only execution",
    )
    preflight.add_argument("--report-root", default=None, help="Preflight report output root")
    preflight.add_argument("--run-id", default=None, help="Stable run id")
    preflight.add_argument(
        "--sample-asset",
        action="append",
        default=None,
        help="Repeatable <category>/<uid> sample asset spot-check",
    )
    preflight.add_argument(
        "--skip-center-checks",
        action="store_true",
        help="Skip compute_asset_center spot-checks when pxr/Isaac is unavailable",
    )
    preflight.set_defaults(func=cmd_preflight)

    smoke = sub.add_parser("smoke", help="Run one-scene smoke normalize-only pass locally")
    smoke.add_argument(
        "--source-root",
        default=str(repo_root / "GRScenes-test0-rebuilt"),
        help="Rebuilt clean baseline root",
    )
    smoke.add_argument(
        "--scene-uid",
        default="MV7J6NIKTKJZ2AABAAAAADA8",
        help="Scene UID to package for smoke testing",
    )
    smoke.add_argument(
        "--scene-category",
        default="home",
        help="Scene category for --scene-uid",
    )
    smoke.add_argument("--smoke-root", default=None, help="Smoke subset root")
    smoke.add_argument("--normalized-root", default=None, help="Smoke normalized root")
    smoke.add_argument("--report-root", default=None, help="Smoke report root")
    smoke.add_argument("--run-id", default=None, help="Stable smoke run id")
    smoke.set_defaults(func=cmd_smoke)

    submit_full = sub.add_parser("submit-full", help="Submit full rebuilt Phase 1 to DLC")
    submit_full.add_argument(
        "--source-root",
        default=str(repo_root / "GRScenes-test0-rebuilt"),
        help="Rebuilt clean baseline root",
    )
    submit_full.add_argument(
        "--compare-root",
        default=str(repo_root / "GRScenes-test0"),
        help="Optional polluted root for spot-check comparison",
    )
    submit_full.add_argument("--normalized-root", default=None, help="Full normalized output root")
    submit_full.add_argument("--report-root", default=None, help="Full report root")
    submit_full.add_argument("--run-id", default=None, help="Stable full run id")
    submit_full.add_argument(
        "--sample-asset",
        action="append",
        default=None,
        help="Repeatable <category>/<uid> sample asset hash baseline",
    )
    submit_full.add_argument(
        "--skip-center-checks",
        action="store_true",
        help="Skip compute_asset_center spot-checks when pxr/Isaac is unavailable",
    )
    submit_full.add_argument(
        "--dry-run",
        action="store_true",
        help="Write the manifest and per-category commands without submitting DLC jobs",
    )
    submit_full.set_defaults(func=cmd_submit_full)

    wait_full = sub.add_parser("wait-full", help="Wait for DLC jobs and finalize full rebuilt run")
    wait_full.add_argument("--manifest", required=True, help="Path to run_manifest.json from submit-full")
    wait_full.add_argument(
        "--poll-seconds",
        type=int,
        default=60,
        help="Polling interval for DLC status checks",
    )
    wait_full.add_argument(
        "--workers",
        type=int,
        default=8,
        help="Worker count for pairwise/audit post-processing",
    )
    wait_full.set_defaults(func=cmd_wait_full)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
