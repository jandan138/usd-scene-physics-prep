#!/usr/bin/env python3
"""Run the test0 full normalize flow and normalize-only gate as one command.

This wrapper is intentionally thin. It delegates all normalize and gate logic
to existing repo scripts and only handles:

- path validation and run-id based directory layout
- step ordering
- per-step log capture
- synthetic pre_c1 snapshots required by the phase-2 audit
- final manifest / status emission

Default reports live under:
  check_reports/test0_full/<run_id>/

The canonical normalized dataset root defaults to:
  GRScenes-test0-normalized

This keeps the first-phase output aligned with the formal handoff contract:
one stable normalized dataset root plus one run-scoped report tree.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence


def _utc_run_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def _abs_path(path: str) -> Path:
    return Path(path).expanduser().resolve()


def _path_has_entries(path: Path) -> bool:
    return path.exists() and any(path.iterdir())


def _require_dir(path: Path, label: str) -> None:
    if not path.is_dir():
        raise FileNotFoundError(f"{label} directory not found: {path}")


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _count_immediate_dirs(path: Path) -> int:
    return sum(1 for child in path.iterdir() if child.is_dir())


def _collect_source_inventory(source_scenes: Path, source_assets: Path) -> Dict[str, int]:
    home_dir = source_scenes / "home"
    commercial_dir = source_scenes / "commercial"
    home_count = _count_immediate_dirs(home_dir) if home_dir.is_dir() else 0
    commercial_count = _count_immediate_dirs(commercial_dir) if commercial_dir.is_dir() else 0
    category_count = _count_immediate_dirs(source_assets)
    return {
        "category_count": category_count,
        "scene_count": home_count + commercial_count,
        "home_scene_count": home_count,
        "commercial_scene_count": commercial_count,
    }


def _print_header(title: str) -> None:
    bar = "=" * 72
    print()
    print(bar)
    print(title)
    print(bar)


def _run_step(
    *,
    name: str,
    cmd: Sequence[str],
    log_path: Path,
    cwd: Path,
    allowed_exit_codes: Sequence[int] = (0,),
) -> Dict[str, Any]:
    _print_header(f"STEP: {name}")
    command_text = shlex.join(str(part) for part in cmd)
    print(command_text)

    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.time()
    with log_path.open("w", encoding="utf-8") as log_handle:
        log_handle.write(f"$ {command_text}\n\n")
        process = subprocess.Popen(
            [str(part) for part in cmd],
            cwd=str(cwd),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdout is not None
        for line in process.stdout:
            sys.stdout.write(line)
            log_handle.write(line)
        exit_code = process.wait()

    elapsed = round(time.time() - started, 1)
    result = {
        "command": command_text,
        "cwd": str(cwd),
        "log_path": str(log_path),
        "exit_code": exit_code,
        "elapsed_seconds": elapsed,
        "allowed_exit_codes": list(allowed_exit_codes),
        "status": "ok" if exit_code in allowed_exit_codes else "failed",
    }
    print(f"[{name}] exit_code={exit_code} elapsed={elapsed}s log={log_path}")
    if exit_code not in allowed_exit_codes:
        raise RuntimeError(f"Step failed: {name} (exit code {exit_code})")
    return result


def _create_pre_c1_snapshots(
    *,
    normalized_root: Path,
    normalize_dir: Path,
    timestamp: str,
) -> Dict[str, Any]:
    _print_header("STEP: pre_c1_snapshot")
    scenes_root = normalized_root / "GRScenes100"
    snapshot_name = f"layout.pre_c1_normalize_only.{timestamp}.usd"
    created: List[Dict[str, str]] = []
    missing_layouts: List[str] = []

    for subcategory in ("home", "commercial"):
        base = scenes_root / subcategory
        if not base.is_dir():
            continue
        for scene_dir in sorted(path for path in base.iterdir() if path.is_dir()):
            layout_path = scene_dir / "layout.usd"
            if not layout_path.is_file():
                missing_layouts.append(f"{subcategory}/{scene_dir.name}")
                continue
            snapshot_path = scene_dir / snapshot_name
            if snapshot_path.exists():
                raise FileExistsError(f"Snapshot already exists: {snapshot_path}")
            shutil.copy2(layout_path, snapshot_path)
            created.append(
                {
                    "scene_id": f"{subcategory}/{scene_dir.name}",
                    "layout": str(layout_path),
                    "snapshot": str(snapshot_path),
                }
            )

    report = {
        "normalized_root": str(normalized_root),
        "scenes_root": str(scenes_root),
        "snapshot_name": snapshot_name,
        "created_count": len(created),
        "missing_layout_count": len(missing_layouts),
        "missing_layout_scenes": missing_layouts,
        "created_preview": created[:50],
    }
    report_path = normalize_dir / "pre_c1_snapshot_report.json"
    _write_json(report_path, report)
    print(f"Created {len(created)} pre_c1 snapshot(s)")
    print(f"Snapshot report: {report_path}")
    return {
        "status": "ok",
        "exit_code": 0,
        "elapsed_seconds": 0.0,
        "log_path": None,
        "report_path": str(report_path),
        "snapshot_name": snapshot_name,
        "created_count": len(created),
        "missing_layout_count": len(missing_layouts),
    }


def _build_parser(default_run_id: str, repo_root: Path) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default=default_run_id)
    parser.add_argument(
        "--source-root",
        default=str(repo_root / "GRScenes-test0"),
        help="Input test0 dataset root",
    )
    parser.add_argument(
        "--report-root",
        default=None,
        help="Run report directory; defaults to check_reports/test0_full/<run_id>",
    )
    parser.add_argument(
        "--normalized-root",
        default=None,
        help="Output normalized dataset root; defaults to GRScenes-test0-normalized",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=8,
        help="Worker count for pairwise/audit helpers",
    )
    parser.add_argument(
        "--scene-filter",
        default=None,
        help="Optional substring filter passed to pairwise/audit helpers",
    )
    parser.add_argument(
        "--symlink-copy",
        action="store_true",
        help="Pass --symlink-copy to normalize_asset_transforms.py",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the planned commands and output paths without executing them",
    )
    parser.add_argument(
        "--skip-inventory-check",
        action="store_true",
        help=(
            "Skip the fixed 114/99/69/30 inventory gate. "
            "Intended for smoke subsets only."
        ),
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    repo_root = Path(__file__).resolve().parent.parent
    default_run_id = _utc_run_id()
    args = _build_parser(default_run_id, repo_root).parse_args(argv)

    run_id = args.run_id
    source_root = _abs_path(args.source_root)
    report_root = _abs_path(
        args.report_root
        or str(repo_root / "check_reports" / "test0_full" / run_id)
    )
    normalized_root = _abs_path(
        args.normalized_root or str(repo_root / "GRScenes-test0-normalized")
    )

    source_assets = source_root / "GRScenes_assets"
    source_scenes = source_root / "GRScenes100"
    isaac_python = repo_root / "scripts" / "isaac_python.sh"
    logs_dir = report_root / "logs"
    normalize_dir = report_root / "normalize"
    phase1_dir = normalize_dir / "phase1"
    phase2_dir = normalize_dir / "phase2"
    summary_dir = report_root / "summary"
    pairwise_json = normalize_dir / "test0_vs_normalized_pre_dedup.json"
    audit_json = normalize_dir / "test0_normalize_phase2_audit.json"
    manifest_path = report_root / "run_manifest.json"
    final_status_path = summary_dir / "run_status.json"
    snapshot_timestamp = _utc_run_id()
    label = f"test0_full_normalize_{run_id}"

    _require_dir(source_root, "source root")
    _require_dir(source_assets, "source assets root")
    _require_dir(source_scenes, "source scenes root")
    if not isaac_python.is_file():
        raise FileNotFoundError(f"isaac runner not found: {isaac_python}")

    inventory = _collect_source_inventory(source_scenes, source_assets)
    expected_inventory = {
        "category_count": 114,
        "scene_count": 99,
        "home_scene_count": 69,
        "commercial_scene_count": 30,
    }
    if not args.skip_inventory_check and inventory != expected_inventory:
        raise RuntimeError(
            "Source inventory mismatch for GRScenes-test0: "
            f"observed={inventory} expected={expected_inventory}"
        )

    if _path_has_entries(report_root):
        raise RuntimeError(f"Report root already exists and is not empty: {report_root}")
    if _path_has_entries(normalized_root):
        raise RuntimeError(f"Normalized root already exists and is not empty: {normalized_root}")

    phase1_cmd: List[str] = [
        str(isaac_python),
        str(repo_root / "scripts" / "normalize_asset_transforms.py"),
        "--assets-root",
        str(source_assets),
        "--scenes-root",
        str(source_scenes),
        "--output-root",
        str(normalized_root),
        "--phase",
        "1",
        "--report-dir",
        str(phase1_dir),
    ]
    if args.symlink_copy:
        phase1_cmd.append("--symlink-copy")

    phase2_cmd: List[str] = [
        str(isaac_python),
        str(repo_root / "scripts" / "normalize_asset_transforms.py"),
        "--assets-root",
        str(source_assets),
        "--scenes-root",
        str(source_scenes),
        "--output-root",
        str(normalized_root),
        "--phase",
        "2",
        "--centers-dir",
        str(phase1_dir),
        "--report-dir",
        str(phase2_dir),
    ]

    if args.symlink_copy:
        phase2_cmd.append("--symlink-copy")

    pairwise_cmd: List[str] = [
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
        label,
        "--workers",
        str(args.workers),
        "--out",
        str(pairwise_json),
    ]
    if args.scene_filter:
        pairwise_cmd.extend(["--scene-filter", args.scene_filter])

    audit_cmd: List[str] = [
        str(isaac_python),
        str(repo_root / "scripts" / "audit_normalize_phase2.py"),
        "--source-root",
        str(source_root),
        "--normalized-root",
        str(normalized_root),
        "--centers-dir",
        str(phase1_dir),
        "--workers",
        str(args.workers),
        "--out",
        str(audit_json),
    ]
    if args.scene_filter:
        audit_cmd.extend(["--scene-filter", args.scene_filter])

    bundle_cmd: List[str] = [
        sys.executable,
        str(repo_root / "scripts" / "assemble_normalize_gate_bundle.py"),
        "--summary-dir",
        str(summary_dir),
        "--run-id",
        run_id,
        "--pairwise-report",
        str(pairwise_json),
        "--audit-report",
        str(audit_json),
        "--python",
        sys.executable,
    ]

    manifest: Dict[str, Any] = {
        "run_id": run_id,
        "repo_root": str(repo_root),
        "inputs": {
            "source_root": str(source_root),
            "source_assets": str(source_assets),
            "source_scenes": str(source_scenes),
            "source_inventory": inventory,
            "inventory_check_skipped": args.skip_inventory_check,
            "scene_filter": args.scene_filter,
            "workers": args.workers,
            "symlink_copy": args.symlink_copy,
        },
        "outputs": {
            "report_root": str(report_root),
            "normalized_root": str(normalized_root),
            "normalize_dir": str(normalize_dir),
            "phase1_report_dir": str(phase1_dir),
            "phase2_report_dir": str(phase2_dir),
            "pairwise_json": str(pairwise_json),
            "audit_json": str(audit_json),
            "summary_dir": str(summary_dir),
            "logs_dir": str(logs_dir),
        },
        "commands": {
            "phase1": shlex.join(phase1_cmd),
            "phase2": shlex.join(phase2_cmd),
            "pairwise": shlex.join(pairwise_cmd),
            "audit": shlex.join(audit_cmd),
            "bundle": shlex.join(bundle_cmd),
        },
        "steps": {},
        "status": "planned" if args.dry_run else "running",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
    }

    if args.dry_run:
        print(json.dumps(manifest, indent=2, ensure_ascii=False))
        return 0

    report_root.mkdir(parents=True, exist_ok=False)
    logs_dir.mkdir(parents=True, exist_ok=True)
    _write_json(manifest_path, manifest)

    bundle_result: Optional[Dict[str, Any]] = None
    try:
        manifest["steps"]["phase1"] = _run_step(
            name="phase1",
            cmd=phase1_cmd,
            log_path=logs_dir / "phase1.log",
            cwd=repo_root,
        )
        _write_json(manifest_path, manifest)

        manifest["steps"]["phase2"] = _run_step(
            name="phase2",
            cmd=phase2_cmd,
            log_path=logs_dir / "phase2.log",
            cwd=repo_root,
        )
        _write_json(manifest_path, manifest)

        manifest["steps"]["pre_c1_snapshot"] = _create_pre_c1_snapshots(
            normalized_root=normalized_root,
            normalize_dir=normalize_dir,
            timestamp=snapshot_timestamp,
        )
        _write_json(manifest_path, manifest)

        manifest["steps"]["pairwise"] = _run_step(
            name="pairwise",
            cmd=pairwise_cmd,
            log_path=logs_dir / "pairwise.log",
            cwd=repo_root,
        )
        _write_json(manifest_path, manifest)

        manifest["steps"]["audit"] = _run_step(
            name="audit",
            cmd=audit_cmd,
            log_path=logs_dir / "audit.log",
            cwd=repo_root,
        )
        _write_json(manifest_path, manifest)

        bundle_result = _run_step(
            name="bundle",
            cmd=bundle_cmd,
            log_path=logs_dir / "bundle.log",
            cwd=repo_root,
            allowed_exit_codes=(0, 1),
        )
        manifest["steps"]["bundle"] = bundle_result
        manifest["status"] = "completed"
        _write_json(manifest_path, manifest)
    except Exception as exc:
        manifest["status"] = "failed"
        manifest["error"] = str(exc)
        _write_json(manifest_path, manifest)
        final_status = {
            "run_id": run_id,
            "status": "failed",
            "error": str(exc),
            "manifest": str(manifest_path),
        }
        _write_json(final_status_path, final_status)
        print(str(exc), file=sys.stderr)
        return 2

    normalize_gate_path = summary_dir / "normalize_gate_verdict.json"
    final_verdict_path = summary_dir / "final_verdict.json"
    summary_md_path = summary_dir / "summary.md"
    normalize_gate = _load_json(normalize_gate_path) if normalize_gate_path.is_file() else {}
    final_verdict = _load_json(final_verdict_path) if final_verdict_path.is_file() else {}

    final_status = {
        "run_id": run_id,
        "status": "pass" if bundle_result and bundle_result["exit_code"] == 0 else "fail",
        "manifest": str(manifest_path),
        "normalize_gate_exit_code": bundle_result["exit_code"] if bundle_result else None,
        "normalize_gate_status": normalize_gate.get("status"),
        "final_verdict_status": final_verdict.get("status"),
        "summary_md": str(summary_md_path),
        "pairwise_json": str(pairwise_json),
        "audit_json": str(audit_json),
    }
    _write_json(final_status_path, final_status)

    print()
    print(f"Run manifest: {manifest_path}")
    print(f"Run status:   {final_status_path}")
    print(f"Summary dir:  {summary_dir}")
    return bundle_result["exit_code"] if bundle_result else 1


if __name__ == "__main__":
    raise SystemExit(main())
