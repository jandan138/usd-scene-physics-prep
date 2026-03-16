#!/usr/bin/env python3
"""Orchestrate rebuilt test0 smoke and full normalize runs.

This wrapper keeps the rebuilt normalize execution decision-complete:

- preflight the clean rebuilt source
- run a local smoke subset
- submit full Phase 1 to DLC per category
- collect Phase 1 status, run allowlist gate, merge centers
- submit full Phase 2 to DLC
- collect Phase 2 status and run post-processing / hard gate
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import shlex
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence


DEFAULT_WORKSPACE_ID = os.environ.get("DLC_WORKSPACE_ID", "270969")
TERMINAL_JOB_STATUSES = {"Succeeded", "Failed", "Stopped", "Completed"}
SUCCESS_JOB_STATUSES = {"Succeeded", "Completed"}


def _utc_run_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def _abs_path(value: str) -> Path:
    return Path(value).expanduser().resolve()


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _run(cmd: Sequence[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(part) for part in cmd],
        cwd=str(cwd),
        text=True,
        capture_output=True,
        check=True,
    )


def _load_preflight_module(repo_root: Path):
    script_path = repo_root / "scripts" / "preflight_test0_rebuilt_normalize.py"
    spec = importlib.util.spec_from_file_location("preflight_test0_rebuilt_normalize", script_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"failed to load preflight module: {script_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _main_usd_hashes(repo_root: Path, source_root: Path) -> List[Dict[str, str]]:
    preflight_mod = _load_preflight_module(repo_root)
    hashes: List[Dict[str, str]] = []
    for sample in preflight_mod.DEFAULT_SAMPLE_ASSETS:
        main_usd = preflight_mod._find_main_usd(source_root, sample["category"], sample["uid"])
        hashes.append(
            {
                "category": sample["category"],
                "uid": sample["uid"],
                "path": str(main_usd),
                "sha256": preflight_mod._sha256_file(main_usd),
            }
        )
    return hashes


def _verify_hashes(repo_root: Path, source_root: Path, expected: Sequence[Dict[str, str]]) -> Dict[str, Any]:
    current = _main_usd_hashes(repo_root, source_root)
    mismatches: List[Dict[str, str]] = []
    expected_map = {(item["category"], item["uid"]): item["sha256"] for item in expected}
    for item in current:
        key = (item["category"], item["uid"])
        expected_sha = expected_map.get(key)
        if expected_sha != item["sha256"]:
            mismatches.append(
                {
                    "category": item["category"],
                    "uid": item["uid"],
                    "expected_sha256": expected_sha or "",
                    "observed_sha256": item["sha256"],
                }
            )
    return {
        "status": "pass" if not mismatches else "fail",
        "expected": list(expected),
        "observed": current,
        "mismatches": mismatches,
    }


def _parse_dlc_job_table(text: str) -> List[Dict[str, str]]:
    lines = [line.rstrip() for line in text.splitlines() if line.strip()]
    if len(lines) < 4:
        return []
    rows: List[Dict[str, str]] = []
    headers: Optional[List[str]] = None
    for line in lines:
        if not line.startswith("|"):
            continue
        cells = [cell.strip() for cell in line.strip("|").split("|")]
        if headers is None:
            headers = cells
            continue
        if headers == cells:
            continue
        row = {header: value for header, value in zip(headers, cells)}
        if row:
            rows.append(row)
    return rows


def _get_dlc_job_by_name(job_name: str, workspace_id: str) -> Dict[str, Any]:
    repo_root = Path(__file__).resolve().parent.parent
    cmd = [
        str(repo_root / "dlc"),
        "get",
        "job",
        "--workspace_id",
        workspace_id,
        "--display_name_regex",
        f"^{job_name}$",
        "--page_size",
        "20",
    ]
    result = _run(cmd, repo_root)
    rows = _parse_dlc_job_table(result.stdout)
    if not rows:
        return {"status": "missing", "job_name": job_name}
    row = rows[0]
    return {
        "status": row.get("JobStatus", ""),
        "job_name": row.get("Name", job_name),
        "job_id": row.get("JobId", ""),
        "create_time": row.get("CreateTime", ""),
        "finish_time": row.get("FinishTime", ""),
        "raw": row,
    }


def _derive_phase1_job_names(categories: Sequence[str], run_id: str) -> List[str]:
    return [f"norm_p1_{run_id}_{category}_0_1" for category in categories]


def _discover_categories(source_root: Path) -> List[str]:
    assets_root = source_root / "GRScenes_assets"
    return sorted(
        child.name
        for child in assets_root.iterdir()
        if child.is_dir() and not child.name.startswith(".")
    )


def _build_paths(repo_root: Path, source_root: Path, run_id: str) -> Dict[str, str]:
    smoke_source = repo_root / "GRScenes-test0-rebuilt-smoke-S1"
    smoke_normalized = repo_root / "GRScenes-test0-rebuilt-smoke-S1-normalized"
    smoke_report = repo_root / "check_reports" / "test0_rebuilt_smoke" / run_id
    full_report = repo_root / "check_reports" / "test0_rebuilt_full" / run_id
    full_normalized = repo_root / "GRScenes-test0-rebuilt-normalized"
    phase1_root = full_report / "normalize" / "phase1"
    phase2_root = full_report / "normalize" / "phase2"
    verify_root = full_report / "normalize"
    summary_root = full_report / "summary"
    return {
        "source_root": str(source_root),
        "smoke_source_root": str(smoke_source),
        "smoke_normalized_root": str(smoke_normalized),
        "smoke_report_root": str(smoke_report),
        "full_report_root": str(full_report),
        "full_normalized_root": str(full_normalized),
        "phase1_root": str(phase1_root),
        "phase2_root": str(phase2_root),
        "verify_root": str(verify_root),
        "summary_root": str(summary_root),
    }


def _manifest_path(report_root: Path) -> Path:
    return report_root / "run_manifest.json"


def _default_manifest(repo_root: Path, source_root: Path, run_id: str) -> Dict[str, Any]:
    return {
        "run_id": run_id,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "repo_root": str(repo_root),
        "paths": _build_paths(repo_root, source_root, run_id),
        "stages": {},
    }


def _load_or_init_manifest(repo_root: Path, source_root: Path, run_id: str) -> tuple[Path, Dict[str, Any]]:
    full_report_root = Path(_build_paths(repo_root, source_root, run_id)["full_report_root"])
    manifest_path = _manifest_path(full_report_root)
    if manifest_path.is_file():
        return manifest_path, _load_json(manifest_path)
    manifest = _default_manifest(repo_root, source_root, run_id)
    return manifest_path, manifest


def cmd_smoke(args: argparse.Namespace) -> int:
    repo_root = Path(__file__).resolve().parent.parent
    source_root = _abs_path(args.source_root)
    run_id = args.run_id or _utc_run_id()
    manifest_path, manifest = _load_or_init_manifest(repo_root, source_root, run_id)
    paths = manifest["paths"]
    full_report_root = Path(paths["full_report_root"])
    full_report_root.mkdir(parents=True, exist_ok=True)

    smoke_report_root = _abs_path(paths["smoke_report_root"])
    smoke_preflight_root = smoke_report_root / "preflight"
    smoke_source_root = _abs_path(paths["smoke_source_root"])
    smoke_normalized_root = _abs_path(paths["smoke_normalized_root"])

    preflight_cmd = [
        sys.executable,
        str(repo_root / "scripts" / "preflight_test0_rebuilt_normalize.py"),
        "--source-root",
        str(source_root),
        "--normalized-root",
        str(smoke_normalized_root),
        "--report-root",
        str(smoke_preflight_root),
    ]
    _run(preflight_cmd, repo_root)
    baseline_hashes = _main_usd_hashes(repo_root, source_root)

    subset_cmd = [
        str(repo_root / "scripts" / "isaac_python.sh"),
        str(repo_root / "scripts" / "build_scene_uid_subset_package.py"),
        "--src",
        str(source_root),
        "--dst",
        str(smoke_source_root),
        "--scene-uid",
        args.scene_uid,
        "--scene-category",
        args.scene_category,
        "--write-manifest",
        "--verify",
    ]
    normalize_root = smoke_report_root / "normalize"
    phase1_root = normalize_root / "phase1"
    phase2_root = normalize_root / "phase2"
    pairwise_json = normalize_root / "test0_vs_normalized_pre_dedup.json"
    audit_json = normalize_root / "test0_normalize_phase2_audit.json"
    summary_root = smoke_report_root / "summary"
    phase1_cmd = [
        str(repo_root / "scripts" / "isaac_python.sh"),
        str(repo_root / "scripts" / "normalize_asset_transforms.py"),
        "--assets-root",
        str(smoke_source_root / "GRScenes_assets"),
        "--scenes-root",
        str(smoke_source_root / "GRScenes100"),
        "--output-root",
        str(smoke_normalized_root),
        "--phase",
        "1",
        "--symlink-copy",
        "--report-dir",
        str(phase1_root),
    ]
    phase2_cmd = [
        str(repo_root / "scripts" / "isaac_python.sh"),
        str(repo_root / "scripts" / "normalize_asset_transforms.py"),
        "--assets-root",
        str(smoke_source_root / "GRScenes_assets"),
        "--scenes-root",
        str(smoke_source_root / "GRScenes100"),
        "--output-root",
        str(smoke_normalized_root),
        "--phase",
        "2",
        "--centers-dir",
        str(phase1_root),
        "--symlink-copy",
        "--report-dir",
        str(phase2_root),
    ]
    snapshot_cmd = [
        sys.executable,
        str(repo_root / "scripts" / "create_pre_c1_normalize_only_snapshots.py"),
        "--normalized-root",
        str(smoke_normalized_root),
        "--run-id",
        f"{run_id}_smoke",
        "--report-path",
        str(normalize_root / "pre_c1_snapshot_report.json"),
    ]
    pairwise_cmd = [
        str(repo_root / "scripts" / "isaac_python.sh"),
        str(repo_root / "scripts" / "placement_pairwise_compare.py"),
        "--left-root",
        str(smoke_source_root),
        "--right-root",
        str(smoke_normalized_root),
        "--left-mode",
        "current",
        "--right-mode",
        "current",
        "--label",
        f"test0_rebuilt_smoke_{run_id}",
        "--workers",
        str(args.smoke_workers),
        "--out",
        str(pairwise_json),
    ]
    audit_cmd = [
        str(repo_root / "scripts" / "isaac_python.sh"),
        str(repo_root / "scripts" / "audit_normalize_phase2.py"),
        "--source-root",
        str(smoke_source_root),
        "--normalized-root",
        str(smoke_normalized_root),
        "--centers-dir",
        str(phase1_root),
        "--workers",
        str(args.smoke_workers),
        "--out",
        str(audit_json),
    ]
    smoke_allowlist_json = phase1_root / "allowlist_verdict.json"
    bundle_cmd = [
        sys.executable,
        str(repo_root / "scripts" / "assemble_normalize_gate_bundle.py"),
        "--summary-dir",
        str(summary_root),
        "--run-id",
        f"{run_id}_smoke",
        "--pairwise-report",
        str(pairwise_json),
        "--audit-report",
        str(audit_json),
        "--python",
        sys.executable,
    ]
    if smoke_allowlist_json.is_file():
        bundle_cmd.extend(["--allowlist-json", str(smoke_allowlist_json)])
    _run(subset_cmd, repo_root)
    smoke_outputs: Dict[str, Any] = {}
    smoke_outputs["phase1"] = _run(phase1_cmd, repo_root).stdout
    smoke_outputs["phase2"] = _run(phase2_cmd, repo_root).stdout
    smoke_outputs["snapshot"] = _run(snapshot_cmd, repo_root).stdout
    smoke_outputs["pairwise"] = _run(pairwise_cmd, repo_root).stdout
    smoke_outputs["audit"] = _run(audit_cmd, repo_root).stdout
    bundle = subprocess.run(
        [str(part) for part in bundle_cmd],
        cwd=str(repo_root),
        text=True,
        capture_output=True,
    )
    if bundle.returncode not in (0, 1):
        raise RuntimeError(bundle.stderr or bundle.stdout or "smoke bundle failed")
    hash_verification = _verify_hashes(repo_root, source_root, baseline_hashes)

    manifest["stages"]["smoke"] = {
        "status": "pass" if hash_verification["status"] == "pass" and bundle.returncode == 0 else "fail",
        "scene_uid": args.scene_uid,
        "scene_category": args.scene_category,
        "smoke_report_root": str(smoke_report_root),
        "outputs": smoke_outputs,
        "bundle_stdout": bundle.stdout,
        "bundle_stderr": bundle.stderr,
        "bundle_exit_code": bundle.returncode,
        "source_hash_verification": hash_verification,
    }
    _write_json(manifest_path, manifest)
    return 0 if manifest["stages"]["smoke"]["status"] == "pass" else 1


def cmd_submit_full(args: argparse.Namespace) -> int:
    repo_root = Path(__file__).resolve().parent.parent
    source_root = _abs_path(args.source_root)
    run_id = args.run_id
    manifest_path, manifest = _load_or_init_manifest(repo_root, source_root, run_id)
    paths = manifest["paths"]
    full_report_root = _abs_path(paths["full_report_root"])
    phase1_root = _abs_path(paths["phase1_root"])
    full_normalized_root = _abs_path(paths["full_normalized_root"])
    full_report_root.mkdir(parents=True, exist_ok=True)

    smoke_stage = manifest["stages"].get("smoke")
    if not args.skip_smoke_gate and smoke_stage and smoke_stage.get("status") != "pass":
        raise RuntimeError("smoke stage did not pass; refusing to submit full run")

    preflight_root = full_report_root / "preflight"
    preflight_cmd = [
        sys.executable,
        str(repo_root / "scripts" / "preflight_test0_rebuilt_normalize.py"),
        "--source-root",
        str(source_root),
        "--normalized-root",
        str(full_normalized_root),
        "--report-root",
        str(preflight_root),
    ]
    _run(preflight_cmd, repo_root)
    baseline_hashes = _main_usd_hashes(repo_root, source_root)

    submit_cmd = [
        "bash",
        str(repo_root / "scripts" / "dlc" / "submit_normalize_phase1.sh"),
        "--assets-root",
        str(source_root / "GRScenes_assets"),
        "--scenes-root",
        str(source_root / "GRScenes100"),
        "--output-root",
        str(full_normalized_root),
        "--report-dir",
        str(phase1_root),
        "--run-label",
        run_id,
    ]
    if args.data_sources:
        submit_cmd.extend(["--data-sources", args.data_sources])
    result = _run(submit_cmd, repo_root)
    categories = _discover_categories(source_root)
    manifest["stages"]["phase1_submit"] = {
        "status": "submitted",
        "submitted_at_utc": datetime.now(timezone.utc).isoformat(),
        "command": shlex.join(submit_cmd),
        "categories": categories,
        "job_names": _derive_phase1_job_names(categories, run_id),
        "stdout": result.stdout,
        "stderr": result.stderr,
        "source_hashes_before_full": baseline_hashes,
    }
    _write_json(manifest_path, manifest)
    return 0


def _phase1_collect(repo_root: Path, manifest: Dict[str, Any], workspace_id: str) -> Dict[str, Any]:
    phase1_submit = manifest["stages"].get("phase1_submit")
    if not phase1_submit:
        raise RuntimeError("phase1_submit stage missing")
    job_statuses = {
        job_name: _get_dlc_job_by_name(job_name, workspace_id)
        for job_name in phase1_submit["job_names"]
    }
    nonterminal = {
        name: info for name, info in job_statuses.items()
        if info["status"] not in TERMINAL_JOB_STATUSES
    }
    failures = {
        name: info for name, info in job_statuses.items()
        if info["status"] in TERMINAL_JOB_STATUSES and info["status"] not in SUCCESS_JOB_STATUSES
    }
    if nonterminal:
        return {"status": "waiting", "jobs": job_statuses}
    if failures:
        return {"status": "fail", "jobs": job_statuses, "failures": failures}

    phase1_root = Path(manifest["paths"]["phase1_root"])
    allowlist_json = phase1_root / "allowlist_verdict.json"
    allowlist_cmd = [
        sys.executable,
        str(repo_root / "scripts" / "check_test0_phase1_allowlist.py"),
        "--phase1-root",
        str(phase1_root),
        "--json-out",
        str(allowlist_json),
    ]
    _run(allowlist_cmd, repo_root)

    centers_merged = phase1_root / "centers_merged"
    merge_cmd = [
        sys.executable,
        str(repo_root / "scripts" / "merge_phase1_centers.py"),
        "--phase1-root",
        str(phase1_root),
        "--out-dir",
        str(centers_merged),
        "--mode",
        "copy",
    ]
    _run(merge_cmd, repo_root)
    return {
        "status": "pass",
        "jobs": job_statuses,
        "allowlist_json": str(allowlist_json),
        "centers_merged": str(centers_merged),
    }


def _submit_phase2(repo_root: Path, manifest: Dict[str, Any], data_sources: Optional[str]) -> Dict[str, Any]:
    paths = manifest["paths"]
    submit_cmd = [
        "bash",
        str(repo_root / "scripts" / "dlc" / "submit_normalize_phase2.sh"),
        "--assets-root",
        str(Path(paths["source_root"]) / "GRScenes_assets"),
        "--scenes-root",
        str(Path(paths["source_root"]) / "GRScenes100"),
        "--output-root",
        paths["full_normalized_root"],
        "--report-dir",
        paths["phase2_root"],
        "--centers-dir",
        str(Path(paths["phase1_root"]) / "centers_merged"),
        "--run-label",
        manifest["run_id"],
    ]
    if data_sources:
        submit_cmd.extend(["--data-sources", data_sources])
    result = _run(submit_cmd, repo_root)
    return {
        "status": "submitted",
        "job_name": f"norm_p2_{manifest['run_id']}_scenes_0_1",
        "command": shlex.join(submit_cmd),
        "stdout": result.stdout,
        "stderr": result.stderr,
    }


def _collect_phase2(repo_root: Path, manifest: Dict[str, Any], workspace_id: str) -> Dict[str, Any]:
    phase2_submit = manifest["stages"].get("phase2_submit")
    if not phase2_submit:
        raise RuntimeError("phase2_submit stage missing")
    job = _get_dlc_job_by_name(phase2_submit["job_name"], workspace_id)
    if job["status"] not in TERMINAL_JOB_STATUSES:
        return {"status": "waiting", "job": job}
    if job["status"] not in SUCCESS_JOB_STATUSES:
        return {"status": "fail", "job": job}

    paths = manifest["paths"]
    full_report_root = Path(paths["full_report_root"])
    full_normalized_root = Path(paths["full_normalized_root"])
    phase2_root = Path(paths["phase2_root"])
    pairwise_json = full_report_root / "normalize" / "test0_vs_normalized_pre_dedup.json"
    audit_json = full_report_root / "normalize" / "test0_normalize_phase2_audit.json"
    summary_root = Path(paths["summary_root"])

    snapshot_cmd = [
        sys.executable,
        str(repo_root / "scripts" / "create_pre_c1_normalize_only_snapshots.py"),
        "--normalized-root",
        str(full_normalized_root),
        "--run-id",
        manifest["run_id"],
        "--report-path",
        str(full_report_root / "normalize" / "pre_c1_snapshot_report.json"),
    ]
    pairwise_cmd = [
        str(repo_root / "scripts" / "isaac_python.sh"),
        str(repo_root / "scripts" / "placement_pairwise_compare.py"),
        "--left-root",
        paths["source_root"],
        "--right-root",
        paths["full_normalized_root"],
        "--left-mode",
        "current",
        "--right-mode",
        "current",
        "--label",
        f"test0_rebuilt_full_{manifest['run_id']}",
        "--workers",
        "8",
        "--out",
        str(pairwise_json),
    ]
    audit_cmd = [
        str(repo_root / "scripts" / "isaac_python.sh"),
        str(repo_root / "scripts" / "audit_normalize_phase2.py"),
        "--source-root",
        paths["source_root"],
        "--normalized-root",
        paths["full_normalized_root"],
        "--centers-dir",
        str(Path(paths["phase1_root"]) / "centers_merged"),
        "--workers",
        "8",
        "--out",
        str(audit_json),
    ]
    bundle_cmd = [
        sys.executable,
        str(repo_root / "scripts" / "assemble_normalize_gate_bundle.py"),
        "--summary-dir",
        str(summary_root),
        "--run-id",
        manifest["run_id"],
        "--pairwise-report",
        str(pairwise_json),
        "--audit-report",
        str(audit_json),
        "--python",
        sys.executable,
    ]
    if allowlist_json.is_file():
        bundle_cmd.extend(["--allowlist-json", str(allowlist_json)])
    _run(snapshot_cmd, repo_root)
    _run(pairwise_cmd, repo_root)
    _run(audit_cmd, repo_root)
    bundle = subprocess.run(
        [str(part) for part in bundle_cmd],
        cwd=str(repo_root),
        text=True,
        capture_output=True,
    )
    if bundle.returncode not in (0, 1):
        raise RuntimeError(bundle.stderr or bundle.stdout or "bundle command failed")

    hash_verification = _verify_hashes(
        repo_root,
        Path(paths["source_root"]),
        manifest["stages"]["phase1_submit"]["source_hashes_before_full"],
    )
    return {
        "status": "pass" if bundle.returncode == 0 and hash_verification["status"] == "pass" else "fail",
        "job": job,
        "bundle_exit_code": bundle.returncode,
        "bundle_stdout": bundle.stdout,
        "bundle_stderr": bundle.stderr,
        "source_hash_verification": hash_verification,
        "phase2_report": str(phase2_root / "normalize_report.json"),
        "pairwise_report": str(pairwise_json),
        "audit_report": str(audit_json),
        "summary_root": str(summary_root),
    }


def cmd_resume_full(args: argparse.Namespace) -> int:
    repo_root = Path(__file__).resolve().parent.parent
    source_root = _abs_path(args.source_root)
    manifest_path, manifest = _load_or_init_manifest(repo_root, source_root, args.run_id)

    if "phase1_collect" not in manifest["stages"] or manifest["stages"]["phase1_collect"]["status"] == "waiting":
        phase1_collect = _phase1_collect(repo_root, manifest, args.workspace_id)
        manifest["stages"]["phase1_collect"] = phase1_collect
        if phase1_collect["status"] != "pass":
            _write_json(manifest_path, manifest)
            return 0 if phase1_collect["status"] == "waiting" else 1

    if "phase2_submit" not in manifest["stages"]:
        manifest["stages"]["phase2_submit"] = _submit_phase2(repo_root, manifest, args.data_sources)
        _write_json(manifest_path, manifest)
        return 0

    phase2_collect = _collect_phase2(repo_root, manifest, args.workspace_id)
    manifest["stages"]["phase2_collect"] = phase2_collect
    manifest["stages"]["final"] = {
        "status": phase2_collect["status"],
        "updated_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    _write_json(manifest_path, manifest)
    return 0 if phase2_collect["status"] == "pass" else (0 if phase2_collect["status"] == "waiting" else 1)


def cmd_status(args: argparse.Namespace) -> int:
    repo_root = Path(__file__).resolve().parent.parent
    source_root = _abs_path(args.source_root)
    manifest_path, manifest = _load_or_init_manifest(repo_root, source_root, args.run_id)
    print(f"Manifest: {manifest_path}")
    print(json.dumps(manifest.get("stages", {}), indent=2, ensure_ascii=False))
    return 0


def build_parser() -> argparse.ArgumentParser:
    repo_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    smoke = sub.add_parser("smoke", help="Run rebuilt smoke subset locally")
    smoke.add_argument("--run-id", default=None)
    smoke.add_argument("--source-root", default=str(repo_root / "GRScenes-test0-rebuilt"))
    smoke.add_argument("--scene-uid", default="MV7J6NIKTKJZ2AABAAAAADA8")
    smoke.add_argument("--scene-category", default="home")
    smoke.add_argument("--smoke-workers", type=int, default=1)
    smoke.set_defaults(func=cmd_smoke)

    submit = sub.add_parser("submit-full", help="Preflight and submit full Phase 1 to DLC")
    submit.add_argument("--run-id", required=True)
    submit.add_argument("--source-root", default=str(repo_root / "GRScenes-test0-rebuilt"))
    submit.add_argument("--data-sources", default=None)
    submit.add_argument("--skip-smoke-gate", action="store_true")
    submit.set_defaults(func=cmd_submit_full)

    resume = sub.add_parser("resume-full", help="Advance full run after DLC jobs complete")
    resume.add_argument("--run-id", required=True)
    resume.add_argument("--source-root", default=str(repo_root / "GRScenes-test0-rebuilt"))
    resume.add_argument("--workspace-id", default=DEFAULT_WORKSPACE_ID)
    resume.add_argument("--data-sources", default=None)
    resume.set_defaults(func=cmd_resume_full)

    status = sub.add_parser("status", help="Show current run manifest stages")
    status.add_argument("--run-id", required=True)
    status.add_argument("--source-root", default=str(repo_root / "GRScenes-test0-rebuilt"))
    status.set_defaults(func=cmd_status)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
