#!/usr/bin/env python3
"""Orchestrate C1 parallel pipeline DLC jobs.

Commands:
  submit-phase1  Submit one DLC job per remaining category
  gate-check     Verify all Phase 1 jobs completed (phase1_done.json)
  submit-phase2  Submit the Phase 2 merge+scan+delete job
  status         Print completion status table

Mock submit mode (dry run):
  DLC_BIN=echo ./scripts/orchestrate_c1_parallel.py submit-phase1 ...
"""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

DEFAULT_DATA_SOURCES = (
    "d-mzps5b7joy2axmqpa8,d-d49o5g0h2818sw8j1g,"
    "d-8wz4emfs21s5ajs9oz,d-f1dsz5nbamclxgydo8"
)

SCRIPTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPTS_DIR.parent


def get_dlc_bin() -> str:
    return os.environ.get("DLC_BIN", str(REPO_ROOT / "dlc"))


def is_mock_mode() -> bool:
    dlc_bin = os.environ.get("DLC_BIN", "")
    return dlc_bin in ("echo", "/bin/echo", "/usr/bin/echo")


def _resolved_dlc_bin_env(env: Dict[str, str]) -> None:
    """Resolve DLC_BIN=echo to a real binary path for launch_job.sh -x check."""
    dlc_bin = env.get("DLC_BIN", "")
    if dlc_bin in ("echo", "/bin/echo", "/usr/bin/echo"):
        env["DLC_BIN"] = "/usr/bin/echo"


def launch_job(
    job_name: str,
    chunk_id: int,
    chunk_total: int,
    data_sources: str,
    command_args: str,
    env_extra: Optional[Dict[str, str]] = None,
) -> subprocess.CompletedProcess:
    cmd = [
        "bash",
        str(SCRIPTS_DIR / "dlc" / "launch_job.sh"),
        job_name,
        str(chunk_id),
        str(chunk_total),
        data_sources,
        command_args,
    ]
    env = os.environ.copy()
    _resolved_dlc_bin_env(env)
    if env_extra:
        env.update(env_extra)
    return subprocess.run(cmd, capture_output=True, text=True, env=env)


def _discover_categories_from_report(report_path: str) -> List[str]:
    """Extract category names from dedup report using streaming JSON parser.

    The report format is: {"meta": ..., "duplicates": [...], "assets": ..., "errors": ...}
    """
    import ijson
    cats: set = set()
    with open(report_path, "rb") as f:
        for group in ijson.items(f, "duplicates.item"):
            for p in group.get("usd_paths", []):
                parts = Path(p).parts
                for i, part in enumerate(parts):
                    if part == "GRScenes_assets" and i + 1 < len(parts):
                        cats.add(parts[i + 1])
                        break
    return sorted(cats)


def _discover_categories_from_disk(
    c1_bulk_dir: str, bbox_policy: str, out_version: str
) -> List[str]:
    """Discover categories from directory names in c1_bulk."""
    cats: List[str] = []
    p = Path(c1_bulk_dir)
    if not p.exists():
        return cats
    for d in sorted(p.iterdir()):
        if not d.is_dir():
            continue
        name = d.name
        suffix = f"_{bbox_policy}_{out_version}"
        if name.endswith(suffix):
            cats.append(name[: -len(suffix)])
    return cats


def _discover_completed_categories(
    c1_bulk_dir: str, bbox_policy: str, out_version: str
) -> List[str]:
    """Discover categories with valid phase1_done.json (status=ok, audit_passed=true)."""
    sys.path.insert(0, str(SCRIPTS_DIR))
    from c1_parallel_merge import gate_check_phase1  # noqa: E402

    all_cats = _discover_categories_from_disk(c1_bulk_dir, bbox_policy, out_version)
    ok, failed = gate_check_phase1(Path(c1_bulk_dir), bbox_policy, out_version, all_cats)
    return [c for c in all_cats if c not in {f.split(":")[0] for f in failed}]


def submit_phase1(args: argparse.Namespace):
    """Submit Phase 1 DLC jobs (one per remaining category)."""
    dataset_root = args.dataset_root
    bak_root = args.bak_root
    report_path = args.report
    c1_bulk_dir = args.c1_bulk_dir
    group_label = args.group_label
    bbox_policy = args.bbox_policy
    out_version = args.out_version

    all_cats = _discover_categories_from_report(report_path)
    done_cats = _discover_completed_categories(c1_bulk_dir, bbox_policy, out_version)
    remaining = [c for c in all_cats if c not in done_cats]

    print(f"Total categories in report: {len(all_cats)}")
    print(f"Already completed: {len(done_cats)}")
    print(f"Remaining to process: {len(remaining)}")
    if remaining:
        preview = ", ".join(remaining[:5])
        if len(remaining) > 5:
            preview += f", ... (and {len(remaining) - 5} more)"
        print(f"Categories: {preview}")

    if not remaining:
        print("No remaining categories to process.")
        return

    if not args.yes:
        resp = input("Proceed with submission? [y/N]: ")
        if resp.lower() != "y":
            print("Aborted.")
            return

    if not args.mode_reports_dir:
        print("ERROR: --mode-reports-dir is required (needed for per-pair mode lookup when --bbox-gated)")
        print("Example: --mode-reports-dir check_reports/test0_rebuilt_dedup/v8_prededup")
        sys.exit(1)

    if is_mock_mode():
        print("=== MOCK MODE (DLC_BIN=echo) — commands will be printed, not submitted ===")

    c1_bulk_path = Path(c1_bulk_dir)
    c1_bulk_path.mkdir(parents=True, exist_ok=True)

    cats_file = c1_bulk_path / "phase2_categories.json"
    cats_file.write_text(json.dumps(remaining, indent=2) + "\n", encoding="utf-8")

    manifest: Dict = {"jobs": {}, "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S")}

    script_path = str(SCRIPTS_DIR / "c1_phase1_apply_and_audit.py")
    command_args = (
        f"custom {script_path} "
        f"--category {{CAT}} "
        f"--dataset-root {dataset_root} "
        f"--bak-root {bak_root} "
        f"--report {report_path} "
        f"--c1-bulk-dir {c1_bulk_dir} "
        f"--group-label {group_label} "
        f"--bbox-gated --bbox-policy {bbox_policy} "
        f"--dedup-mode geom_only --v-matrix-mode auto "
        f"--out-version {out_version}"
    )
    if args.mode_reports_dir:
        command_args += f" --mode-reports-dir {args.mode_reports_dir}"

    submitted = 0
    failed = 0
    for i, cat in enumerate(remaining):
        job_name = f"test0_parallel_phase1_{cat}"
        cmd_args = command_args.replace("{CAT}", cat)
        print(f"[{i + 1}/{len(remaining)}] Submitting {job_name}...", end=" ", flush=True)
        r = launch_job(job_name, 0, 1, args.data_sources, cmd_args)
        if r.returncode == 0:
            print("OK")
            submitted += 1
            manifest["jobs"][cat] = {"name": job_name, "status": "submitted"}
        else:
            print(f"FAILED: {r.stderr[:200]}")
            failed += 1
            manifest["jobs"][cat] = {
                "name": job_name,
                "status": "failed",
                "error": r.stderr[:500],
            }

    manifest_path = c1_bulk_path / "phase1_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"\nSubmitted: {submitted}, Failed: {failed}")
    print(f"Manifest: {manifest_path}")
    print(f"Categories list for Phase 2: {cats_file}")


def gate_check(args: argparse.Namespace) -> int:
    """Check Phase 1 completion status."""
    sys.path.insert(0, str(SCRIPTS_DIR))
    from c1_parallel_merge import gate_check_phase1  # noqa: E402

    cats_file = Path(args.c1_bulk_dir) / "phase2_categories.json"
    if not cats_file.exists():
        print("ERROR: phase2_categories.json not found. Run submit-phase1 first.")
        return 1

    categories = json.loads(cats_file.read_text(encoding="utf-8"))
    ok, failed = gate_check_phase1(
        Path(args.c1_bulk_dir), args.bbox_policy, args.out_version, categories
    )

    if ok:
        print(f"GATE PASSED — all {len(categories)} Phase 1 jobs completed successfully")
        print("Ready to submit Phase 2. Run:")
        print(f"  {sys.argv[0]} submit-phase2 ...")
        return 0
    else:
        completed = len(categories) - len(failed)
        print(f"GATE FAILED — {completed}/{len(categories)} complete, {len(failed)} not ready:")
        for f in failed:
            print(f"  - {f}")
        return 1


def submit_phase2(args: argparse.Namespace) -> int:
    """Submit Phase 2 job."""
    cats_file = Path(args.c1_bulk_dir) / "phase2_categories.json"
    if not cats_file.exists():
        print("ERROR: phase2_categories.json not found. Run submit-phase1 first.")
        return 1

    categories = json.loads(cats_file.read_text(encoding="utf-8"))
    print(f"Phase 2 will process {len(categories)} categories")

    if is_mock_mode():
        print("=== MOCK MODE (DLC_BIN=echo) — commands will be printed, not submitted ===")

    script_path = str(SCRIPTS_DIR / "c1_phase2_merge_scan_delete.py")
    command_args = (
        f"custom {script_path} "
        f"--dataset-root {args.dataset_root} "
        f"--bak-root {args.bak_root} "
        f"--c1-bulk-dir {args.c1_bulk_dir} "
        f"--bbox-policy {args.bbox_policy} "
        f"--out-version {args.out_version} "
        f"--group-label {args.group_label} "
        f"--categories-file {cats_file}"
    )
    if args.mode_reports_dir:
        command_args += f" --mode-reports-dir {args.mode_reports_dir}"

    print("Submitting Phase 2 job (timeout=480min)...")
    r = launch_job(
        "test0_parallel_phase2_merge", 0, 1, args.data_sources, command_args,
        env_extra={"DLC_JOB_TIMEOUT": "480"},
    )
    if r.returncode == 0:
        print("Phase 2 job submitted successfully")
        return 0
    else:
        print(f"Submission failed: {r.stderr[:500]}")
        return 1


def status_cmd(args: argparse.Namespace):
    """Print completion status table."""
    sys.path.insert(0, str(SCRIPTS_DIR))
    from c1_parallel_merge import gate_check_phase1  # noqa: E402

    cats_file = Path(args.c1_bulk_dir) / "phase2_categories.json"
    if cats_file.exists():
        categories = json.loads(cats_file.read_text(encoding="utf-8"))
    else:
        categories = _discover_categories_from_disk(
            args.c1_bulk_dir, args.bbox_policy, args.out_version
        )

    c1_bulk_path = Path(args.c1_bulk_dir)

    # Phase 2 status
    merge_report = c1_bulk_path / "phase2_merge_report.json"
    if merge_report.exists():
        report = json.loads(merge_report.read_text(encoding="utf-8"))
        p2_status = report.get("status", "unknown")
        p2_ts = report.get("timestamp", "")
        print(f"Phase 2: {p2_status} ({p2_ts})")
        if "categories_processed" in report:
            print(f"  Categories processed: {report['categories_processed']}")
        if "hit_files" in report:
            print(f"  Old references found (pre-delete): {report['hit_files']}")
        if "post_hits" in report:
            print(f"  Old references found (post-delete): {report['post_hits']}")
        print()
    else:
        print("Phase 2: not started\n")

    # Phase 1 status
    if not categories:
        print("Phase 1: no categories discovered")
        return

    ok, failed = gate_check_phase1(
        c1_bulk_path, args.bbox_policy, args.out_version, categories
    )

    completed = len(categories) - len(failed)
    print(f"Phase 1: {completed}/{len(categories)} complete")

    if failed:
        print("Pending/failed:")
        for f in failed:
            print(f"  - {f}")
        print()

    if ok:
        print("All Phase 1 jobs passed.")

    # Print detail table
    print(f"\n{'Category':<25} {'Status':<12} {'Audit':<8}")
    print("-" * 47)
    for cat in categories:
        done_file = (
            c1_bulk_path
            / f"{cat}_{args.bbox_policy}_{args.out_version}"
            / "phase1_done.json"
        )
        if done_file.exists():
            try:
                payload = json.loads(done_file.read_text(encoding="utf-8"))
                st = payload.get("status", "?")
                ap = "PASS" if payload.get("audit_passed", False) else "FAIL"
                print(f"{cat:<25} {st:<12} {ap:<8}")
            except Exception:
                print(f"{cat:<25} {'error':<12} {'?':<8}")
        else:
            print(f"{cat:<25} {'pending':<12} {'?':<8}")


def main():
    ap = argparse.ArgumentParser(
        description="C1 Parallel Pipeline Orchestrator",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s submit-phase1 \\
      --dataset-root /path/to/dataset \\
      --bak-root /path/to/bak \\
      --report /path/to/dedup_report.json \\
      --c1-bulk-dir /path/to/c1_bulk

  %(prog)s gate-check --c1-bulk-dir /path/to/c1_bulk

  %(prog)s submit-phase2 \\
      --dataset-root /path/to/dataset \\
      --bak-root /path/to/bak \\
      --c1-bulk-dir /path/to/c1_bulk

  %(prog)s status --c1-bulk-dir /path/to/c1_bulk

Mock mode (dry-run):
  DLC_BIN=echo %(prog)s submit-phase1 ...
""",
    )
    sub = ap.add_subparsers(dest="command")

    p1 = sub.add_parser("submit-phase1", help="Submit one DLC job per remaining category")
    p1.add_argument("--dataset-root", required=True, help="Dataset root directory")
    p1.add_argument("--bak-root", required=True, help="Backup root directory")
    p1.add_argument("--report", required=True, help="Unified dedup report JSON path")
    p1.add_argument("--c1-bulk-dir", required=True, help="C1 bulk processing directory")
    p1.add_argument("--group-label", default="test0_transitive_apply_seeded",
                    help="Group label for sidecar naming")
    p1.add_argument("--bbox-policy", default="bbox_primary_rmse_observe",
                    help="Bounding-box gating policy")
    p1.add_argument("--out-version", default="v1", help="Output version suffix")
    p1.add_argument("--data-sources", default=DEFAULT_DATA_SOURCES,
                    help="Comma-separated DLC data source IDs")
    p1.add_argument(
        "--mode-reports-dir",
        default=None,
        help="Directory of mode reports (geom_only/, shape_invariant/, topo_filesize/). "
             "Required when --bbox-gated is set for per-pair mode lookup.",
    )
    p1.add_argument("--yes", action="store_true", help="Skip confirmation prompt")

    p2 = sub.add_parser("gate-check", help="Verify all Phase 1 jobs completed")
    p2.add_argument("--c1-bulk-dir", required=True, help="C1 bulk processing directory")
    p2.add_argument("--bbox-policy", default="bbox_primary_rmse_observe",
                    help="Bounding-box gating policy")
    p2.add_argument("--out-version", default="v1", help="Output version suffix")

    p3 = sub.add_parser("submit-phase2", help="Submit Phase 2 merge+scan+delete job")
    p3.add_argument("--dataset-root", required=True, help="Dataset root directory")
    p3.add_argument("--bak-root", required=True, help="Backup root directory")
    p3.add_argument("--c1-bulk-dir", required=True, help="C1 bulk processing directory")
    p3.add_argument("--bbox-policy", default="bbox_primary_rmse_observe",
                    help="Bounding-box gating policy")
    p3.add_argument("--out-version", default="v1", help="Output version suffix")
    p3.add_argument("--group-label", default="test0_transitive_apply_seeded",
                    help="Group label for sidecar naming")
    p3.add_argument("--data-sources", default=DEFAULT_DATA_SOURCES,
                    help="Comma-separated DLC data source IDs")
    p3.add_argument(
        "--mode-reports-dir",
        default=None,
        help="Directory of mode reports for V matrix compensation "
             "(default: auto-discover or skip)",
    )

    p4 = sub.add_parser("status", help="Print completion status table")
    p4.add_argument("--c1-bulk-dir", required=True, help="C1 bulk processing directory")
    p4.add_argument("--bbox-policy", default="bbox_primary_rmse_observe",
                    help="Bounding-box gating policy")
    p4.add_argument("--out-version", default="v1", help="Output version suffix")

    args = ap.parse_args()

    if args.command == "submit-phase1":
        submit_phase1(args)
    elif args.command == "gate-check":
        sys.exit(gate_check(args))
    elif args.command == "submit-phase2":
        sys.exit(submit_phase2(args))
    elif args.command == "status":
        status_cmd(args)
    else:
        ap.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
