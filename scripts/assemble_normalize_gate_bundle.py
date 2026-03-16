#!/usr/bin/env python3
"""Assemble normalize-only gate outputs for a run summary directory.

This helper delegates to the existing gate scripts so their logic stays in one
place. It writes these files into the target summary directory:

- normalize_gate_verdict.json
- final_verdict.json
- summary.md
"""

from __future__ import annotations

import argparse
import json
import re
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set


def _glob_unique(base: Path, patterns: Iterable[str]) -> List[Path]:
    matches: List[Path] = []
    seen = set()
    for pattern in patterns:
        for match in sorted(base.glob(pattern)):
            resolved = match.resolve()
            if resolved in seen or not match.is_file():
                continue
            seen.add(resolved)
            matches.append(resolved)
    return matches


def _infer_report_path(
    run_root: Path,
    explicit_path: Optional[str],
    preferred_patterns: Sequence[str],
    fallback_patterns: Sequence[str],
    label: str,
) -> Path:
    if explicit_path:
        path = Path(explicit_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"{label} report not found: {path}")
        return path

    preferred = _glob_unique(run_root, preferred_patterns)
    if len(preferred) == 1:
        return preferred[0]
    if len(preferred) > 1:
        joined = "\n".join(f"  - {path}" for path in preferred)
        raise RuntimeError(
            f"Multiple {label} reports matched preferred patterns under {run_root}:\n{joined}\n"
            f"Pass --{label}-report explicitly."
        )

    fallback = _glob_unique(run_root, fallback_patterns)
    if len(fallback) == 1:
        return fallback[0]
    if len(fallback) > 1:
        joined = "\n".join(f"  - {path}" for path in fallback)
        raise RuntimeError(
            f"Multiple {label} reports matched fallback patterns under {run_root}:\n{joined}\n"
            f"Pass --{label}-report explicitly."
        )

    raise FileNotFoundError(
        f"Could not infer {label} report under {run_root}. Pass --{label}-report explicitly."
    )


def _run_command(argv: Sequence[str], allow_exit_codes: Sequence[int]) -> subprocess.CompletedProcess[str]:
    completed = subprocess.run(argv, check=False, capture_output=True, text=True)
    if completed.returncode not in allow_exit_codes:
        cmd = shlex.join(argv)
        stderr = completed.stderr.strip()
        stdout = completed.stdout.strip()
        detail_parts = [f"command failed: {cmd}", f"exit code: {completed.returncode}"]
        if stdout:
            detail_parts.append(f"stdout:\n{stdout}")
        if stderr:
            detail_parts.append(f"stderr:\n{stderr}")
        raise RuntimeError("\n".join(detail_parts))
    return completed


def _load_json(path: Path) -> Dict[str, object]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


_EMPTY_MD5 = "d41d8cd98f00b204e9800998ecf8427e"
_INSTANCE_MULTIPLIER = 10


def _count_max_missing_from_allowlist(allowlist_path: str) -> int:
    """Count known-bad asset rel_keys from an allowlist_verdict.json.

    Returns ``len(known_bad_assets) * _INSTANCE_MULTIPLIER`` as a generous
    upper-bound for the number of scene-prim instances that may lack centers.
    """
    with open(allowlist_path, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    default_al = data.get("default_allowlist", {})
    bad_keys: Set[str] = set()

    # Explicit asset errors (book, cabinet, other, person, ...)
    for _cat, assets in (default_al.get("allowed_asset_errors") or {}).items():
        bad_keys.update(assets.keys())

    # door_* categories: each allowed exit-code-1 door category has the empty-MD5 asset
    for cat, _codes in (default_al.get("allowed_nonzero_exit_codes") or {}).items():
        if re.match(r"^door_", cat):
            bad_keys.add(
                f"{cat}/{_EMPTY_MD5}/usd/{_EMPTY_MD5}.usd"
            )

    if not bad_keys:
        return 0
    return len(bad_keys) * _INSTANCE_MULTIPLIER


def _default_run_id(summary_dir: Path) -> str:
    if summary_dir.name == "summary":
        return summary_dir.parent.name
    return summary_dir.name


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary-dir", required=True, help="Directory to receive the gate bundle")
    parser.add_argument("--run-id", default=None, help="Run id for final_verdict.json; defaults from the summary dir")
    parser.add_argument("--pairwise-report", default=None, help="Explicit pairwise report JSON path")
    parser.add_argument("--audit-report", default=None, help="Explicit audit report JSON path")
    parser.add_argument("--python", dest="python_exe", default=sys.executable, help="Python executable for delegated scripts")
    parser.add_argument(
        "--allowlist-json",
        default=None,
        help="Phase 1 allowlist_verdict.json; computes --max-missing-centers for gate scripts",
    )
    args = parser.parse_args(argv)

    summary_dir = Path(args.summary_dir).expanduser().resolve()
    run_root = summary_dir.parent if summary_dir.name == "summary" else summary_dir
    run_id = args.run_id or _default_run_id(summary_dir)

    pairwise_path = _infer_report_path(
        run_root=run_root,
        explicit_path=args.pairwise_report,
        preferred_patterns=(
            "normalize/pairwise_compare.json",
            "normalize/*pairwise*.json",
            "normalize/*vs_normalized_pre_dedup.json",
        ),
        fallback_patterns=(
            "**/pairwise_compare.json",
            "**/*pairwise*.json",
            "**/*vs_normalized_pre_dedup.json",
        ),
        label="pairwise",
    )
    audit_path = _infer_report_path(
        run_root=run_root,
        explicit_path=args.audit_report,
        preferred_patterns=(
            "normalize/audit_phase2.json",
            "normalize/*phase2*audit*.json",
        ),
        fallback_patterns=(
            "**/audit_phase2.json",
            "**/*phase2*audit*.json",
        ),
        label="audit",
    )

    max_missing_centers = 0
    if args.allowlist_json:
        max_missing_centers = _count_max_missing_from_allowlist(args.allowlist_json)

    summary_dir.mkdir(parents=True, exist_ok=True)
    script_dir = Path(__file__).resolve().parent
    normalize_gate_out = summary_dir / "normalize_gate_verdict.json"
    final_verdict_out = summary_dir / "final_verdict.json"
    summary_md_out = summary_dir / "summary.md"

    check_cmd = [
        args.python_exe,
        str(script_dir / "check_normalize_gate_from_reports.py"),
        "--pairwise-report",
        str(pairwise_path),
        "--audit-report",
        str(audit_path),
        "--out",
        str(normalize_gate_out),
        "--max-missing-centers",
        str(max_missing_centers),
    ]
    check_result = _run_command(check_cmd, allow_exit_codes=(0, 1))

    build_cmd = [
        args.python_exe,
        str(script_dir / "build_normalize_gate_verdict.py"),
        "--run-id",
        run_id,
        "--pairwise-json",
        str(pairwise_path),
        "--audit-json",
        str(audit_path),
        "--out-json",
        str(final_verdict_out),
        "--out-md",
        str(summary_md_out),
        "--max-missing-centers",
        str(max_missing_centers),
    ]
    _run_command(build_cmd, allow_exit_codes=(0,))

    normalize_gate = _load_json(normalize_gate_out)
    final_verdict = _load_json(final_verdict_out)
    manifest = {
        "summary_dir": str(summary_dir),
        "run_id": run_id,
        "inputs": {
            "pairwise_report": str(pairwise_path),
            "audit_report": str(audit_path),
        },
        "outputs": {
            "normalize_gate_verdict": str(normalize_gate_out),
            "final_verdict": str(final_verdict_out),
            "summary_md": str(summary_md_out),
        },
        "status": {
            "normalize_gate_exit_code": check_result.returncode,
            "normalize_gate_status": normalize_gate.get("status"),
            "final_verdict_status": final_verdict.get("status"),
        },
        "commands": {
            "check_normalize_gate_from_reports": shlex.join(check_cmd),
            "build_normalize_gate_verdict": shlex.join(build_cmd),
        },
    }
    json.dump(manifest, sys.stdout, indent=2, ensure_ascii=False)
    sys.stdout.write("\n")
    return check_result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
