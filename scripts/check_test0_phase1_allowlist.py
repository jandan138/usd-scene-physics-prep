#!/usr/bin/env python3
"""Validate per-category phase1 outputs against the documented allowlist.

This helper is intentionally narrow:

- validate that every discovered per-category phase1 report produced centers
- fail on unexpected nonzero category exits
- fail on unexpected asset errors outside the documented phase1 allowlist

Expected input layout:

    <phase1-root>/
      <category>/stdout.log
      <category>/stderr.log
      <category>/normalize_report.json
      <category>/centers_<category>.json
      phase1_remaining_parallel_summary.json

The summary file is optional but strongly preferred. When a category is missing
from the summary, the helper falls back to the category logs and, for DLC-style
submissions, the per-category `normalize_report.json`.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple


DEFAULT_ALLOWED_NONZERO_EXITS: Dict[str, Set[int]] = {
    "book": {1},
    "cabinet": {1},
    "other": {1},
    "person": {1},
}

DEFAULT_ALLOWED_ASSET_ERRORS: Dict[str, Dict[str, Tuple[str, ...]]] = {
    "book": {
        (
            "book/585006c3abc4841de5e97852f6727331/usd/"
            "585006c3abc4841de5e97852f6727331.usd"
        ): ("No center computed",),
        (
            "book/857ae933547ca0b6091ce1aab9a4774b/usd/"
            "857ae933547ca0b6091ce1aab9a4774b.usd"
        ): ("No center computed",),
        (
            "book/94df8e97aeb11dee37f733fe67ebde38/usd/"
            "94df8e97aeb11dee37f733fe67ebde38.usd"
        ): ("No center computed",),
        (
            "book/9740b28a0b529ab2c999213f5cfd48be/usd/"
            "9740b28a0b529ab2c999213f5cfd48be.usd"
        ): ("No center computed",),
        (
            "book/97ec7af7eacbfad9040a96f576ba5e08/usd/"
            "97ec7af7eacbfad9040a96f576ba5e08.usd"
        ): ("No center computed",),
        (
            "book/9fc72d2c47b84d6d82800b57f4cc469b/usd/"
            "9fc72d2c47b84d6d82800b57f4cc469b.usd"
        ): ("No center computed",),
        (
            "book/aa2302fb9bc7dc1905734a0bc89966fc/usd/"
            "aa2302fb9bc7dc1905734a0bc89966fc.usd"
        ): ("No center computed",),
        (
            "book/ab5ceb64a14186a387b1d975a1b327e8/usd/"
            "ab5ceb64a14186a387b1d975a1b327e8.usd"
        ): ("No center computed",),
        (
            "book/ad6777fe601fabcb710275ff5723f0e2/usd/"
            "ad6777fe601fabcb710275ff5723f0e2.usd"
        ): ("No center computed",),
        (
            "book/ae6901127295a99f2d83d935b4d7b769/usd/"
            "ae6901127295a99f2d83d935b4d7b769.usd"
        ): ("No center computed",),
        (
            "book/aeb041d9eaa496ce0f944d548f18bbb3/usd/"
            "aeb041d9eaa496ce0f944d548f18bbb3.usd"
        ): ("No center computed",),
        (
            "book/af17a29b09ac4f1a8a65f20aa12a3618/usd/"
            "af17a29b09ac4f1a8a65f20aa12a3618.usd"
        ): ("No center computed",),
        (
            "book/af42ec1e2bea0fb40f66337330847aba/usd/"
            "af42ec1e2bea0fb40f66337330847aba.usd"
        ): ("No center computed",),
        (
            "book/bd6f0e9e6544659bcdaa699b7abd339c/usd/"
            "bd6f0e9e6544659bcdaa699b7abd339c.usd"
        ): ("No center computed",),
        (
            "book/c3dcc1f31e1a028a87a6e9881656760b/usd/"
            "c3dcc1f31e1a028a87a6e9881656760b.usd"
        ): ("No center computed",),
        (
            "book/c8050db73a6223c92609b36a5dc4b44c/usd/"
            "c8050db73a6223c92609b36a5dc4b44c.usd"
        ): ("No center computed",),
        (
            "book/caa9b16ef4521367a012086ae3f31850/usd/"
            "caa9b16ef4521367a012086ae3f31850.usd"
        ): ("No center computed",),
        (
            "book/d03afce478e70fe8a8a0046ebdfa7c34/usd/"
            "d03afce478e70fe8a8a0046ebdfa7c34.usd"
        ): ("No center computed",),
        (
            "book/d34424bf14c4cae45d016f3a19d8223c/usd/"
            "d34424bf14c4cae45d016f3a19d8223c.usd"
        ): ("No center computed",),
        (
            "book/dd4744a337da574f6886665fc6054634/usd/"
            "dd4744a337da574f6886665fc6054634.usd"
        ): ("No center computed",),
        (
            "book/ddfe570be55cd05b8d4a163ca9e10fc5/usd/"
            "ddfe570be55cd05b8d4a163ca9e10fc5.usd"
        ): ("No center computed",),
        (
            "book/de903bbd31063f2ca1cde63cb4fdea99/usd/"
            "de903bbd31063f2ca1cde63cb4fdea99.usd"
        ): ("No center computed",),
        (
            "book/e266c1fb005960b3675def911955c41e/usd/"
            "e266c1fb005960b3675def911955c41e.usd"
        ): ("No center computed",),
        (
            "book/e54c05d7a30acdc01c9be935bbbb7d62/usd/"
            "e54c05d7a30acdc01c9be935bbbb7d62.usd"
        ): ("No center computed",),
        (
            "book/e663a5f45dd8c2ac036b5fa4ea8f585a/usd/"
            "e663a5f45dd8c2ac036b5fa4ea8f585a.usd"
        ): ("No center computed",),
        (
            "book/e7e652638eb1f338a809c105e695a5ae/usd/"
            "e7e652638eb1f338a809c105e695a5ae.usd"
        ): ("No center computed",),
        (
            "book/e9f3cd3da73f23db70c8f087fc7de9e5/usd/"
            "e9f3cd3da73f23db70c8f087fc7de9e5.usd"
        ): ("No center computed",),
        (
            "book/ecdbb975eb549ba118e556d112905e01/usd/"
            "ecdbb975eb549ba118e556d112905e01.usd"
        ): ("No center computed",),
        (
            "book/efd4ba9b911d3dda6a41a5a858276778/usd/"
            "efd4ba9b911d3dda6a41a5a858276778.usd"
        ): ("No center computed",),
        (
            "book/f31662ee5ba8b759ac19d1ef00101d2d/usd/"
            "f31662ee5ba8b759ac19d1ef00101d2d.usd"
        ): ("No center computed",),
        (
            "book/f442edec38240800488541663c7e758e/usd/"
            "f442edec38240800488541663c7e758e.usd"
        ): ("No center computed",),
        (
            "book/f4b8e2a86f625a1984617f4a6df785e1/usd/"
            "f4b8e2a86f625a1984617f4a6df785e1.usd"
        ): ("No center computed",),
        (
            "book/f5773edd4429562909f3d9c2bc567f36/usd/"
            "f5773edd4429562909f3d9c2bc567f36.usd"
        ): ("No center computed",),
        (
            "book/f6f9402a89b4327076a46531d2dcfd11/usd/"
            "f6f9402a89b4327076a46531d2dcfd11.usd"
        ): ("No center computed",),
        (
            "book/f8ffb202addfc47695fad0edd3427aba/usd/"
            "f8ffb202addfc47695fad0edd3427aba.usd"
        ): ("No center computed",),
        (
            "book/fb26c80194a84e45a500102838ba15c5/usd/"
            "fb26c80194a84e45a500102838ba15c5.usd"
        ): ("No center computed",),
    },
    "cabinet": {
        (
            "cabinet/b98d6ccbeb75dfdeb60e27649a5b055a/usd/"
            "b98d6ccbeb75dfdeb60e27649a5b055a.usd"
        ): ("No meshes found under /Root/Instance",),
    },
    "other": {
        (
            "other/d41d8cd98f00b204e9800998ecf8427e/usd/"
            "d41d8cd98f00b204e9800998ecf8427e.usd"
        ): ("No meshes found under /Root/Instance",),
    },
    "person": {
        (
            "person/351316cbb083f9f4df0cccd60cbfa848/usd/"
            "351316cbb083f9f4df0cccd60cbfa848.usd"
        ): ("No meshes found under /Root/Instance",),
    },
}

DOCUMENTED_ALLOWLIST_SOURCES: Tuple[str, ...] = (
    "check_reports/normalize/normalize_report.json",
    "check_reports/normalize_v2/phase1_verification.json",
)

SUMMARY_FILE_NAME = "phase1_remaining_parallel_summary.json"
IGNORED_DIR_NAMES = {"centers_merged", "__pycache__", "_submit_logs"}
PHASE1_SUMMARY_RE = re.compile(
    r"Phase 1 complete:\s+(\d+)/(\d+)\s+assets normalized\s+\((\d+)\s+errors\)\s+in\s+([0-9.]+)s"
)
ASSET_ERROR_RE = re.compile(r"^\s*ERROR:\s+([^:]+):\s*(.+)$", re.MULTILINE)


def _normalize_rel_path(value: str) -> str:
    return value.strip().replace("\\", "/")


def _read_text(path: Path) -> str:
    if not path.is_file():
        return ""
    return path.read_text(encoding="utf-8", errors="replace")


def _load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _discover_categories(phase1_root: Path, summary: Dict[str, Any]) -> List[str]:
    categories = set(summary.keys())
    for entry in sorted(phase1_root.iterdir(), key=lambda item: item.name):
        if not entry.is_dir():
            continue
        if entry.name.startswith(".") or entry.name in IGNORED_DIR_NAMES:
            continue
        if entry.name.startswith("centers_"):
            continue
        categories.add(entry.name)
    return sorted(categories)


def _last_phase1_summary(stdout_text: str) -> Optional[Dict[str, Any]]:
    matches = list(PHASE1_SUMMARY_RE.finditer(stdout_text))
    if not matches:
        return None
    match = matches[-1]
    return {
        "normalized_assets": int(match.group(1)),
        "total_assets": int(match.group(2)),
        "error_count": int(match.group(3)),
        "elapsed_sec": float(match.group(4)),
    }


def _infer_returncode(stdout_text: str, stderr_text: str) -> Optional[int]:
    last_summary = _last_phase1_summary(stdout_text)
    if last_summary is not None:
        return 1 if last_summary["error_count"] else 0
    if "Traceback (most recent call last):" in stderr_text:
        return 1
    return None


def _parse_asset_errors(texts: Sequence[str]) -> List[Dict[str, str]]:
    results: List[Dict[str, str]] = []
    seen: Set[Tuple[str, str]] = set()
    for text in texts:
        for match in ASSET_ERROR_RE.finditer(text):
            asset = _normalize_rel_path(match.group(1))
            message = match.group(2).strip()
            key = (asset, message)
            if key in seen:
                continue
            seen.add(key)
            results.append({"asset": asset, "message": message})
    return results


def _parse_asset_errors_from_report(report: Dict[str, Any]) -> List[Dict[str, str]]:
    results: List[Dict[str, str]] = []
    seen: Set[Tuple[str, str]] = set()
    for item in report.get("errors", []) or []:
        if not isinstance(item, dict):
            continue
        asset = _normalize_rel_path(str(item.get("asset") or ""))
        message = str(item.get("error") or item.get("message") or "").strip()
        if not asset or not message:
            continue
        key = (asset, message)
        if key in seen:
            continue
        seen.add(key)
        results.append({"asset": asset, "message": message})
    return results


def _allowed_exit_codes_for(category: str) -> Set[int]:
    allowed = {0}
    allowed.update(DEFAULT_ALLOWED_NONZERO_EXITS.get(category, set()))
    if category.startswith("door_"):
        allowed.add(1)
    return allowed


def _allowed_asset_errors_for(category: str) -> Dict[str, Tuple[str, ...]]:
    static = DEFAULT_ALLOWED_ASSET_ERRORS.get(category)
    if static is not None:
        return static
    if category.startswith("door_"):
        asset_key = (
            f"{category}/d41d8cd98f00b204e9800998ecf8427e/usd/"
            "d41d8cd98f00b204e9800998ecf8427e.usd"
        )
        return {asset_key: ("No meshes found under /Root/Instance",)}
    return {}


def _evaluate_category(
    *,
    phase1_root: Path,
    category: str,
    summary_entry: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    category_dir = phase1_root / category
    stdout_path = category_dir / "stdout.log"
    stderr_path = category_dir / "stderr.log"
    normalize_report_path = category_dir / "normalize_report.json"
    stdout_text = _read_text(stdout_path)
    stderr_text = _read_text(stderr_path)
    normalize_report = (
        _load_json(normalize_report_path)
        if normalize_report_path.is_file()
        else None
    )

    inferred_summary = _last_phase1_summary(stdout_text)
    returncode = None
    if isinstance(summary_entry, dict):
        raw_returncode = summary_entry.get("returncode")
        if isinstance(raw_returncode, int):
            returncode = raw_returncode
    if returncode is None:
        returncode = _infer_returncode(stdout_text, stderr_text)
    if returncode is None and isinstance(normalize_report, dict):
        raw_errors_count = normalize_report.get("meta", {}).get("errors_count")
        if isinstance(raw_errors_count, int):
            returncode = 1 if raw_errors_count else 0

    centers_files: List[str] = []
    if isinstance(summary_entry, dict):
        raw_centers_files = summary_entry.get("centers_files")
        if isinstance(raw_centers_files, list):
            centers_files = [str(item) for item in raw_centers_files]
    if not centers_files and category_dir.is_dir():
        centers_files = sorted(path.name for path in category_dir.glob("centers_*.json"))

    missing_centers_files: List[str] = []
    for name in centers_files:
        if not (category_dir / name).is_file():
            missing_centers_files.append(name)
    if not centers_files:
        missing_centers_files.append(f"centers_<{category}>.json")

    asset_errors = _parse_asset_errors((stderr_text, stdout_text))
    if not asset_errors and isinstance(normalize_report, dict):
        asset_errors = _parse_asset_errors_from_report(normalize_report)
    if inferred_summary is None and centers_files and not asset_errors:
        returncode = 0

    final_error_count = None
    if inferred_summary is not None:
        final_error_count = int(inferred_summary["error_count"])
    elif isinstance(normalize_report, dict):
        raw_errors_count = normalize_report.get("meta", {}).get("errors_count")
        if isinstance(raw_errors_count, int):
            final_error_count = raw_errors_count
    elif returncode is not None and returncode != 0:
        final_error_count = 1
    elif returncode == 0:
        final_error_count = 0

    if final_error_count == 0:
        asset_errors = []
    allowed_asset_errors = _allowed_asset_errors_for(category)
    allowlisted_hits: List[Dict[str, str]] = []
    unexpected_asset_errors: List[Dict[str, str]] = []
    for item in asset_errors:
        allowed_substrings = allowed_asset_errors.get(item["asset"])
        if allowed_substrings and any(substring in item["message"] for substring in allowed_substrings):
            allowlisted_hits.append(item)
        else:
            unexpected_asset_errors.append(item)

    failures: List[str] = []
    if missing_centers_files:
        failures.append(
            "missing centers files: " + ", ".join(sorted(missing_centers_files))
        )

    allowed_exit_codes = _allowed_exit_codes_for(category)
    if returncode is None:
        failures.append("could not determine category returncode from summary or logs")
    elif returncode not in allowed_exit_codes:
        failures.append(
            f"unexpected returncode {returncode}; allowed={sorted(allowed_exit_codes)}"
        )

    if unexpected_asset_errors:
        failures.append(
            "unexpected asset errors: "
            + "; ".join(
                f"{item['asset']} -> {item['message']}" for item in unexpected_asset_errors
            )
        )

    if final_error_count is not None:
        if final_error_count != len(allowlisted_hits) + len(unexpected_asset_errors):
            failures.append(
                "phase1 error count mismatch: "
                f"summary/logs report {final_error_count}, "
                f"parsed {len(allowlisted_hits) + len(unexpected_asset_errors)} asset errors"
            )

    return {
        "category": category,
        "category_dir": str(category_dir),
        "summary_present": isinstance(summary_entry, dict),
        "returncode": returncode,
        "allowed_exit_codes": sorted(allowed_exit_codes),
        "centers_files": centers_files,
        "missing_centers_files": missing_centers_files,
        "final_phase1_error_count": final_error_count,
        "allowlisted_asset_errors": allowlisted_hits,
        "unexpected_asset_errors": unexpected_asset_errors,
        "failures": failures,
    }


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--phase1-root",
        required=True,
        help="Per-category phase1 report root",
    )
    parser.add_argument(
        "--summary-json",
        default=None,
        help=(
            "Optional path to the per-category phase1 summary JSON. "
            f"Defaults to <phase1-root>/{SUMMARY_FILE_NAME} when present."
        ),
    )
    parser.add_argument(
        "--json-out",
        default=None,
        help="Optional path to write the full verdict JSON",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    phase1_root = Path(args.phase1_root).expanduser().resolve()
    if not phase1_root.is_dir():
        print(f"ERROR: phase1 root is not a directory: {phase1_root}", file=sys.stderr)
        return 2

    summary_path = (
        Path(args.summary_json).expanduser().resolve()
        if args.summary_json
        else phase1_root / SUMMARY_FILE_NAME
    )
    summary: Dict[str, Any] = {}
    summary_loaded = False
    if summary_path.is_file():
        loaded = _load_json(summary_path)
        if not isinstance(loaded, dict):
            print(f"ERROR: summary JSON is not an object: {summary_path}", file=sys.stderr)
            return 2
        summary = loaded
        summary_loaded = True

    categories = _discover_categories(phase1_root, summary)
    if not categories:
        print(f"ERROR: no category outputs found under {phase1_root}", file=sys.stderr)
        return 2

    category_results = [
        _evaluate_category(
            phase1_root=phase1_root,
            category=category,
            summary_entry=summary.get(category),
        )
        for category in categories
    ]
    failures = [
        {"category": result["category"], "messages": result["failures"]}
        for result in category_results
        if result["failures"]
    ]

    verdict = {
        "status": "pass" if not failures else "fail",
        "phase1_root": str(phase1_root),
        "summary_json": str(summary_path) if summary_loaded else None,
        "summary_loaded": summary_loaded,
        "checked_categories": len(category_results),
        "failed_categories": len(failures),
        "documented_allowlist_sources": list(DOCUMENTED_ALLOWLIST_SOURCES),
        "default_allowlist": {
            "allowed_nonzero_exit_codes": {
                category: sorted(codes)
                for category, codes in sorted(DEFAULT_ALLOWED_NONZERO_EXITS.items())
            },
            "allowed_asset_errors": {
                category: {
                    asset: list(substrings)
                    for asset, substrings in sorted(entries.items())
                }
                for category, entries in sorted(DEFAULT_ALLOWED_ASSET_ERRORS.items())
            },
        },
        "failures": failures,
        "categories": category_results,
    }

    if args.json_out:
        out_path = Path(args.json_out).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with out_path.open("w", encoding="utf-8") as handle:
            json.dump(verdict, handle, indent=2, ensure_ascii=False)
            handle.write("\n")

    if verdict["status"] == "pass":
        print(
            f"PASS: validated {len(category_results)} categories under {phase1_root}"
        )
        if summary_loaded:
            print(f"Summary: {summary_path}")
        return 0

    print(
        f"FAIL: {len(failures)} categories failed the phase1 allowlist gate",
        file=sys.stderr,
    )
    for failure in failures:
        for message in failure["messages"]:
            print(f"  - {failure['category']}: {message}", file=sys.stderr)
    if args.json_out:
        print(f"Verdict JSON: {Path(args.json_out).expanduser().resolve()}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
