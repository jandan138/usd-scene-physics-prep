#!/usr/bin/env python3
"""
Batch-fix MDL absolute imports to relative imports.

Rewrites `import ::KooPbr::X;` and `import ::KooPbr_maps::X;` patterns
into `using .::KooPbr import X;` and `using .::KooPbr_maps import X;`
respectively, making MDL files use relative imports.

Usage:
    python scripts/fix_mdl_absolute_imports.py \
        --mdl-dir /path/to/Material/mdl \
        [--dry-run] \
        [--report check_reports/mdl_import_fix_report.json]
"""

import argparse
import json
import logging
import os
import re
import sys
import time

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger(__name__)

# Files to skip (library MDLs, not user assets)
SKIP_PREFIXES = ("KooPbr.", "KooPbr_maps.", "OmniUe4", "Num")

# Regex for 32-char hex hash filenames (e.g., d41d8cd98f00b204e9800998ecf8427e.mdl)
HEX_HASH_RE = re.compile(r"^[0-9a-f]{24,32}\.mdl$")

# Replacement patterns
REPLACEMENTS = [
    (
        re.compile(r"import\s+::KooPbr::(\w+);"),
        r"using .::KooPbr import \1;",
    ),
    (
        re.compile(r"import\s+::KooPbr_maps::(\w+);"),
        r"using .::KooPbr_maps import \1;",
    ),
]

# Residual check pattern
RESIDUAL_RE = re.compile(r"import\s+::KooPbr")


def _is_target_mdl(filename: str) -> bool:
    """Check if a filename is a target MDL (MI_*.mdl or hex-hash named)."""
    if any(filename.startswith(p) for p in SKIP_PREFIXES):
        return False
    if filename.startswith("MI_") and filename.endswith(".mdl"):
        return True
    if HEX_HASH_RE.match(filename):
        return True
    return False


def _process_file(filepath: str, dry_run: bool) -> dict:
    """Process a single MDL file. Returns per-file result dict."""
    result = {"path": filepath, "replacements": 0, "error": None}
    try:
        with open(filepath, "r", encoding="utf-8", errors="replace") as f:
            content = f.read()

        new_content = content
        total_replacements = 0
        for pattern, replacement in REPLACEMENTS:
            new_content, count = pattern.subn(replacement, new_content)
            total_replacements += count

        result["replacements"] = total_replacements

        if total_replacements > 0 and not dry_run:
            with open(filepath, "w", encoding="utf-8") as f:
                f.write(new_content)

    except Exception as e:
        result["error"] = str(e)

    return result


def main():
    parser = argparse.ArgumentParser(
        description="Batch-fix MDL absolute imports to relative imports"
    )
    parser.add_argument(
        "--mdl-dir",
        required=True,
        help="Path to Material/mdl directory containing MDL files.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Scan and report without modifying files.",
    )
    parser.add_argument(
        "--report",
        default="check_reports/mdl_import_fix_report.json",
        help="Path to output JSON report file (default: check_reports/mdl_import_fix_report.json).",
    )
    args = parser.parse_args()

    mdl_dir = os.path.abspath(args.mdl_dir)
    if not os.path.isdir(mdl_dir):
        log.error("MDL directory not found: %s", mdl_dir)
        sys.exit(1)

    # Find target MDL files
    all_mdl_files = []
    for entry in sorted(os.listdir(mdl_dir)):
        if entry.endswith(".mdl") and _is_target_mdl(entry):
            all_mdl_files.append(os.path.join(mdl_dir, entry))

    log.info("Found %d target MDL files in %s", len(all_mdl_files), mdl_dir)
    if not all_mdl_files:
        log.warning("No target MDL files found. Check --mdl-dir.")
        sys.exit(0)

    if args.dry_run:
        log.info("DRY RUN -- no files will be modified.")

    # Process files
    t0 = time.time()
    results = []
    files_modified = 0
    total_replacements = 0
    errors = []

    for i, filepath in enumerate(all_mdl_files):
        r = _process_file(filepath, args.dry_run)
        results.append(r)

        if r["error"]:
            errors.append(r)
        if r["replacements"] > 0:
            files_modified += 1
            total_replacements += r["replacements"]

        # Progress every 100 files or at the end
        done = i + 1
        if done % 100 == 0 or done == len(all_mdl_files):
            log.info(
                "  Progress: %d/%d files | %d modified | %d replacements",
                done, len(all_mdl_files), files_modified, total_replacements,
            )

    elapsed = time.time() - t0

    # Verification scan for residual absolute imports
    log.info("Running verification scan for residual absolute imports...")
    residual_count = 0
    for filepath in all_mdl_files:
        try:
            with open(filepath, "r", encoding="utf-8", errors="replace") as f:
                content = f.read()
            if RESIDUAL_RE.search(content):
                residual_count += 1
                log.warning("Residual absolute import found in: %s", filepath)
        except Exception:
            pass

    # Summary
    mode_label = "dry-run" if args.dry_run else "apply"
    log.info("=" * 60)
    log.info("[%s] Complete in %.1fs", mode_label.upper(), elapsed)
    log.info("  Total files scanned   : %d", len(all_mdl_files))
    log.info("  Files modified        : %d", files_modified)
    log.info("  Total replacements    : %d", total_replacements)
    log.info("  Errors                : %d", len(errors))
    log.info("  Residual abs imports  : %d", residual_count)

    if errors:
        log.info("Errors:")
        for e in errors:
            log.info("  %s: %s", e["path"], e["error"])

    # Build report
    report = {
        "mode": mode_label,
        "mdl_dir": mdl_dir,
        "total_files_scanned": len(all_mdl_files),
        "files_modified": files_modified,
        "total_replacements": total_replacements,
        "errors": [{"path": e["path"], "error": e["error"]} for e in errors],
        "residual_absolute_imports": residual_count,
        "per_file": [
            {"path": r["path"], "replacements": r["replacements"], "error": r["error"]}
            for r in results
        ],
    }

    # Save report
    report_path = os.path.abspath(args.report)
    os.makedirs(os.path.dirname(report_path), exist_ok=True)
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)
    log.info("Report saved to: %s", report_path)


if __name__ == "__main__":
    main()
