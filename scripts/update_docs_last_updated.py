#!/usr/bin/env python3
"""Update docs "last updated" date based on git history.

This repo's docs use a lightweight header convention:

  > 最后更新：YYYY-MM-DD
  >
  > 相关代码：
  > - ../../path/to/file.py
  > - ...

This script reads those related-code paths, maps them to repo-relative paths,
then sets "最后更新" to the latest git commit date among:
- the doc file itself
- all related code files listed in the doc

Usage:
  python3 scripts/update_docs_last_updated.py --dry-run
  python3 scripts/update_docs_last_updated.py --write

Notes:
- Requires running inside a git repository.
- If a related path doesn't exist, it is ignored (but reported in dry-run).
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import datetime
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple


DOCS_GLOB_ROOT = "docs"


@dataclass
class UpdateResult:
    doc_rel: str
    old_date: Optional[str]
    new_date: Optional[str]
    missing_related: List[str]


HEADER_LAST_UPDATED_RE = re.compile(r"^>\s*最后更新：\s*(\d{4}-\d{2}-\d{2})\s*$")
HEADER_RELATED_START_RE = re.compile(r"^>\s*相关代码：\s*$")
HEADER_RELATED_ITEM_RE = re.compile(r"^>\s*-\s+(.+?)\s*$")


def _run(cmd: List[str], cwd: str) -> str:
    return subprocess.check_output(cmd, cwd=cwd, stderr=subprocess.STDOUT, text=True).strip()


def _git_root(cwd: str) -> str:
    return _run(["git", "rev-parse", "--show-toplevel"], cwd=cwd)


def _git_last_date(cwd: str, rel_path: str) -> Optional[str]:
    try:
        out = _run(["git", "log", "-1", "--format=%cs", "--", rel_path], cwd=cwd)
        return out or None
    except subprocess.CalledProcessError:
        return None


def _git_is_dirty(cwd: str, rel_path: str) -> bool:
    """Return True if rel_path has staged or unstaged changes."""

    try:
        out = _run(["git", "status", "--porcelain", "--", rel_path], cwd=cwd)
        return bool(out.strip())
    except subprocess.CalledProcessError:
        return False


def _walk_markdown_files(repo_root: str) -> List[str]:
    docs_root = os.path.join(repo_root, DOCS_GLOB_ROOT)
    results: List[str] = []
    for root, _, files in os.walk(docs_root):
        for f in files:
            if f.lower().endswith(".md"):
                abs_path = os.path.join(root, f)
                results.append(os.path.relpath(abs_path, repo_root))
    results.sort()
    return results


def _extract_related_paths(doc_text: str) -> Tuple[Optional[str], List[str]]:
    old_date: Optional[str] = None
    related: List[str] = []

    lines = doc_text.splitlines()

    in_related = False
    for line in lines[:120]:  # header is expected near top
        m_date = HEADER_LAST_UPDATED_RE.match(line)
        if m_date:
            old_date = m_date.group(1)
            continue

        if HEADER_RELATED_START_RE.match(line):
            in_related = True
            continue

        if in_related:
            m_item = HEADER_RELATED_ITEM_RE.match(line)
            if m_item:
                related.append(m_item.group(1))
                continue
            # stop scanning related items when header block ends
            if line.strip() == ">":
                continue
            if not line.startswith(">"):
                break

    return old_date, related


def _strip_related_item(raw: str) -> str:
    """Extract a filesystem-ish path from a related-code bullet.

    Allows entries like:
      ../../set_physics/pxr_utils/data_clean.py（拆分/规范化：`parse_scene`）
      ../../clean_data.py (entrypoint)
    """

    s = raw.strip().strip("`")
    # Drop inline notes that start with Chinese/English parentheses.
    for sep in ("（", "("):
        if sep in s:
            s = s.split(sep, 1)[0].rstrip()
    # Also drop trailing whitespace + comment.
    if " " in s:
        s = s.split(" ", 1)[0]
    return s


def _resolve_related_to_repo(rel_doc_path: str, related_item: str) -> Optional[str]:
    # related_item is typically like ../../set_physics/simready.py
    # Resolve from doc directory to a repo-relative path.
    doc_dir = os.path.dirname(rel_doc_path)
    candidate = os.path.normpath(os.path.join(doc_dir, related_item))
    # Ensure it's repo-relative (no absolute paths)
    if os.path.isabs(candidate):
        return None
    return candidate


def _update_last_updated_line(doc_text: str, new_date: str) -> str:
    lines = doc_text.splitlines()
    out_lines: List[str] = []
    replaced = False
    for line in lines:
        if not replaced and HEADER_LAST_UPDATED_RE.match(line):
            out_lines.append(f"> 最后更新：{new_date}")
            replaced = True
        else:
            out_lines.append(line)
    # If missing, do not auto-insert (keep minimal and predictable)
    return "\n".join(out_lines) + ("\n" if doc_text.endswith("\n") else "")


def _max_date(dates: Iterable[Optional[str]]) -> Optional[str]:
    present = [d for d in dates if d]
    return max(present) if present else None


def process_doc(repo_root: str, doc_rel: str) -> UpdateResult:
    abs_doc = os.path.join(repo_root, doc_rel)
    with open(abs_doc, "r", encoding="utf-8") as f:
        text = f.read()

    old_date, related_items = _extract_related_paths(text)

    missing: List[str] = []
    related_repo_paths: List[str] = []
    for item in related_items:
        cleaned = _strip_related_item(item)
        if not cleaned:
            continue
        repo_rel = _resolve_related_to_repo(doc_rel, cleaned)
        if not repo_rel:
            missing.append(item)
            continue
        abs_related = os.path.join(repo_root, repo_rel)
        if not os.path.exists(abs_related):
            missing.append(repo_rel)
            continue
        related_repo_paths.append(repo_rel)

    today = datetime.date.today().isoformat()
    dates = [today if _git_is_dirty(repo_root, doc_rel) else _git_last_date(repo_root, doc_rel)]
    for p in related_repo_paths:
        dates.append(today if _git_is_dirty(repo_root, p) else _git_last_date(repo_root, p))

    new_date = _max_date(dates)

    return UpdateResult(
        doc_rel=doc_rel,
        old_date=old_date,
        new_date=new_date,
        missing_related=missing,
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true", help="Print intended updates, do not write files")
    mode.add_argument("--write", action="store_true", help="Write updates to files")
    ap.add_argument(
        "--docs-root",
        default=DOCS_GLOB_ROOT,
        help="Docs root directory (default: docs)",
    )

    args = ap.parse_args()

    repo_root = _git_root(os.getcwd())

    docs_root = os.path.join(repo_root, args.docs_root)
    if not os.path.isdir(docs_root):
        raise SystemExit(f"Docs root not found: {docs_root}")

    doc_files = _walk_markdown_files(repo_root)

    changed = 0
    missing_total = 0

    for doc_rel in doc_files:
        if not doc_rel.startswith(args.docs_root + os.sep):
            continue

        res = process_doc(repo_root, doc_rel)
        if res.missing_related:
            missing_total += len(res.missing_related)

        if not res.new_date or not res.old_date:
            # If header is missing, keep as-is (we only update existing convention).
            continue

        if res.old_date == res.new_date:
            continue

        changed += 1
        if args.dry_run:
            print(f"UPDATE {res.doc_rel}: {res.old_date} -> {res.new_date}")
            if res.missing_related:
                for m in res.missing_related:
                    print(f"  missing-related: {m}")
        else:
            abs_doc = os.path.join(repo_root, res.doc_rel)
            with open(abs_doc, "r", encoding="utf-8") as f:
                text = f.read()
            new_text = _update_last_updated_line(text, res.new_date)
            with open(abs_doc, "w", encoding="utf-8") as f:
                f.write(new_text)

    if args.dry_run:
        print(f"\nDocs scanned: {len(doc_files)}")
        print(f"Docs to update: {changed}")
        if missing_total:
            print(f"Missing related-code entries (ignored): {missing_total}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
