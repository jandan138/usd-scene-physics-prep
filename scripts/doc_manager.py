#!/usr/bin/env python3
"""
Documentation Manager Script

Features:
1. Validate Docs: Check for required metadata headers (YAML front matter).
2. Generate Index: Auto-generate docs/INDEX.md based on metadata.
"""

import os
import glob
import yaml
import datetime
from typing import Dict, List, Optional

DOCS_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "docs")
INDEX_FILE = os.path.join(DOCS_DIR, "INDEX.md")

REQUIRED_FIELDS = ["title", "created_at", "updated_at", "maintainer"]
SKIP_BASENAMES = {"INDEX.md", "README.md", "index.md"}
VALID_DOC_CLASSES = {"primary", "record", "archive"}
ROOT_GROUP_TITLE = "General"
ARCHIVE_GROUP_TITLE = "Archive"

PRIMARY_GROUP_TITLES = {
    "architecture": "Architecture",
    "examples": "Examples",
    "faq": "FAQ",
    "operations": "Operations",
    "overview": "Overview",
    "references": "References",
    "scripts": "Scripts",
    "specs_normalizer": "Specs Normalizer",
    "usage": "Getting Started",
}

RECORD_GROUP_TITLES = {
    "changes": "Changes",
    "dlc": "Research",
    "runs": "Runs",
    "operations": "Operations",
    "research": "Research",
    "test0_full": "Research",
    "test0_smoke": "Research",
}

LEGACY_RECORD_ROOTS = {"changes", "dlc", "research", "test0_full", "test0_smoke"}


def normalize_code_references(code_ref) -> List[str]:
    """Normalize code_reference metadata into a flat list of string paths."""
    if not code_ref:
        return []
    if isinstance(code_ref, (list, tuple)):
        refs = []
        for item in code_ref:
            refs.extend(normalize_code_references(item))
        return refs

    raw_text = str(code_ref)
    refs = []
    for line in raw_text.splitlines():
        for part in line.split(","):
            ref = part.strip()
            if not ref:
                continue

            normalized = ref.upper()
            if (
                normalized == "N/A"
                or normalized.startswith("N/A ")
                or normalized.startswith("N/A-")
            ):
                continue

            refs.append(ref)
    return refs


def parse_front_matter(file_path: str) -> Optional[Dict]:
    """Extract and parse YAML front matter from a markdown file."""
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    if not content.startswith("---\n"):
        return None

    try:
        parts = content.split("---\n", 2)
        if len(parts) < 3:
            return None
        return yaml.safe_load(parts[1])
    except yaml.YAMLError:
        return None


def infer_doc_class(rel_path: str) -> str:
    """Infer doc_class from the document path when metadata omits it."""
    parts = rel_path.split(os.sep)
    root = parts[0]
    if root == "records":
        if len(parts) > 1 and parts[1] == "archive":
            return "archive"
        return "record"
    if root in LEGACY_RECORD_ROOTS:
        return "record"
    return "primary"


def get_doc_class(meta: Optional[Dict], rel_path: str) -> str:
    if meta and meta.get("doc_class"):
        return str(meta["doc_class"])
    return infer_doc_class(rel_path)


def get_section_name(doc_class: str) -> str:
    if doc_class == "primary":
        return "Primary Docs"
    if doc_class in {"record", "archive"}:
        return "Records"
    raise ValueError(f"Unsupported doc_class for index generation: {doc_class}")


def get_group_title(rel_path: str, doc_class: str) -> str:
    if doc_class == "archive":
        return ARCHIVE_GROUP_TITLE

    if os.sep not in rel_path:
        return ROOT_GROUP_TITLE

    parts = rel_path.split(os.sep)
    root = parts[0]
    if root == "records":
        group_root = parts[1] if len(parts) > 1 else root
        return RECORD_GROUP_TITLES.get(
            group_root, group_root.replace("_", " ").replace("-", " ").title()
        )

    if doc_class == "primary":
        return PRIMARY_GROUP_TITLES.get(
            root, root.replace("_", " ").replace("-", " ").title()
        )
    return RECORD_GROUP_TITLES.get(
        root, root.replace("_", " ").replace("-", " ").title()
    )


def format_code_reference_link(path: str) -> str:
    clean_path = path.strip()
    label = (
        os.path.basename(clean_path.rstrip("/\\"))
        or clean_path.rstrip("/\\")
        or clean_path
    )
    return f"[`{label}`]({clean_path})"


def validate_docs():
    """Validate all markdown files in docs/ directory."""
    print("Validating documentation...")
    md_files = glob.glob(os.path.join(DOCS_DIR, "**/*.md"), recursive=True)

    issues = []

    for file_path in md_files:
        if os.path.basename(file_path) in SKIP_BASENAMES:
            continue

        rel_path = os.path.relpath(file_path, DOCS_DIR)
        meta = parse_front_matter(file_path)

        if not meta:
            issues.append(f"[MISSING HEADER] {rel_path}")
            continue

        missing_fields = [f for f in REQUIRED_FIELDS if f not in meta]
        if missing_fields:
            issues.append(f"[INCOMPLETE HEADER] {rel_path}: Missing {missing_fields}")

        doc_class = meta.get("doc_class")
        if doc_class is not None and str(doc_class) not in VALID_DOC_CLASSES:
            issues.append(f"[INVALID DOC CLASS] {rel_path}: {doc_class}")

    if issues:
        print(f"Found {len(issues)} issues:")
        for issue in issues:
            print(issue)
    else:
        print("All documents look good!")


def find_references(target_file: str):
    """Find all docs that reference the given code file."""
    print(f"Searching for references to: {target_file}")
    md_files = glob.glob(os.path.join(DOCS_DIR, "**/*.md"), recursive=True)
    found = []

    # Normalize target path for comparison (basic normalization)
    target_base = os.path.basename(target_file)

    for file_path in md_files:
        meta = parse_front_matter(file_path)
        if not meta:
            continue

        code_refs = normalize_code_references(meta.get("code_reference", ""))
        if not code_refs:
            continue

        if any(
            target_file in code_ref
            or target_base == os.path.basename(code_ref.rstrip("/\\"))
            for code_ref in code_refs
        ):
            rel_path = os.path.relpath(file_path, DOCS_DIR)
            found.append((rel_path, meta.get("title", "Untitled")))

    if found:
        print(f"Found {len(found)} documents referencing '{target_file}':")
        for path, title in found:
            print(f"- {path} ({title})")
    else:
        print(f"No documents found referencing '{target_file}'.")


def generate_index():
    """Generate docs/INDEX.md based on metadata."""
    print("Generating INDEX.md...")
    md_files = glob.glob(os.path.join(DOCS_DIR, "**/*.md"), recursive=True)

    sections = {
        "Primary Docs": {},
        "Records": {},
    }

    for file_path in md_files:
        filename = os.path.basename(file_path)
        if filename in SKIP_BASENAMES:
            continue

        rel_path = os.path.relpath(file_path, DOCS_DIR)
        meta = parse_front_matter(file_path)
        title = meta.get("title", filename) if meta else filename
        status = meta.get("status", "Active") if meta else "Unknown"
        updated = meta.get("updated_at", "N/A") if meta else "N/A"
        code_ref = meta.get("code_reference", "") if meta else ""
        doc_class = get_doc_class(meta, rel_path)
        section_name = get_section_name(doc_class)
        group_title = get_group_title(rel_path, doc_class)

        entry = {
            "title": title,
            "path": rel_path.replace("\\", "/"),
            "status": status,
            "updated": updated,
            "code_ref": normalize_code_references(code_ref),
        }
        sections[section_name].setdefault(group_title, []).append(entry)

    with open(INDEX_FILE, "w", encoding="utf-8") as f:
        f.write("# Project Documentation Index\n\n")
        f.write(f"> Generated at: {datetime.date.today()}\n\n")

        for section_name, groups in sections.items():
            if not groups:
                continue

            f.write(f"## {section_name}\n\n")
            for group_title in sorted(groups):
                entries = groups[group_title]
                f.write(f"### {group_title}\n\n")
                f.write("| Document | Status | Last Updated | Related Code |\n")
                f.write("| :--- | :--- | :--- | :--- |\n")

                entries.sort(key=lambda x: x["title"])

                for entry in entries:
                    if entry["code_ref"]:
                        code_link = ", ".join(
                            format_code_reference_link(path)
                            for path in entry["code_ref"]
                        )
                    else:
                        code_link = "-"
                    f.write(
                        f"| [{entry['title']}]({entry['path']}) | {entry['status']} | {entry['updated']} | {code_link} |\n"
                    )
                f.write("\n")

    print(f"Index generated at {INDEX_FILE}")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--validate", action="store_true", help="Validate docs metadata"
    )
    parser.add_argument("--gen-index", action="store_true", help="Generate INDEX.md")
    parser.add_argument(
        "--find-refs", help="Find docs referencing a specific code file"
    )
    args = parser.parse_args()

    if args.validate:
        validate_docs()

    if args.gen_index:
        generate_index()

    if args.find_refs:
        find_references(args.find_refs)

    if not any([args.validate, args.gen_index, args.find_refs]):
        # Default behavior: run validation and index generation
        validate_docs()
        generate_index()
