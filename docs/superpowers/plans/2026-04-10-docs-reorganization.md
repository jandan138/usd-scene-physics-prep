---
title: Docs Reorganization Implementation Plan
code_reference:
  - docs/
  - scripts/doc_manager.py
  - tests/test_doc_manager.py
created_at: 2026-04-10
updated_at: 2026-04-10
maintainer: OpenCode
status: completed
doc_class: archive
---

# Docs Reorganization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reorganize `docs/` into curated primary documentation plus historical records, upgrade `scripts/doc_manager.py` to generate a two-layer index, and leave the repository with passing doc validation and cleaner navigation.

**Architecture:** Implement the reorganization in six phases. First harden `scripts/doc_manager.py` with tests that prove `INDEX.md` separates primary docs from records and rejects invalid `doc_class` values. Then create the new directory skeleton and curated landing pages, move stable guides into `getting-started/`, `how-to/`, `reference/`, and `operations/`, rehome historical material under `records/`, repair the mixed `docs/dlc/` and `docs/operations/` areas, and finish with metadata/link cleanup plus final validation. Use path-based `doc_class` inference for the first migration pass so the tool works immediately, and add explicit `doc_class` only where a header is already being repaired or a file remains ambiguous.

**Tech Stack:** Markdown, Python 3, PyYAML, pytest, bash (`mkdir`, `mv`, `rmdir`)

---

## File Map

- Create: `tests/test_doc_manager.py` — regression tests for grouped index generation, invalid `doc_class` handling, and skipped `index.md` / `README.md` files.
- Modify: `scripts/doc_manager.py` — add `doc_class` validation, path-based classification helpers, and `Primary Docs` / `Records` index output.
- Create: `docs/getting-started/index.md`, `docs/how-to/index.md`, `docs/reference/index.md`, `docs/records/index.md`, `docs/operations/index.md` — human landing pages that stay out of `docs/INDEX.md` because `index.md` is skipped.
- Modify: `docs/index.md` — curated top-level documentation home.
- Move: `docs/overview/project_overview.md -> docs/getting-started/project_overview.md`
- Move: `docs/usage/quickstart.md -> docs/getting-started/quickstart.md`
- Move: `docs/faq/faq.md -> docs/getting-started/faq.md`
- Move: `docs/examples/workflow_examples.md -> docs/how-to/workflow_examples.md`
- Move: remaining `docs/usage/*.md` files into `docs/how-to/`
- Move: `docs/modules -> docs/reference/modules`
- Move: `docs/scripts -> docs/reference/scripts`
- Move: `docs/specs -> docs/reference/specs`
- Move: `docs/specs_normalizer -> docs/reference/specs-normalizer`
- Move: `docs/references/assets_and_materials.md -> docs/reference/assets_and_materials.md`
- Move: `docs/references/dependencies.md -> docs/reference/dependencies.md`
- Move: `docs/MAINTENANCE_WORKFLOW.md -> docs/operations/maintenance_workflow.md`
- Move: `docs/agent-team-playbook.md -> docs/operations/agent-team-playbook.md`
- Move: `docs/changes -> docs/records/changes`
- Move: `docs/research -> docs/records/research`
- Move: `docs/test0_full -> docs/records/runs/test0-full`
- Move: `docs/test0_smoke -> docs/records/runs/test0-smoke`
- Move: `docs/tmp -> docs/records/archive/tmp`
- Move: `docs/dlc/README.md -> docs/operations/dlc/README.md`
- Move: non-README files from `docs/dlc/` into `docs/records/research/dlc/`
- Move: most dated files from `docs/operations/` into `docs/records/research/operations/`
- Modify: moved docs whose stale `总索引` backlink still points at `overview/docs_index.md`
- Modify: the six known metadata failures after they reach their new destinations

### Task 1: Harden `doc_manager.py` For Two-Layer Indexing

**Files:**
- Create: `tests/test_doc_manager.py`
- Modify: `scripts/doc_manager.py:19-173`

- [ ] **Step 1: Write the failing tests**

```python
#!/usr/bin/env python3
from pathlib import Path
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import scripts.doc_manager as doc_manager


def _write_doc(path: Path, front_matter: str, body: str = "# Test Document\n") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f"---\n{front_matter}\n---\n\n{body}", encoding="utf-8")


def test_generate_index_separates_primary_and_records(tmp_path, monkeypatch):
    docs_dir = tmp_path / "docs"
    index_file = docs_dir / "INDEX.md"

    _write_doc(
        docs_dir / "getting-started" / "quickstart.md",
        """title: Quickstart
created_at: 2026-04-10
updated_at: 2026-04-10
maintainer: OpenCode
status: Active""",
    )
    _write_doc(
        docs_dir / "records" / "research" / "investigation.md",
        """title: Investigation
created_at: 2026-04-10
updated_at: 2026-04-10
maintainer: OpenCode
status: complete
doc_class: record""",
    )

    monkeypatch.setattr(doc_manager, "DOCS_DIR", str(docs_dir))
    monkeypatch.setattr(doc_manager, "INDEX_FILE", str(index_file))

    doc_manager.generate_index()

    output = index_file.read_text(encoding="utf-8")
    assert "## Primary Docs" in output
    assert "### Getting Started" in output
    assert "[Quickstart](getting-started/quickstart.md)" in output
    assert "## Records" in output
    assert "### Research" in output
    assert "[Investigation](records/research/investigation.md)" in output


def test_validate_docs_rejects_invalid_doc_class(tmp_path, monkeypatch, capsys):
    docs_dir = tmp_path / "docs"
    index_file = docs_dir / "INDEX.md"

    _write_doc(
        docs_dir / "getting-started" / "bad.md",
        """title: Broken Class
created_at: 2026-04-10
updated_at: 2026-04-10
maintainer: OpenCode
status: Active
doc_class: mystery""",
    )

    monkeypatch.setattr(doc_manager, "DOCS_DIR", str(docs_dir))
    monkeypatch.setattr(doc_manager, "INDEX_FILE", str(index_file))

    doc_manager.validate_docs()
    output = capsys.readouterr().out
    assert "[INVALID DOC CLASS] getting-started/bad.md: mystery" in output


def test_generate_index_skips_index_and_readme_files(tmp_path, monkeypatch):
    docs_dir = tmp_path / "docs"
    index_file = docs_dir / "INDEX.md"

    (docs_dir / "reference").mkdir(parents=True, exist_ok=True)
    (docs_dir / "reference" / "index.md").write_text("# Reference\n", encoding="utf-8")
    (docs_dir / "operations" / "dlc").mkdir(parents=True, exist_ok=True)
    (docs_dir / "operations" / "dlc" / "README.md").write_text("# DLC\n", encoding="utf-8")

    _write_doc(
        docs_dir / "reference" / "dependencies.md",
        """title: Dependencies
created_at: 2026-04-10
updated_at: 2026-04-10
maintainer: OpenCode
status: Active""",
    )

    monkeypatch.setattr(doc_manager, "DOCS_DIR", str(docs_dir))
    monkeypatch.setattr(doc_manager, "INDEX_FILE", str(index_file))

    doc_manager.generate_index()

    output = index_file.read_text(encoding="utf-8")
    assert "reference/index.md" not in output
    assert "operations/dlc/README.md" not in output
    assert "[Dependencies](reference/dependencies.md)" in output
```

- [ ] **Step 2: Run the tests to confirm they fail against the current implementation**

Run: `python -m pytest tests/test_doc_manager.py -q`

Expected: FAIL because `scripts/doc_manager.py` currently emits directory-based sections like `## Overview`, does not emit `## Primary Docs` / `## Records`, and does not report invalid `doc_class` values.

- [ ] **Step 3: Implement the minimal `doc_manager.py` changes**

```python
SKIP_BASENAMES = {"INDEX.md", "README.md", "index.md"}
VALID_DOC_CLASSES = {"primary", "record", "archive"}
PRIMARY_DIRS = {"getting-started", "how-to", "architecture", "reference", "operations"}


def infer_doc_class(rel_path: str) -> str:
    parts = rel_path.replace("\\", "/").split("/")
    if not parts:
        return "archive"
    if parts[0] in PRIMARY_DIRS:
        return "primary"
    if parts[0] == "records":
        return "record"
    return "archive"


def format_group_heading(rel_path: str) -> str:
    parts = rel_path.replace("\\", "/").split("/")
    if parts[0] == "records" and len(parts) > 1:
        return parts[1].replace("-", " ").title()
    if parts[0] == "reference" and len(parts) > 1:
        return f"Reference / {parts[1].replace('-', ' ').title()}"
    return parts[0].replace("-", " ").title()


def validate_docs():
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

        missing_fields = [field for field in REQUIRED_FIELDS if field not in meta]
        if missing_fields:
            issues.append(f"[INCOMPLETE HEADER] {rel_path}: Missing {missing_fields}")

        doc_class = meta.get("doc_class")
        if doc_class and doc_class not in VALID_DOC_CLASSES:
            issues.append(f"[INVALID DOC CLASS] {rel_path}: {doc_class}")

    if issues:
        print(f"Found {len(issues)} issues:")
        for issue in issues:
            print(issue)
    else:
        print("All documents look good!")


def generate_index():
    print("Generating INDEX.md...")
    md_files = glob.glob(os.path.join(DOCS_DIR, "**/*.md"), recursive=True)
    sections = {"Primary Docs": {}, "Records": {}}

    for file_path in md_files:
        filename = os.path.basename(file_path)
        if filename in SKIP_BASENAMES:
            continue

        rel_path = os.path.relpath(file_path, DOCS_DIR).replace("\\", "/")
        meta = parse_front_matter(file_path)
        title = meta.get("title", filename) if meta else filename
        status = meta.get("status", "Active") if meta else "Unknown"
        updated = meta.get("updated_at", "N/A") if meta else "N/A"
        code_ref = meta.get("code_reference", "") if meta else ""
        doc_class = (meta.get("doc_class") if meta else None) or infer_doc_class(rel_path)

        top_section = "Primary Docs" if doc_class == "primary" else "Records"
        group_heading = format_group_heading(rel_path)

        sections[top_section].setdefault(group_heading, []).append(
            {
                "title": title,
                "path": rel_path,
                "status": status,
                "updated": updated,
                "code_ref": normalize_code_references(code_ref),
            }
        )

    with open(INDEX_FILE, "w", encoding="utf-8") as handle:
        handle.write("# Project Documentation Index\n\n")
        handle.write(f"> Generated at: {datetime.date.today()}\n\n")

        for section_name, grouped_entries in sections.items():
            if not grouped_entries:
                continue

            handle.write(f"## {section_name}\n\n")
            for group_name in sorted(grouped_entries):
                entries = sorted(grouped_entries[group_name], key=lambda item: item["title"])
                handle.write(f"### {group_name}\n\n")
                handle.write("| Document | Status | Last Updated | Related Code |\n")
                handle.write("| :--- | :--- | :--- | :--- |\n")
                for entry in entries:
                    if entry["code_ref"]:
                        code_link = ", ".join(
                            f"[`{os.path.basename(path)}`]({path})" for path in entry["code_ref"]
                        )
                    else:
                        code_link = "-"
                    handle.write(
                        f"| [{entry['title']}]({entry['path']}) | {entry['status']} | {entry['updated']} | {code_link} |\n"
                    )
                handle.write("\n")

    print(f"Index generated at {INDEX_FILE}")
```

- [ ] **Step 4: Re-run the tests after the tool change**

Run: `python -m pytest tests/test_doc_manager.py -q`

Expected: PASS.

- [ ] **Step 5: Optional commit checkpoint if the execution session includes commits**

```bash
git add tests/test_doc_manager.py scripts/doc_manager.py
git commit -m "docs: group index into primary docs and records"
```

### Task 2: Create The New Documentation Skeleton And Landing Pages

**Files:**
- Create: `docs/getting-started/index.md`
- Create: `docs/how-to/index.md`
- Create: `docs/reference/index.md`
- Create: `docs/records/index.md`
- Modify: `docs/index.md`

- [ ] **Step 1: Create the new top-level directories**

Run: `mkdir -p docs/getting-started docs/how-to docs/reference docs/records`

Expected: the four new directories exist alongside the current `docs/` structure.

- [ ] **Step 2: Replace `docs/index.md` with a curated landing page**

```markdown
# Documentation Home

This page is the curated entry point for the repository. Use it for the stable path through the project; use [`INDEX.md`](INDEX.md) when you need the exhaustive generated catalog.

## Start Here

- [Getting Started](getting-started/index.md)
- [Project Overview](getting-started/project_overview.md)
- [Quickstart](getting-started/quickstart.md)
- [FAQ](getting-started/faq.md)

## Common Work

- [How-To Guides](how-to/index.md)
- [Architecture: Processing Pipeline](architecture/pipeline.md)
- [Architecture: Directory Structure](architecture/directory_structure.md)
- [Reference](reference/index.md)
- [Operations](operations/index.md)

## Historical Material

- [Records](records/index.md)
- [Generated Full Index](INDEX.md)
```

- [ ] **Step 3: Add the `getting-started` landing page**

```markdown
# Getting Started

- [Project Overview](project_overview.md)
- [Quickstart](quickstart.md)
- [FAQ](faq.md)
- [Dependencies](../reference/dependencies.md)
- [Documentation Home](../index.md)
```

- [ ] **Step 4: Add the `how-to` landing page**

```markdown
# How-To Guides

- [Interaction Preprocessing](interaction_preprocessing.md)
- [Navigation Preprocessing](navigation_preprocessing.md)
- [Scene Export](export_scenes.md)
- [SimReady CLI](simready.md)
- [Workflow Examples](workflow_examples.md)
- [Documentation Home](../index.md)
```

- [ ] **Step 5: Add the `reference` landing page**

```markdown
# Reference

- [Assets and Materials](assets_and_materials.md)
- [Dependencies](dependencies.md)
- [Modules](modules/)
- [Script Guides](scripts/index.md)
- [Specs Normalizer Guide](specs-normalizer/specs_normalizer_guide.md)
- [Documentation Home](../index.md)
```

- [ ] **Step 6: Add the `records` landing page**

```markdown
# Records

This section contains dated research, run output, change logs, and archived scratch material.

- [Change History](changes/)
- [Research](research/)
- [Run Logs](runs/)
- [Archive](archive/)
- [Generated Full Index](../INDEX.md)
- [Documentation Home](../index.md)
```

- [ ] **Step 7: Quick-check that the landing pages exist and have the expected headings**

Run: `rg -n "^# Documentation Home|^# Getting Started|^# How-To Guides|^# Reference|^# Records" docs/index.md docs/getting-started/index.md docs/how-to/index.md docs/reference/index.md docs/records/index.md`

Expected: one heading match per file.

- [ ] **Step 8: Optional commit checkpoint if the execution session includes commits**

```bash
git add docs/index.md docs/getting-started/index.md docs/how-to/index.md docs/reference/index.md docs/records/index.md
git commit -m "docs: add curated documentation landing pages"
```

### Task 3: Move Onboarding And Task-Oriented Docs Into `getting-started/` And `how-to/`

**Files:**
- Move: `docs/overview/project_overview.md -> docs/getting-started/project_overview.md`
- Move: `docs/usage/quickstart.md -> docs/getting-started/quickstart.md`
- Move: `docs/faq/faq.md -> docs/getting-started/faq.md`
- Move: `docs/examples/workflow_examples.md -> docs/how-to/workflow_examples.md`
- Move: `docs/usage/export_scenes.md -> docs/how-to/export_scenes.md`
- Move: `docs/usage/interaction_preprocessing.md -> docs/how-to/interaction_preprocessing.md`
- Move: `docs/usage/layout_json_for_blender.md -> docs/how-to/layout_json_for_blender.md`
- Move: `docs/usage/layout_usd_to_layout_json_deep_dive.md -> docs/how-to/layout_usd_to_layout_json_deep_dive.md`
- Move: `docs/usage/navigation_preprocessing.md -> docs/how-to/navigation_preprocessing.md`
- Move: `docs/usage/normalize_asset_transforms.md -> docs/how-to/normalize_asset_transforms.md`
- Move: `docs/usage/prep_interaction_root_scene.md -> docs/how-to/prep_interaction_root_scene.md`
- Move: `docs/usage/rebuild_test0_from_legacy.md -> docs/how-to/rebuild_test0_from_legacy.md`
- Move: `docs/usage/rewrite_layout_asset_refs_with_compensation.md -> docs/how-to/rewrite_layout_asset_refs_with_compensation.md`
- Move: `docs/usage/scene_subset_package.md -> docs/how-to/scene_subset_package.md`
- Move: `docs/usage/simready.md -> docs/how-to/simready.md`
- Move: `docs/usage/uid_subset_package.md -> docs/how-to/uid_subset_package.md`
- Move: `docs/usage/usd_to_glb_in_subset.md -> docs/how-to/usd_to_glb_in_subset.md`

- [ ] **Step 1: Move the getting-started documents**

Run: `mv docs/overview/project_overview.md docs/getting-started/project_overview.md && mv docs/usage/quickstart.md docs/getting-started/quickstart.md && mv docs/faq/faq.md docs/getting-started/faq.md`

Expected: `docs/getting-started/` contains `project_overview.md`, `quickstart.md`, and `faq.md`.

- [ ] **Step 2: Move the workflow example and the remaining how-to guides**

Run: `mv docs/examples/workflow_examples.md docs/how-to/workflow_examples.md && for name in export_scenes.md interaction_preprocessing.md layout_json_for_blender.md layout_usd_to_layout_json_deep_dive.md navigation_preprocessing.md normalize_asset_transforms.md prep_interaction_root_scene.md rebuild_test0_from_legacy.md rewrite_layout_asset_refs_with_compensation.md scene_subset_package.md simready.md uid_subset_package.md usd_to_glb_in_subset.md; do mv "docs/usage/$name" "docs/how-to/$name"; done`

Expected: `docs/how-to/` contains all former `usage/` docs except `quickstart.md`, plus `workflow_examples.md`.

- [ ] **Step 3: Remove the emptied legacy directories**

Run: `rmdir docs/overview docs/usage docs/faq docs/examples`

Expected: the command succeeds because the directories are empty.

- [ ] **Step 4: Update the stale landing-page backlink in the moved primary docs**

Use a multi-file `apply_patch` that replaces the old `总索引` line in every file under `docs/getting-started/` and `docs/how-to/`:

```text
Old: > 总索引：../overview/docs_index.md
New: > 总入口：../index.md
```

For `docs/getting-started/project_overview.md`, replace the unique root-local line as well:

```text
Old: > 总索引：docs_index.md
New: > 总入口：../index.md
```

- [ ] **Step 5: Verify the old `overview/docs_index.md` backlink is gone from the moved primary docs**

Run: `rg -n "overview/docs_index.md|docs_index.md" docs/getting-started docs/how-to`

Expected: no matches.

- [ ] **Step 6: Optional commit checkpoint if the execution session includes commits**

```bash
git add docs/getting-started docs/how-to
git commit -m "docs: move onboarding and how-to guides into primary sections"
```

### Task 4: Consolidate Reference Docs And Stable Top-Level Process Docs

**Files:**
- Move: `docs/modules -> docs/reference/modules`
- Move: `docs/scripts -> docs/reference/scripts`
- Move: `docs/specs -> docs/reference/specs`
- Move: `docs/specs_normalizer -> docs/reference/specs-normalizer`
- Move: `docs/references/assets_and_materials.md -> docs/reference/assets_and_materials.md`
- Move: `docs/references/dependencies.md -> docs/reference/dependencies.md`
- Move: `docs/MAINTENANCE_WORKFLOW.md -> docs/operations/maintenance_workflow.md`
- Move: `docs/agent-team-playbook.md -> docs/operations/agent-team-playbook.md`

- [ ] **Step 1: Move the reference subtrees into `docs/reference/`**

Run: `mv docs/modules docs/reference/modules && mv docs/scripts docs/reference/scripts && mv docs/specs docs/reference/specs && mv docs/specs_normalizer docs/reference/specs-normalizer`

Expected: `docs/reference/` now contains `modules/`, `scripts/`, `specs/`, and `specs-normalizer/`.

- [ ] **Step 2: Move the flat reference files into `docs/reference/`**

Run: `mv docs/references/assets_and_materials.md docs/reference/assets_and_materials.md && mv docs/references/dependencies.md docs/reference/dependencies.md && rmdir docs/references`

Expected: `docs/references/` is gone and both files now live directly under `docs/reference/`.

- [ ] **Step 3: Move the stable top-level process docs into `docs/operations/`**

Run: `mv docs/MAINTENANCE_WORKFLOW.md docs/operations/maintenance_workflow.md && mv docs/agent-team-playbook.md docs/operations/agent-team-playbook.md`

Expected: both files now live in `docs/operations/` with stable lowercase names.

- [ ] **Step 4: Fix the backlink depth in the moved reference docs**

Use a multi-file `apply_patch` with these replacements:

```text
For docs directly under docs/reference/:
Old: > 总索引：../overview/docs_index.md
New: > 总入口：../index.md

For docs under docs/reference/modules/, docs/reference/scripts/, docs/reference/specs/, and docs/reference/specs-normalizer/:
Old: > 总索引：../overview/docs_index.md
New: > 总入口：../../index.md
```

- [ ] **Step 5: Verify the old `references/`, `modules/`, `scripts/`, `specs/`, and `specs_normalizer/` directories are no longer top-level entry points**

Run: `ls docs`

Expected: `references`, `modules`, `scripts`, `specs`, and `specs_normalizer` no longer appear at the top level; `reference` does.

- [ ] **Step 6: Optional commit checkpoint if the execution session includes commits**

```bash
git add docs/reference docs/operations
git commit -m "docs: consolidate reference material under reference and operations"
```

### Task 5: Rehome Historical Material Under `records/` And Triage `docs/dlc/` / `docs/operations/`

**Files:**
- Move: `docs/changes -> docs/records/changes`
- Move: `docs/research -> docs/records/research`
- Move: `docs/test0_full -> docs/records/runs/test0-full`
- Move: `docs/test0_smoke -> docs/records/runs/test0-smoke`
- Move: `docs/tmp -> docs/records/archive/tmp`
- Move: `docs/dlc/README.md -> docs/operations/dlc/README.md`
- Move: `docs/dlc/RESEARCH_INDEX.md -> docs/records/research/dlc/RESEARCH_INDEX.md`
- Move: `docs/dlc/debug_normalize_failure.md -> docs/records/research/dlc/debug_normalize_failure.md`
- Move: `docs/dlc/dedup_compensation_failure_analysis.md -> docs/records/research/dlc/dedup_compensation_failure_analysis.md`
- Move: `docs/dlc/universal_compensation_plan.md -> docs/records/research/dlc/universal_compensation_plan.md`
- Move: `docs/operations/asset_mesh_dedup_code_guide.md -> docs/reference/scripts/asset_mesh_dedup_code_guide.md`
- Keep in `docs/operations/`: `environment_setup.md`, `asset_dedup_c1_scaling_workflow.md`, `asset_dedup_c1_scene_instancing_runbook.md`, `grscenes_oss_rclone_runbook.md`, `isaacsim_mdl_workflow.md`, `prep_interaction_root_scene_checklist.md`, `troubleshooting_glb_payload_multimesh.md`, `troubleshooting_interaction_preprocess.md`, `windows_notes.md`, `maintenance_workflow.md`, `agent-team-playbook.md`
- Move: every other dated `.md` file from `docs/operations/` into `docs/records/research/operations/`
- Create: `docs/operations/index.md`

- [ ] **Step 1: Move the obvious record trees into `docs/records/`**

Run: `mkdir -p docs/records/runs docs/records/archive && mv docs/changes docs/records/changes && mv docs/research docs/records/research && mv docs/test0_full docs/records/runs/test0-full && mv docs/test0_smoke docs/records/runs/test0-smoke && mv docs/tmp docs/records/archive/tmp`

Expected: `docs/records/` now contains `changes/`, `research/`, `runs/test0-full/`, `runs/test0-smoke/`, and `archive/tmp/`.

- [ ] **Step 2: Split `docs/dlc/` into evergreen operations docs and historical research docs**

Run: `mkdir -p docs/operations/dlc docs/records/research/dlc && mv docs/dlc/README.md docs/operations/dlc/README.md && mv docs/dlc/RESEARCH_INDEX.md docs/records/research/dlc/RESEARCH_INDEX.md && mv docs/dlc/debug_normalize_failure.md docs/records/research/dlc/debug_normalize_failure.md && mv docs/dlc/dedup_compensation_failure_analysis.md docs/records/research/dlc/dedup_compensation_failure_analysis.md && mv docs/dlc/universal_compensation_plan.md docs/records/research/dlc/universal_compensation_plan.md && rmdir docs/dlc`

Expected: `docs/dlc/` no longer exists; the README remains discoverable via `docs/operations/dlc/README.md`; the analysis files live under `docs/records/research/dlc/`.

- [ ] **Step 3: Move the one stable code guide out of `docs/operations/` before the bulk operations triage**

Run: `mv docs/operations/asset_mesh_dedup_code_guide.md docs/reference/scripts/asset_mesh_dedup_code_guide.md`

Expected: `asset_mesh_dedup_code_guide.md` now lives with the other script guides.

- [ ] **Step 4: Protect the evergreen operations docs, move everything else into records, then restore the keepers**

Run: `mkdir -p docs/operations/_keep docs/records/research/operations && mv docs/operations/environment_setup.md docs/operations/_keep/ && mv docs/operations/asset_dedup_c1_scaling_workflow.md docs/operations/_keep/ && mv docs/operations/asset_dedup_c1_scene_instancing_runbook.md docs/operations/_keep/ && mv docs/operations/grscenes_oss_rclone_runbook.md docs/operations/_keep/ && mv docs/operations/isaacsim_mdl_workflow.md docs/operations/_keep/ && mv docs/operations/prep_interaction_root_scene_checklist.md docs/operations/_keep/ && mv docs/operations/troubleshooting_glb_payload_multimesh.md docs/operations/_keep/ && mv docs/operations/troubleshooting_interaction_preprocess.md docs/operations/_keep/ && mv docs/operations/windows_notes.md docs/operations/_keep/ && mv docs/operations/maintenance_workflow.md docs/operations/_keep/ && mv docs/operations/agent-team-playbook.md docs/operations/_keep/ && mv docs/operations/*.md docs/records/research/operations/ && mv docs/operations/_keep/*.md docs/operations/ && rmdir docs/operations/_keep`

Expected: only the evergreen operations docs remain under `docs/operations/`; all dated investigations and status notes move under `docs/records/research/operations/`.

- [ ] **Step 5: Add the `operations` landing page after the bulk move**

```markdown
# Operations

- [Maintenance Workflow](maintenance_workflow.md)
- [Environment Setup](environment_setup.md)
- [Asset Dedup C1 Scaling Workflow](asset_dedup_c1_scaling_workflow.md)
- [Scene Instancing Runbook](asset_dedup_c1_scene_instancing_runbook.md)
- [GRScenes OSS Rclone Runbook](grscenes_oss_rclone_runbook.md)
- [Troubleshooting: Interaction Preprocess](troubleshooting_interaction_preprocess.md)
- [Troubleshooting: GLB Payload Multi-Mesh](troubleshooting_glb_payload_multimesh.md)
- [Windows Notes](windows_notes.md)
- [Documentation Home](../index.md)
```

- [ ] **Step 6: Confirm the top-level `docs/` surface only contains the new primary and records entry points**

Run: `ls docs`

Expected: the top level contains `architecture`, `getting-started`, `how-to`, `operations`, `reference`, `records`, `superpowers`, `index.md`, and `INDEX.md`; it no longer contains `changes`, `research`, `test0_full`, `test0_smoke`, `tmp`, `dlc`, `usage`, `overview`, `faq`, or `examples`.

- [ ] **Step 7: Optional commit checkpoint if the execution session includes commits**

```bash
git add docs/operations docs/reference docs/records
git commit -m "docs: archive historical material under records"
```

### Task 6: Repair Metadata, Remove Stale Backlinks, And Run Final Validation

**Files:**
- Modify: `docs/records/archive/tmp/claude-proxy-setup-complete.md`
- Modify: `docs/records/archive/tmp/mdl-import-fix-proposal.md`
- Modify: `docs/records/research/codex_proxy_solution1b_analysis.md`
- Modify: `docs/records/runs/test0-full/usd_structure_analysis_20260315.md`
- Modify: `docs/records/research/dlc/RESEARCH_INDEX.md`
- Modify: `docs/records/research/operations/topo_tier2_gate_recovery_plan.md`
- Modify: any remaining file that still contains `overview/docs_index.md` or `docs_index.md`
- Regenerate: `docs/INDEX.md`

- [ ] **Step 1: Fix the six known metadata failures with explicit front matter**

Apply these header updates exactly:

```yaml
# docs/records/archive/tmp/claude-proxy-setup-complete.md
---
title: Claude Proxy Setup Complete
created_at: 2026-03-19
updated_at: 2026-03-19
maintainer: OpenCode
status: completed
doc_class: archive
---

# docs/records/archive/tmp/mdl-import-fix-proposal.md
---
title: MDL Import Fix Proposal
code_reference: scripts/fix_mdl_absolute_imports.py
created_at: 2026-03-16
updated_at: 2026-03-16
maintainer: OpenCode
status: proposal
doc_class: archive
---

# docs/records/research/codex_proxy_solution1b_analysis.md
---
title: Codex Proxy Solution 1B Analysis
created_at: 2026-04-03
updated_at: 2026-04-03
maintainer: OpenCode
status: completed
doc_class: record
---

# docs/records/runs/test0-full/usd_structure_analysis_20260315.md
---
title: USD Structure Analysis (2026-03-15)
created_at: 2026-03-15
updated_at: 2026-03-15
maintainer: OpenCode
status: completed
doc_class: record
---

# docs/records/research/dlc/RESEARCH_INDEX.md
---
title: Dedup Compensation Research Index (2026-03-19)
created_at: 2026-03-19
updated_at: 2026-03-19
maintainer: OpenCode
status: complete
doc_class: record
---

# docs/records/research/operations/topo_tier2_gate_recovery_plan.md
---
title: Topo Tier2 Gate Recovery Plan
code_reference: scripts/compute_vertex_transform.py
created_at: 2026-04-03
updated_at: 2026-04-03
maintainer: OpenCode
status: implemented
doc_class: record
---
```

- [ ] **Step 2: Remove or replace every stale `总索引` backlink**

Use this policy when patching files:

```text
For docs directly under docs/getting-started/, docs/how-to/, docs/architecture/, docs/operations/, and docs/reference/:
replace the stale line with `> 总入口：../index.md`

For docs under docs/reference/modules/, docs/reference/scripts/, docs/reference/specs/, docs/reference/specs-normalizer/, and docs/operations/dlc/:
replace the stale line with `> 总入口：../../index.md`

For any file under docs/records/**:
delete the stale `总索引` line instead of keeping a fragile deep relative path
```

- [ ] **Step 3: Prove the old backlink target is gone from the repository**

Run: `rg -n "overview/docs_index.md|docs_index.md" docs`

Expected: no matches.

- [ ] **Step 4: Run documentation validation and regenerate the full index**

Run: `python scripts/doc_manager.py --validate && python scripts/doc_manager.py --gen-index`

Expected: validation prints `All documents look good!` and index generation writes `docs/INDEX.md` successfully.

- [ ] **Step 5: Confirm the generated index exposes the two-layer structure**

Run: `rg -n "^## Primary Docs$|^## Records$|^### Getting Started$|^### Research$" docs/INDEX.md`

Expected: matches for `## Primary Docs`, `## Records`, `### Getting Started`, and `### Research`.

- [ ] **Step 6: Optional commit checkpoint if the execution session includes commits**

```bash
git add docs scripts/doc_manager.py tests/test_doc_manager.py
git commit -m "docs: reorganize docs into primary and records layers"
```
