import os
import sys


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from scripts import doc_manager


def _write_doc(path, *, title, doc_class=None, code_reference=None):
    lines = [
        "---",
        f"title: {title}",
    ]
    if code_reference is None:
        lines.append("code_reference: scripts/example.py")
    elif isinstance(code_reference, list):
        lines.append("code_reference:")
        lines.extend(f"  - {item}" for item in code_reference)
    elif "\n" in code_reference:
        lines.append("code_reference: |")
        lines.extend(f"  {line}" for line in code_reference.splitlines())
    else:
        lines.append(f"code_reference: {code_reference}")
    lines.extend(
        [
            "created_at: 2026-04-10",
            "updated_at: 2026-04-10",
            "maintainer: tests",
            "status: Active",
        ]
    )
    if doc_class is not None:
        lines.append(f"doc_class: {doc_class}")
    lines.extend(["---", "", f"# {title}", ""])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def test_generate_index_separates_primary_docs_and_records(tmp_path, monkeypatch):
    docs_dir = tmp_path / "docs"
    _write_doc(docs_dir / "usage" / "quickstart.md", title="Quickstart")
    _write_doc(
        docs_dir / "test0_smoke" / "20260410_120000" / "research.md",
        title="Smoke Research",
    )

    monkeypatch.setattr(doc_manager, "DOCS_DIR", str(docs_dir))
    monkeypatch.setattr(doc_manager, "INDEX_FILE", str(docs_dir / "INDEX.md"))

    doc_manager.generate_index()

    index_text = (docs_dir / "INDEX.md").read_text(encoding="utf-8")
    assert "## Primary Docs" in index_text
    assert "## Records" in index_text
    assert "### Getting Started" in index_text
    assert "### Research" in index_text
    assert "[Quickstart](usage/quickstart.md)" in index_text
    assert "[Smoke Research](test0_smoke/20260410_120000/research.md)" in index_text


def test_validate_docs_reports_invalid_doc_class(tmp_path, monkeypatch, capsys):
    docs_dir = tmp_path / "docs"
    _write_doc(docs_dir / "bad.md", title="Bad Doc Class", doc_class="invalid")

    monkeypatch.setattr(doc_manager, "DOCS_DIR", str(docs_dir))

    doc_manager.validate_docs()

    output = capsys.readouterr().out
    assert "[INVALID DOC CLASS] bad.md: invalid" in output


def test_generate_index_skips_index_and_readme_files(tmp_path, monkeypatch):
    docs_dir = tmp_path / "docs"
    _write_doc(docs_dir / "usage" / "guide.md", title="Guide")
    _write_doc(docs_dir / "usage" / "README.md", title="Usage Readme")
    _write_doc(docs_dir / "usage" / "index.md", title="Usage Index")
    _write_doc(docs_dir / "INDEX.md", title="Root Index")

    monkeypatch.setattr(doc_manager, "DOCS_DIR", str(docs_dir))
    monkeypatch.setattr(doc_manager, "INDEX_FILE", str(docs_dir / "INDEX.md"))

    doc_manager.generate_index()

    index_text = (docs_dir / "INDEX.md").read_text(encoding="utf-8")
    assert "[Guide](usage/guide.md)" in index_text
    assert "Usage Readme" not in index_text
    assert "Usage Index" not in index_text
    assert "Root Index" not in index_text


def test_generate_index_uses_stable_group_for_root_level_docs(tmp_path, monkeypatch):
    docs_dir = tmp_path / "docs"
    _write_doc(docs_dir / "agent-team-playbook.md", title="Agent Team Playbook")

    monkeypatch.setattr(doc_manager, "DOCS_DIR", str(docs_dir))
    monkeypatch.setattr(doc_manager, "INDEX_FILE", str(docs_dir / "INDEX.md"))

    doc_manager.generate_index()

    index_text = (docs_dir / "INDEX.md").read_text(encoding="utf-8")
    assert "### General" in index_text
    assert "### Agent-Team-Playbook.Md" not in index_text
    assert "[Agent Team Playbook](agent-team-playbook.md)" in index_text


def test_generate_index_handles_archive_docs_explicitly(tmp_path, monkeypatch):
    docs_dir = tmp_path / "docs"
    _write_doc(
        docs_dir / "operations" / "legacy-runbook.md",
        title="Legacy Runbook",
        doc_class="archive",
    )

    monkeypatch.setattr(doc_manager, "DOCS_DIR", str(docs_dir))
    monkeypatch.setattr(doc_manager, "INDEX_FILE", str(docs_dir / "INDEX.md"))

    doc_manager.generate_index()

    index_text = (docs_dir / "INDEX.md").read_text(encoding="utf-8")
    assert "## Records" in index_text
    assert "### Archive" in index_text
    assert "[Legacy Runbook](operations/legacy-runbook.md)" in index_text


def test_generate_index_uses_records_root_and_keeps_operations_primary(
    tmp_path, monkeypatch
):
    docs_dir = tmp_path / "docs"
    _write_doc(
        docs_dir / "records" / "research" / "foo.md",
        title="Research Note",
    )
    _write_doc(
        docs_dir / "operations" / "runbook.md",
        title="Operations Runbook",
    )

    monkeypatch.setattr(doc_manager, "DOCS_DIR", str(docs_dir))
    monkeypatch.setattr(doc_manager, "INDEX_FILE", str(docs_dir / "INDEX.md"))

    doc_manager.generate_index()

    index_text = (docs_dir / "INDEX.md").read_text(encoding="utf-8")
    assert "## Records\n\n### Research" in index_text
    assert "[Research Note](records/research/foo.md)" in index_text
    assert "## Primary Docs\n\n### Operations" in index_text
    assert "[Operations Runbook](operations/runbook.md)" in index_text
    assert "## Primary Docs\n\n### Records" not in index_text


def test_generate_index_treats_na_code_references_as_empty(tmp_path, monkeypatch):
    docs_dir = tmp_path / "docs"
    _write_doc(
        docs_dir / "records" / "research" / "na.md", title="NA", code_reference="N/A"
    )
    _write_doc(
        docs_dir / "records" / "research" / "guide.md",
        title="Guide",
        code_reference="N/A - 指南文档",
    )

    monkeypatch.setattr(doc_manager, "DOCS_DIR", str(docs_dir))
    monkeypatch.setattr(doc_manager, "INDEX_FILE", str(docs_dir / "INDEX.md"))

    doc_manager.generate_index()

    index_text = (docs_dir / "INDEX.md").read_text(encoding="utf-8")
    assert "[`A`](N/A)" not in index_text
    assert "[`N/A - 指南文档`](N/A - 指南文档)" not in index_text
    assert "| [NA](records/research/na.md) | Active | 2026-04-10 | - |" in index_text
    assert (
        "| [Guide](records/research/guide.md) | Active | 2026-04-10 | - |" in index_text
    )


def test_generate_index_uses_non_empty_label_for_trailing_slash_paths(
    tmp_path, monkeypatch
):
    docs_dir = tmp_path / "docs"
    _write_doc(
        docs_dir / "records" / "research" / "slash.md",
        title="Slash Ref",
        code_reference="~/.local/lib/node_modules/@openai/codex/",
    )

    monkeypatch.setattr(doc_manager, "DOCS_DIR", str(docs_dir))
    monkeypatch.setattr(doc_manager, "INDEX_FILE", str(docs_dir / "INDEX.md"))

    doc_manager.generate_index()

    index_text = (docs_dir / "INDEX.md").read_text(encoding="utf-8")
    assert "[`codex`](~/.local/lib/node_modules/@openai/codex/)" in index_text
    assert "[``](~/.local/lib/node_modules/@openai/codex/)" not in index_text


def test_generate_index_splits_multiline_code_reference_block_scalars(
    tmp_path, monkeypatch
):
    docs_dir = tmp_path / "docs"
    _write_doc(
        docs_dir / "records" / "research" / "block.md",
        title="Block Ref",
        code_reference="scripts/a.py\nscripts/b.py\ncheck_reports/example/",
    )

    monkeypatch.setattr(doc_manager, "DOCS_DIR", str(docs_dir))
    monkeypatch.setattr(doc_manager, "INDEX_FILE", str(docs_dir / "INDEX.md"))

    doc_manager.generate_index()

    index_text = (docs_dir / "INDEX.md").read_text(encoding="utf-8")
    assert (
        "[`a.py`](scripts/a.py), [`b.py`](scripts/b.py), [`example`](check_reports/example/)"
        in index_text
    )
    assert "scripts/a.py\nscripts/b.py" not in index_text


def test_find_references_handles_list_and_multiline_code_references(
    tmp_path, monkeypatch, capsys
):
    docs_dir = tmp_path / "docs"
    _write_doc(
        docs_dir / "records" / "changes" / "list-ref.md",
        title="List Ref",
        code_reference=["scripts/doc_manager.py", "scripts/other.py"],
    )
    _write_doc(
        docs_dir / "records" / "research" / "multiline-ref.md",
        title="Multiline Ref",
        code_reference="scripts/doc_manager.py\nscripts/another.py",
    )

    monkeypatch.setattr(doc_manager, "DOCS_DIR", str(docs_dir))

    doc_manager.find_references("doc_manager.py")

    output = capsys.readouterr().out
    assert "Found 2 documents referencing 'doc_manager.py':" in output
    assert "- records/changes/list-ref.md (List Ref)" in output
    assert "- records/research/multiline-ref.md (Multiline Ref)" in output
