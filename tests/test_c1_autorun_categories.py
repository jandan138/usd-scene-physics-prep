#!/usr/bin/env python3

import errno
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "c1_autorun_categories.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("c1_autorun_categories", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


mod = _load_module()


def test_build_bbox_audit_cmd_includes_certificate_jsonl_and_mode_reports_dir(tmp_path):
    dataset_root = tmp_path / "dataset"
    plan = mod.build_bbox_plan(
        "chair",
        c1_bulk_dir=tmp_path / "bulk",
        policy_tag="bbox_primary",
        out_version="v1",
        group_label="c1_chair_bbox",
    )
    changed_scene_list_json = plan.apply_dir / "changed_scene_ids.json"
    args = SimpleNamespace(
        bbox_policy="bbox_primary",
        eps_bbox=0.01,
        eps_pos=0.02,
        eps_angle=0.03,
        eps_geom=0.04,
        mode_reports_dir="check_reports/modes",
    )

    cmd = mod._build_bbox_audit_cmd(
        dataset_root=dataset_root,
        plan=plan,
        changed_scene_list_json=changed_scene_list_json,
        args=args,
    )

    assert cmd[0] == str(mod.ISAAC_PY)
    assert cmd[1] == str(mod.SCRIPT_AUDIT.relative_to(mod.REPO_ROOT))

    def _assert_flag_value(flag, expected):
        idx = cmd.index(flag)
        assert cmd[idx + 1] == expected

    _assert_flag_value("--left-root", str(dataset_root))
    _assert_flag_value("--right-root", str(dataset_root))
    _assert_flag_value("--left-mode", "current")
    _assert_flag_value("--right-mode", "current")
    _assert_flag_value("--right-layout-name", plan.out_name)
    _assert_flag_value("--label", f"{plan.category}_{args.bbox_policy}")
    _assert_flag_value("--out", str(plan.audit_report_json))
    _assert_flag_value("--verdict-out", str(plan.audit_verdict_json))
    _assert_flag_value("--scene-list-json", str(changed_scene_list_json))
    _assert_flag_value("--bbox-policy", args.bbox_policy)
    _assert_flag_value("--eps-bbox", str(args.eps_bbox))
    _assert_flag_value("--eps-pos", str(args.eps_pos))
    _assert_flag_value("--eps-angle", str(args.eps_angle))
    _assert_flag_value("--eps-geom", str(args.eps_geom))
    assert "--allow-no-mesh" in cmd

    assert "--certificate-jsonl" in cmd
    cert_idx = cmd.index("--certificate-jsonl")
    assert cmd[cert_idx + 1] == str(plan.certificate_jsonl)

    assert "--mode-reports-dir" in cmd
    mode_idx = cmd.index("--mode-reports-dir")
    assert cmd[mode_idx + 1] == args.mode_reports_dir


def test_run_tolerates_log_write_enospc(monkeypatch, tmp_path, capsys):
    class FakeProc:
        def __init__(self):
            self.stdout = iter(["line-1\n", "line-2\n"])

        def wait(self):
            return 0

    class FlakyLog:
        def __init__(self):
            self.write_count = 0
            self.closed = False

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            self.close()
            return False

        def write(self, text):
            self.write_count += 1
            if self.write_count >= 4:
                raise OSError(errno.ENOSPC, "No space left on device")

        def flush(self):
            return None

        def close(self):
            self.closed = True

    flaky = FlakyLog()
    monkeypatch.setattr(Path, "open", lambda self, *args, **kwargs: flaky)
    monkeypatch.setattr(mod.subprocess, "Popen", lambda *args, **kwargs: FakeProc())

    rc = mod._run(["echo", "hi"], cwd=tmp_path, log_path=tmp_path / "run.log")

    assert rc == 0
    err = capsys.readouterr().err
    assert "run.log" in err
    assert flaky.closed is True


def test_is_bbox_done_requires_step6_complete_in_apply_mode(tmp_path):
    category_root = tmp_path / "blanket_bbox_primary_rmse_observe_v1"
    audit = category_root / "03_audit" / "audit_verdict.json"
    audit.parent.mkdir(parents=True, exist_ok=True)
    audit.write_text('{"passed": true}\n', encoding="utf-8")

    step6 = category_root / "04_step6"
    step6.mkdir(parents=True, exist_ok=True)
    (step6 / "post_promote_full_usd_scan_excluding_backups_pxr.json").write_text(
        '{"hit_files": 0}\n', encoding="utf-8"
    )

    assert mod._is_bbox_done(category_root, step6_mode="apply") is False


def test_is_bbox_done_accepts_complete_step6_in_apply_mode(tmp_path):
    category_root = tmp_path / "bed_bbox_primary_rmse_observe_v1"
    audit = category_root / "03_audit" / "audit_verdict.json"
    audit.parent.mkdir(parents=True, exist_ok=True)
    audit.write_text('{"passed": true}\n', encoding="utf-8")

    step6 = category_root / "04_step6"
    step6.mkdir(parents=True, exist_ok=True)
    (step6 / "post_promote_full_usd_scan_excluding_backups_pxr.json").write_text(
        '{"hit_files": 0}\n', encoding="utf-8"
    )
    (step6 / "post_soft_delete_layout_scan_pxr.json").write_text(
        '{"hit_layouts": 0}\n', encoding="utf-8"
    )

    assert mod._is_bbox_done(category_root, step6_mode="apply") is True


def test_is_bbox_done_keeps_audit_only_semantics_outside_apply_mode(tmp_path):
    category_root = tmp_path / "chair_bbox_primary_rmse_observe_v1"
    audit = category_root / "03_audit" / "audit_verdict.json"
    audit.parent.mkdir(parents=True, exist_ok=True)
    audit.write_text('{"passed": true}\n', encoding="utf-8")

    assert mod._is_bbox_done(category_root, step6_mode="dry_run") is True
