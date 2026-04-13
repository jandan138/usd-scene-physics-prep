#!/usr/bin/env python3

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
