from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "summarize_bbox_ab_eval.py"


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def test_summarize_bbox_ab_eval_handles_partial_and_populated_roots(tmp_path: Path) -> None:
    run_root = tmp_path / "run"
    policy_a_cat = run_root / "policy_a" / "chair_bbox_primary_rmse_observe_v1"
    policy_b_cat = run_root / "policy_b" / "chair_bbox_primary_rmse_harder_v1"
    empty_root = tmp_path / "empty_run"

    _write_json(
        policy_a_cat / "01_cert" / "pair_certificate_summary.json",
        {
            "candidate_pairs": 10,
            "eligible_pairs": 9,
            "rejected_pairs": 1,
            "reject_reason_counts": {"rmse_too_high": 1},
            "groups_included": 2,
        },
    )
    _write_json(
        policy_a_cat / "01_cert" / "filtered_mapping.stats.json",
        {
            "mapping_pairs": 9,
            "candidate_pairs": 10,
        },
    )
    _write_json(
        policy_a_cat / "02_apply" / "batch_summary.json",
        {
            "changed_layouts": 2,
            "layouts_total": 3,
            "per_layout": [
                {
                    "scene_file": "layout.usd",
                    "layout_in": "/tmp/GRScenes100/home/SCENE_A/layout.usd",
                    "counts": {"refs_changed": 1, "payloads_changed": 0, "asset_attrs_changed": 0},
                },
                {
                    "scene_file": "layout.usd",
                    "layout_in": "/tmp/GRScenes100/home/SCENE_B/layout.usd",
                    "counts": {"refs_changed": 1, "payloads_changed": 0, "asset_attrs_changed": 0},
                },
            ],
        },
    )
    (policy_a_cat / "02_apply" / "rewrite_reject_ledger.jsonl").write_text(
        json.dumps({"reject_reason": "bbox_gated_requires_compensation"}) + "\n",
        encoding="utf-8",
    )
    _write_json(
        policy_a_cat / "03_audit" / "audit_verdict.json",
        {
            "passed": True,
            "ref_changed_fail_count": 0,
            "total_no_mesh": 0,
            "scenes_error": 0,
            "blocking_reason_counts": {},
        },
    )
    _write_json(
        policy_a_cat / "03_audit" / "placement_pairwise_compare.json",
        {
            "matched_scene_count": 2,
            "aggregate": {
                "global_top_50_worst": [
                    {"scene_id": "home/SCENE_A"},
                    {"scene_id": "home/SCENE_A"},
                    {"scene_id": "home/SCENE_B"},
                ]
            },
        },
    )

    _write_json(
        policy_b_cat / "01_cert" / "pair_certificate_summary.json",
        {
            "candidate_pairs": 10,
            "eligible_pairs": 7,
            "rejected_pairs": 3,
            "reject_reason_counts": {"rmse_too_high": 3},
        },
    )
    _write_json(
        policy_b_cat / "01_cert" / "filtered_mapping.stats.json",
        {
            "mapping_pairs": 7,
            "candidate_pairs": 10,
        },
    )

    out_json = tmp_path / "summary.json"
    out_md = tmp_path / "summary.md"
    subprocess.run(
        [sys.executable, str(SCRIPT), "--run-root", str(run_root), "--out-json", str(out_json), "--out-md", str(out_md)],
        check=True,
        cwd=str(REPO_ROOT),
    )
    payload = json.loads(out_json.read_text(encoding="utf-8"))
    assert payload["policies"]["policy_a"]["totals"]["mapping_pairs"] == 9
    assert payload["policies"]["policy_b"]["totals"]["mapping_pairs"] == 7
    assert payload["comparison"]["totals_delta_b_minus_a"]["mapping_pairs"] == -2
    assert payload["policies"]["policy_a"]["totals"]["changed_scenes"] == 2
    assert payload["policies"]["policy_a"]["apply_reject_reason_counts"] == {
        "bbox_gated_requires_compensation": 1
    }
    assert payload["policies"]["policy_a"]["top_scenes_from_audit_global_worst"][0] == {
        "name": "home/SCENE_A",
        "count": 2,
    }

    empty_json = tmp_path / "empty_summary.json"
    empty_md = tmp_path / "empty_summary.md"
    empty_root.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [sys.executable, str(SCRIPT), "--run-root", str(empty_root), "--out-json", str(empty_json), "--out-md", str(empty_md)],
        check=True,
        cwd=str(REPO_ROOT),
    )
    empty_payload = json.loads(empty_json.read_text(encoding="utf-8"))
    assert empty_payload["policies"]["policy_a"]["discovered_category_count"] == 0
    assert empty_payload["policies"]["policy_b"]["discovered_category_count"] == 0
