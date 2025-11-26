"""
检查工具 check

整体介绍：
- 提供对源目录的结构检查与报告导出能力；支持全结构与仅场景两种模式。
- 输出 JSON 报告，便于在远程机器上快速审阅与归档。
"""

import os
import json
import argparse
import time

from .validators.structure import validate_structure


def validate_scenes_only(src_scenes_dir):
    """
    仅检查 scenes 目录结构：
    - 要求该目录下的每个子目录代表一个场景 id；
    - 每个场景 id 目录中应至少包含 `start_result_fix.usd` 或 `start_result_new.usd`。
    返回 (ok, issues, summary)
    """
    issues = []
    summary = {"total_scene_dirs": 0, "with_layout": 0, "missing_layout": 0, "details": {"missing": [], "present": []}}
    if not os.path.isdir(src_scenes_dir):
        issues.append("scenes dir not found: " + src_scenes_dir)
        return False, issues, summary
    for sid in os.listdir(src_scenes_dir):
        sp = os.path.join(src_scenes_dir, sid)
        if not os.path.isdir(sp):
            continue
        summary["total_scene_dirs"] += 1
        fixp = os.path.join(sp, "start_result_fix.usd")
        newp = os.path.join(sp, "start_result_new.usd")
        if os.path.exists(fixp) or os.path.exists(newp):
            summary["with_layout"] += 1
            summary["details"]["present"].append(sid)
        else:
            summary["missing_layout"] += 1
            summary["details"]["missing"].append(sid)
    ok = summary["missing_layout"] == 0 and summary["with_layout"] > 0
    if summary["with_layout"] == 0:
        issues.append("no start_result_*.usd found in any scene dir")
    if summary["missing_layout"] > 0:
        issues.append("some scene dirs missing layout usd")
    return ok, issues, summary


def ensure_dir(p):
    os.makedirs(p, exist_ok=True)


def write_report(report, output_path):
    ensure_dir(os.path.dirname(output_path))
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src-target", required=True)
    ap.add_argument("--scene-only", action="store_true")
    ap.add_argument("--output", default=None)
    args = ap.parse_args()

    if args.scene_only:
        ok, issues, summary = validate_scenes_only(args.src_target)
        mode = "scene_only"
    else:
        ok, issues = validate_structure(args.src_target)
        summary = None
        mode = "full"

    ts = time.strftime("%Y%m%d-%H%M%S")
    default_out = os.path.join("check_reports", f"report_{mode}_{ts}.json")
    out = args.output or default_out
    report = {
        "mode": mode,
        "src": args.src_target,
        "ok": ok,
        "issues": issues,
        "summary": summary,
    }
    write_report(report, out)
    print("written:", out)


if __name__ == "__main__":
    main()

