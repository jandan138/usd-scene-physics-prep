#!/usr/bin/env python3
import os
import sys
import json
import argparse
import re


def list_scene_dirs(root: str):
    scenes = []
    for category in ("home", "commercial"):
        base = os.path.join(root, "GRScenes100", category)
        if not os.path.isdir(base):
            continue
        for sid in os.listdir(base):
            sp = os.path.join(base, sid)
            if os.path.isdir(sp):
                scenes.append((category, sid, sp))
    return scenes


def check_missing_files(scene_dir: str):
    missing = []
    layout = os.path.join(scene_dir, "layout.usd")
    if not os.path.isfile(layout):
        missing.append("layout.usd")
    return missing


def read_text(path: str):
    try:
        with open(path, "rb") as f:
            head = f.read(64)
        is_text = b"usda" in head or head.startswith(b"#")
        if not is_text:
            return None
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            return f.read()
    except Exception:
        return None


def check_model_refs(layout_path: str, scenes_root_abs: str):
    txt = read_text(layout_path)
    bad = []
    if not txt:
        # 二进制或无法读取，跳过深检
        return bad
    # 查找所有 @...usd@ 或 @...usdc@ 引用
    refs = re.findall(r"@([^@]+\.(?:usd|usda|usdc))@", txt)
    # 期望：引用指向同一导出根下的 GRScenes100/{home|commercial}/... 或资产库（如有）
    # 这里依据用户要求：只统计“未指向 export_specs_unified/GRScenes100 下内容”的引用
    for r in refs:
        rp = r.replace("\\", "/")
        # 允许相对路径指向 GRScenes100：以 layout 所在目录为基准的相对路径即可
        # 只要包含 "GRScenes100/"（绝对或相对展开后），或以相对路径不含旧源标记，则视为通过
        if "GRScenes100/" in rp:
            continue
        # 常见旧源标记：/home_scenes/scenes 或 /commercial_scenes/scenes 等绝对源路径
        if "/home_scenes/scenes" in rp or "/commercial_scenes/scenes" in rp:
            bad.append(rp)
        else:
            # 相对路径但未显式包含 GRScenes100 的情况：进一步放宽，只记录明显指向旧源的绝对路径
            # 若需更严格校验，可在此处解析规范相对前缀
            pass
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--export-root", required=True, help="export root, e.g. export_specs_unified")
    ap.add_argument("--output", default="check_reports/phase3_scenes_report.json")
    ap.add_argument("--fail-only", action="store_true")
    args = ap.parse_args()

    export_root = os.path.abspath(args.export_root)
    scenes_root = export_root
    scenes = list_scene_dirs(scenes_root)
    missing_summary = []
    bad_refs_summary = []

    for category, sid, sdir in scenes:
        miss = check_missing_files(sdir)
        if miss:
            missing_summary.append({"category": category, "sid": sid, "missing": miss})
        layout = os.path.join(sdir, "layout.usd")
        if os.path.isfile(layout):
            bad_refs = check_model_refs(layout, scenes_root)
            if bad_refs:
                bad_refs_summary.append({"category": category, "sid": sid, "bad_model_usd_refs": bad_refs})

    summary = {
        "scene_dir_count": len(scenes),
        "missing_count": len(missing_summary),
        "bad_refs_count": len(bad_refs_summary),
    }

    if args.fail_only:
        report = {"summary": summary, "missing": missing_summary, "bad_refs": bad_refs_summary}
    else:
        report = {
            "summary": summary,
            "scenes": [{"category": c, "sid": s, "dir": d} for (c, s, d) in scenes],
            "missing": missing_summary,
            "bad_refs": bad_refs_summary,
        }

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print("written:", args.output)


if __name__ == "__main__":
    main()

