"""
检查工具 check

整体介绍：
- 提供对源目录的结构检查与报告导出能力；支持两种模式：
  1) 全结构检查：验证是否存在 `Materials`、`models`、`scenes`，符合原始输出结构；
  2) 仅场景检查：只验证每个场景目录是否包含 `start_result_fix.usd` 或 `start_result_new.usd`。
- 输出 JSON 报告，便于在远程或本地快速审阅与归档。
"""

# 标准库：路径/文件操作、JSON 序列化、参数解析、时间戳
import os  # 路径拼接与文件系统检查
import json  # 报告输出为 JSON
import argparse  # 命令行参数解析
import time  # 生成报告文件名中的时间戳

# 导入全结构校验函数
from .validators.structure import validate_structure  # 校验 Materials/models/scenes 的存在与内容


def validate_scenes_only(src_scenes_dir):
    """
    仅检查 scenes 目录结构（放宽规则）：
    - 每个场景 id 子目录中只要存在任意一个 USD 文件（.usd/.usda/.usdc）即视为合格。
    - 返回 (ok, issues, summary)
    """
    issues = []
    summary = {"total_scene_dirs": 0, "with_any_usd": 0, "missing_any_usd": 0, "details": {"missing": [], "present": []}}
    if not os.path.isdir(src_scenes_dir):
        issues.append("scenes dir not found: " + src_scenes_dir)
        return False, issues, summary
    for sid in os.listdir(src_scenes_dir):
        sp = os.path.join(src_scenes_dir, sid)
        if not os.path.isdir(sp):
            continue
        summary["total_scene_dirs"] += 1
        has_usd = any(
            os.path.isfile(os.path.join(sp, f)) and f.lower().endswith((".usd", ".usda", ".usdc"))
            for f in os.listdir(sp)
        )
        if has_usd:
            summary["with_any_usd"] += 1
            summary["details"]["present"].append(sid)
        else:
            summary["missing_any_usd"] += 1
            summary["details"]["missing"].append(sid)
    ok = summary["missing_any_usd"] == 0 and summary["with_any_usd"] > 0
    if summary["with_any_usd"] == 0:
        issues.append("no usd found in any scene dir")
    if summary["missing_any_usd"] > 0:
        issues.append("some scene dirs missing usd")
    return ok, issues, summary


def ensure_dir(p):
    # 确保输出目录存在
    os.makedirs(p, exist_ok=True)


def write_report(report, output_path):
    # 在写报告前确保目录存在
    ensure_dir(os.path.dirname(output_path))
    # 写入 JSON 报告，UTF-8 编码与美化缩进
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)


def main():
    # 创建命令行解析器
    ap = argparse.ArgumentParser()
    # 源目录（必需）
    ap.add_argument("--src-target", required=True)
    # 是否仅检查 scenes 目录
    ap.add_argument("--scene-only", action="store_true")
    # 报告输出路径（可选，默认写入项目下 check_reports/）
    ap.add_argument("--output", default=None)
    # 解析参数
    args = ap.parse_args()

    # 根据模式执行相应检查
    if args.scene_only:
        ok, issues, summary = validate_scenes_only(args.src_target)
        mode = "scene_only"  # 模式标签
    else:
        ok, issues, summary = validate_structure(args.src_target)
        mode = "full"

    # 生成默认输出路径：check_reports/report_<mode>_<timestamp>.json
    ts = time.strftime("%Y%m%d-%H%M%S")
    default_out = os.path.join("check_reports", f"report_{mode}_{ts}.json")
    out = args.output or default_out
    # 构造报告对象
    report = {
        "mode": mode,
        "src": args.src_target,
        "ok": ok,
        "issues": issues,
        "summary": summary,
    }
    # 写入报告并打印路径
    write_report(report, out)
    print("written:", out)


# 作为脚本执行时，调用主函数
if __name__ == "__main__":
    main()
