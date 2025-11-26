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
    仅检查 scenes 目录结构：
    - 要求该目录下的每个子目录代表一个场景 id；
    - 每个场景 id 目录中应至少包含 `start_result_fix.usd` 或 `start_result_new.usd`。
    返回 (ok, issues, summary)
    """
    issues = []  # 问题列表
    # 汇总统计：总场景目录数/存在布局数量/缺失布局数量，以及详细列表
    summary = {"total_scene_dirs": 0, "with_layout": 0, "missing_layout": 0, "details": {"missing": [], "present": []}}
    # 若 scenes 路径不存在，立即返回失败与问题
    if not os.path.isdir(src_scenes_dir):
        issues.append("scenes dir not found: " + src_scenes_dir)
        return False, issues, summary
    # 遍历 scenes 目录下的所有子目录（每个视作一个场景 id）
    for sid in os.listdir(src_scenes_dir):
        sp = os.path.join(src_scenes_dir, sid)  # 场景子目录路径
        if not os.path.isdir(sp):
            continue  # 跳过非目录项
        summary["total_scene_dirs"] += 1  # 计数+1
        fixp = os.path.join(sp, "start_result_fix.usd")  # 修复版布局
        newp = os.path.join(sp, "start_result_new.usd")  # 新版布局
        # 任一布局文件存在即视为该场景目录有效
        if os.path.exists(fixp) or os.path.exists(newp):
            summary["with_layout"] += 1
            summary["details"]["present"].append(sid)  # 记录存在布局的场景 id
        else:
            summary["missing_layout"] += 1
            summary["details"]["missing"].append(sid)  # 记录缺失布局的场景 id
    # 通过条件：没有缺失，且至少有一个场景存在布局
    ok = summary["missing_layout"] == 0 and summary["with_layout"] > 0
    # 汇总问题信息：若全部缺失布局或部分缺失，加入 issues
    if summary["with_layout"] == 0:
        issues.append("no start_result_*.usd found in any scene dir")
    if summary["missing_layout"] > 0:
        issues.append("some scene dirs missing layout usd")
    return ok, issues, summary  # 返回结果与统计


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
        ok, issues = validate_structure(args.src_target)
        summary = None
        mode = "full"  # 模式标签

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

