"""
场景导出模块 scenes

整体介绍：
- 从源目录 `scenes/<sid>/start_result_fix.usd|start_result_new.usd` 选择布局文件，复制到规范结构 `Scene_name/Scene_category/{sid}/layout.usd`。
- 可选生成 `{sid}_annotation.json`，统计 `/Root/Meshes` 子层级下的对象数量（如可用）。
"""

# 导入标准库与工具函数
import os
import json
from ..utils.fs import ensure_dir, copy_file

# 选择一个布局文件（优先 fix，无则 new）
def choose_layout(scene_dir):
    fixp = os.path.join(scene_dir, "start_result_fix.usd")  # 修复版布局
    newp = os.path.join(scene_dir, "start_result_new.usd")  # 新版布局
    if os.path.exists(fixp):                                 # 优先使用修复版
        return fixp
    if os.path.exists(newp):                                 # 备选使用新版
        return newp
    return None                                              # 都不存在返回空

# 导出场景到规范结构
def export_scenes(src_root, dst_root, scene_name, scene_category, with_annotations):
    scenes_root = os.path.join(src_root, "scenes")                  # 源场景根目录
    dest_root = os.path.join(dst_root, scene_name, scene_category)   # 目标场景顶层目录
    ensure_dir(dest_root)                                            # 确保存在
    try:
        from pxr import Usd                                        # USD 可选导入用于统计
    except Exception:
        Usd = None                                                  # 未安装则置空
    for sid in os.listdir(scenes_root):                             # 遍历场景 id
        sp = os.path.join(scenes_root, sid)
        if not os.path.isdir(sp):
            continue
        layout = choose_layout(sp)                                  # 选择布局文件
        if not layout:
            continue
        out_dir = os.path.join(dest_root, sid)                      # 目标场景目录
        ensure_dir(out_dir)
        copy_file(layout, os.path.join(out_dir, "layout.usd"))     # 复制布局文件
        if with_annotations:                                        # 可选生成注释
            ann = {"sid": sid}
            if Usd:                                                 # 若 USD 可用则统计 Meshes 子层级
                try:
                    stage = Usd.Stage.Open(layout)
                    root = stage.GetPrimAtPath("/Root/Meshes")
                    counts = {}
                    for child in root.GetChildren():
                        counts[child.GetName()] = len(child.GetChildren())
                    ann["counts"] = counts
                except Exception:
                    ann["counts"] = {}
            with open(os.path.join(out_dir, sid + "_annotation.json"), "w", encoding="utf-8") as f:
                json.dump(ann, f, ensure_ascii=False, indent=2)
