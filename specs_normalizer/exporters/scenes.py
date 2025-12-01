"""
场景导出模块 scenes

整体介绍：
- 从源目录 `scenes/<sid>/start_result_fix.usd|start_result_new.usd` 选择布局文件，复制到规范结构 `Scene_name/Scene_category/{sid}/layout.usd`。
- 可选生成 `{sid}_annotation.json`，统计 `/Root/Meshes` 子层级下的对象数量（如可用）。
"""

# 导入标准库与工具函数
import os
import json
import re
from ..utils.fs import ensure_dir, copy_file
from ..utils.scene_rewrite import rewrite_scene_refs_inplace

# 选择一个布局文件（优先 raw，其次 fix，其次 new，再否则任取一个 USD）
def choose_layout(scene_dir):
    rawp = os.path.join(scene_dir, "start_result_raw.usd")
    fixp = os.path.join(scene_dir, "start_result_fix.usd")
    newp = os.path.join(scene_dir, "start_result_new.usd")
    if os.path.exists(rawp):
        return rawp
    if os.path.exists(fixp):
        return fixp
    if os.path.exists(newp):
        return newp
    for f in os.listdir(scene_dir):
        p = os.path.join(scene_dir, f)
        if os.path.isfile(p) and f.lower().endswith((".usd", ".usda", ".usdc")):
            return p
    return None

# 导出场景到规范结构
def export_scenes(src_root, dst_root, scene_name, scene_category, with_annotations, scene_ids=None):
    scenes_root = os.path.join(src_root, "scenes")                  # 源场景根目录
    dest_root = os.path.join(dst_root, scene_name, scene_category)   # 目标场景顶层目录
    ensure_dir(dest_root)                                            # 确保存在
    try:
        from pxr import Usd                                        # USD 可选导入用于统计
    except Exception:
        Usd = None                                                  # 未安装则置空
    for sid in os.listdir(scenes_root):
        if scene_ids and sid not in scene_ids:
            continue
        sp = os.path.join(scenes_root, sid)
        if not os.path.isdir(sp):
            continue
        layout = choose_layout(sp)                                  # 选择布局文件
        if not layout:
            continue
        out_dir = os.path.join(dest_root, sid)                      # 目标场景目录
        ensure_dir(out_dir)
        copy_file(layout, os.path.join(out_dir, "layout.usd"))
        for f in os.listdir(sp):
            srcf = os.path.join(sp, f)
            if not os.path.isfile(srcf):
                continue
            if not f.lower().endswith((".usd", ".usda", ".usdc")):
                continue
            if os.path.abspath(srcf) == os.path.abspath(layout):
                continue
            copy_file(srcf, os.path.join(out_dir, f))
        mats_abs = os.path.join(dst_root, "Material", "mdl")
        candidates = [
            os.path.join(dst_root, "models"),
            os.path.join(dst_root, "GRScenes_assets"),
            os.path.join(dst_root, "MesaTask_assets"),
        ]
        models_abs = None
        for c in candidates:
            if os.path.isdir(c):
                models_abs = c
                break
        if models_abs is None:
            models_abs = os.path.join(dst_root, "models")
        mats_rel = os.path.relpath(mats_abs, out_dir)
        models_rel = os.path.relpath(models_abs, out_dir)
        for uf in os.listdir(out_dir):
            if not uf.lower().endswith((".usda", ".usd", ".usdc")):
                continue
            usdf = os.path.join(out_dir, uf)
            try:
                rewrite_scene_refs_inplace(usdf, mats_abs, models_abs, relative_base=out_dir)
            except Exception:
                # 文本兜底重写
                try:
                    with open(usdf, "rb") as f:
                        head = f.read(64)
                    is_text = b"usda" in head or head.startswith(b"#")
                    if not is_text:
                        continue
                    with open(usdf, "r", encoding="utf-8", errors="ignore") as f:
                        txt = f.read()
                    def _sub(m):
                        p = m.group(1)
                        if "Materials/" in p:
                            idx = p.find("Materials/")
                            rest = p[idx + len("Materials/"):]
                            return "@" + mats_rel + "/" + rest + "@"
                        if "models/" in p:
                            idx = p.find("models/")
                            rest = p[idx + len("models/"):]
                            return "@" + models_rel + "/" + rest + "@"
                        return "@" + p + "@"
                    new_txt = re.sub(r"@([^@]+)@", _sub, txt)
                    if new_txt != txt:
                        with open(usdf, "w", encoding="utf-8") as f:
                            f.write(new_txt)
                except Exception:
                    pass
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
