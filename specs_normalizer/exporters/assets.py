"""
资产导出模块 assets

整体介绍：
- 遍历源目录 `models/<scope>/<articulated|others>/<category>/<uid>/instance.usd`，复制为规范结构 `Asset_name/<category>/<uid>.usd`。
- 可选生成最小化注释：单资产 `{uid}_annotation.json` 与类别聚合 `Asset_annotation.json`。
"""

# 导入标准库与工具函数
import os
import json
from ..utils.fs import ensure_dir, copy_file
import re
from ..utils.mdl_rewrite import rewrite_usd_mdl_paths

# 迭代器：遍历源目录下所有模型实例（返回类别、唯一 id、实例文件路径）
def iter_model_instances(models_root):
    scopes = ["layout", "object"]                     # 顶层范围（布局/对象）
    subcats = ["articulated", "others"]               # 子类别（带关节/其它）
    for s in scopes:                                    # 遍历范围
        for sub in subcats:                             # 遍历子类别
            base = os.path.join(models_root, s, sub)    # 拼接路径
            if not os.path.isdir(base):                 # 若不存在则跳过
                continue
            for category in os.listdir(base):           # 遍历具体类别
                cat_path = os.path.join(base, category)
                if not os.path.isdir(cat_path):
                    continue
                for uid in os.listdir(cat_path):        # 遍历唯一 id (hash)
                    u_path = os.path.join(cat_path, uid)
                    if not os.path.isdir(u_path):
                        continue
                    inst = os.path.join(u_path, "instance.usd")  # 实例文件路径
                    if os.path.exists(inst):                       # 确认存在
                        yield category, uid, inst                  # 返回元组

# 导出资产到规范结构
def export_assets(src_root, dst_root, asset_name, with_annotations, rewrite_mdl_paths=False):
    models_root = os.path.join(src_root, "models")             # 源模型根目录
    dest_root = os.path.join(dst_root, asset_name)              # 目标资产库顶层
    ensure_dir(dest_root)                                       # 确保存在
    stats = {}                                                  # 类别统计
    seen = set()                                                # 去重集合
    for category, uid, inst in iter_model_instances(models_root):
        cat_dir = os.path.join(dest_root, category)             # 目标类别目录
        ensure_dir(cat_dir)
        out_path = os.path.join(cat_dir, uid + ".usd")         # 目标文件路径
        key = (category, uid)
        if key in seen:                                         # 已复制则跳过
            continue
        seen.add(key)
        copy_file(inst, out_path)
        if rewrite_mdl_paths:
            mats_abs = os.path.join(dst_root, "Material", "mdl")
            try:
                rewrite_usd_mdl_paths(
                    src_usd=out_path,
                    dst_usd=out_path,
                    materials_dir=mats_abs,
                    use_relative=True,
                    relative_base=cat_dir,
                    rewrite_module_refs=True,
                )
            except Exception:
                pass
        stats.setdefault(category, []).append(uid)              # 记录统计
        if with_annotations:                                    # 可选生成单资产注释
            ann = {"uid": uid, "category": category}
            with open(os.path.join(cat_dir, uid + "_annotation.json"), "w", encoding="utf-8") as f:
                json.dump(ann, f, ensure_ascii=False, indent=2)
    if with_annotations:                                        # 类别聚合注释
        for category, uids in stats.items():
            out = {"category": category, "count": len(uids), "uids": uids}
            with open(os.path.join(dest_root, category, "Asset_annotation.json"), "w", encoding="utf-8") as f:
                json.dump(out, f, ensure_ascii=False, indent=2)
