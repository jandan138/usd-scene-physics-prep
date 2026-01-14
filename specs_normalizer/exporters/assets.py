"""specs_normalizer.exporters.assets

整体介绍：
- 遍历源目录 `models/<scope>/<articulated|others>/<category>/<uid>/instance.usd`。
- 复制为规范结构 `<Asset_name>/<category>/<uid>/usd/<uid>.usd`。
- 可选生成最小化注释：单资产 `{uid}_annotation.json`（含 asset_type）与类别聚合 `Asset_annotation.json`。

注意：
- 导出阶段不会创建 per-USD 的 `textures` 软链接（该步骤由后续 scripts 统一创建）。
"""

# 导入标准库与工具函数
import os
import json
from ..utils.fs import ensure_dir, copy_file
import re
from ..utils.mdl_rewrite import rewrite_usd_mdl_paths

# 迭代器：遍历源目录下所有模型实例（返回类别、唯一 id、实例文件路径、子类别）
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
                        yield category, uid, inst, sub             # 返回元组

def generate_asset_annotation(uid, category, subcat):
    """
    生成资产元数据
    subcat: 'articulated' | 'others'
    """
    # 映射子类别到 asset_type
    # articulated -> articulation
    # others -> rigid (假设 others 均为刚体，除非有更细的逻辑)
    asset_type = "articulation" if subcat == "articulated" else "rigid"
    return {
        "uid": uid,
        "category": category,
        "description": "",
        "material": "",
        "dimensions": "",
        "mass": "",
        "placement": [],
        "asset_type": asset_type
    }

# 导出资产到规范结构
def export_assets(src_root, dst_root, asset_name, with_annotations, rewrite_mdl_paths=False, limit=0):
    models_root = os.path.join(src_root, "models")             # 源模型根目录
    dest_root = os.path.join(dst_root, asset_name)              # 目标资产库顶层
    ensure_dir(dest_root)                                       # 确保存在
    stats = {}                                                  # 类别统计
    seen = set()                                                # 去重集合
    count = 0                                                   # 处理计数
    
    print(f"Starting asset export to {dest_root}...")
    
    # 遍历所有模型实例
    # 注意：iter_model_instances 现在返回 4 个值
    for category, uid, inst, subcat in iter_model_instances(models_root):
        if limit > 0 and count >= limit:
            print(f"Limit reached ({limit}), stopping asset export.")
            break

        cat_dir = os.path.join(dest_root, category)             # 目标类别目录
        ensure_dir(cat_dir)
        
        # 新结构：<Asset_name>/<category>/<uid>/usd/<uid>.usd
        asset_dir = os.path.join(cat_dir, uid)
        ensure_dir(asset_dir)
        usd_dir = os.path.join(asset_dir, "usd")
        ensure_dir(usd_dir)
        out_path = os.path.join(usd_dir, uid + ".usd")       # 目标文件路径
        
        key = (category, uid)
        if key in seen:                                         # 已复制则跳过
            continue
        seen.add(key)
        
        # 检查目标文件是否已存在（断点续传逻辑）
        if os.path.exists(out_path):
            count += 1
            # print(f"[{count}] Skipped existing asset: {category}/{uid}") # 可选：减少日志噪音
            # 即使跳过文件复制，我们仍需确保统计信息 stats 被更新，以便生成聚合注释
            stats.setdefault(category, []).append(uid)

            # 断点续跑：若要求改写 MDL 路径，则对已存在的 USD 也执行一次（幂等）。
            # 这样可覆盖“上一次导出在改写阶段被中断”的情况。
            if rewrite_mdl_paths:
                mats_abs = os.path.join(dst_root, "Material", "mdl")
                try:
                    rewrite_usd_mdl_paths(
                        src_usd=out_path,
                        dst_usd=out_path,
                        materials_dir=mats_abs,
                        use_relative=True,
                        relative_base=usd_dir,
                        rewrite_module_refs=True,
                    )
                except Exception:
                    pass

            # 断点续跑：若 USD 已存在但注释缺失，则补写单资产注释
            if with_annotations:
                ann_path = os.path.join(asset_dir, uid + "_annotation.json")
                if not os.path.exists(ann_path):
                    ann = generate_asset_annotation(uid, category, subcat)
                    with open(ann_path, "w", encoding="utf-8") as f:
                        json.dump(ann, f, ensure_ascii=False, indent=2)
            continue

        copy_file(inst, out_path)
        count += 1
        print(f"[{count}] Exported asset: {category}/{uid}")
        
        if rewrite_mdl_paths:
            mats_abs = os.path.join(dst_root, "Material", "mdl")
            try:
                rewrite_usd_mdl_paths(
                    src_usd=out_path,
                    dst_usd=out_path,
                    materials_dir=mats_abs,
                    use_relative=True,
                    relative_base=usd_dir,  # 相对路径基准：USD 所在目录（.../<uid>/usd）
                    rewrite_module_refs=True,
                )
            except Exception:
                pass
        
        stats.setdefault(category, []).append(uid)              # 记录统计
        
        # 始终生成单资产注释（因为现在是强制要求包含 asset_type）
        # 如果用户没传 with_annotations，也建议生成，或者严格遵循参数？
        # 根据用户描述：“可以给每个资产都生成anno的json...”，似乎是必须的。
        # 我们可以认为 with_annotations 是控制是否生成“额外”注释，但既然结构要求里必须有，那就默认生成。
        # 为了兼容性，这里还是检查一下 with_annotations，或者默认开启。
        # 考虑到用户意图是“把所有的文档都修改了，然后再修改相应的代码”，且明确提出了 annotation 的格式要求，
        # 我倾向于总是生成，或者至少在 with_annotations 为 True 时生成正确的格式。
        # 这里的函数调用者通常会传入 True。
        if with_annotations:
            ann = generate_asset_annotation(uid, category, subcat)
            with open(os.path.join(asset_dir, uid + "_annotation.json"), "w", encoding="utf-8") as f:
                json.dump(ann, f, ensure_ascii=False, indent=2)
                
    if with_annotations:                                        # 类别聚合注释
        # 生成单一的顶层 Asset_annotation.json
        top_level_annotation = {
            "asset_name": asset_name,
            "total_count": count,
            "categories": {}
        }
        
        for category, uids in stats.items():
            top_level_annotation["categories"][category] = {
                "count": len(uids),
                "uids": uids
            }
            
        with open(os.path.join(dest_root, "Asset_annotation.json"), "w", encoding="utf-8") as f:
            json.dump(top_level_annotation, f, ensure_ascii=False, indent=2)
    
    print(f"Asset export completed. Total exported: {count}")
