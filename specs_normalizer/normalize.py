"""
规范化导出工具入口 normalize

整体介绍：
- 从源目录（通常为 `target/`）校验原始输出结构是否符合预期；
- 调用材质/资产/场景导出器，将数据复制并重排到规范结构目录（Materials / Assets / Scenes）；
- 会在“导出副本”上改写 USD 内引用（材质/模型）为相对路径；不修改源目录中的 USD。
- 不创建 per-USD 的 `textures` 软链接（该步骤由后续 scripts 统一创建）。
"""

# 解析命令行参数
import argparse
# 导出材质模块
from .exporters.materials import export_materials
# 导出资产（模型）模块
from .exporters.assets import export_assets
# 导出场景模块
from .exporters.scenes import export_scenes
# 校验原始输出结构模块
from .validators.structure import validate_structure

def main():
    # 创建命令行解析器
    ap = argparse.ArgumentParser()
    # 源目录（默认 target）
    ap.add_argument("--src-target", default="target")
    # 规范化输出根目录（默认 export_specs）
    ap.add_argument("--dst-root", default="export_specs")
    # 资产库顶层名（默认 models，可自定义为 GRScenes_assets 等）
    ap.add_argument("--asset-name", default="models")
    # 场景集顶层名（默认 Scenes，可自定义为 GRScenes100 等）
    ap.add_argument("--scene-name", default="Scenes")
    # 场景类别（默认 default，可设置为 home/commercial 等）
    ap.add_argument("--scene-category", default="default")
    # 是否生成最小化注释 JSON
    ap.add_argument("--with-annotations", action="store_true")
    ap.add_argument("--compat-links", action="store_true")
    ap.add_argument("--models-only", action="store_true")
    ap.add_argument("--scenes-only", action="store_true")
    # 限制处理数量（用于快速验证，0表示不限制）
    ap.add_argument("--limit", type=int, default=0)
    # 解析命令行参数
    args = ap.parse_args()
    if args.models_only:
        export_materials(args.src_target, args.dst_root)
        export_assets(args.src_target, args.dst_root, args.asset_name, args.with_annotations, rewrite_mdl_paths=True, limit=args.limit)
    elif args.scenes_only:
        export_scenes(args.src_target, args.dst_root, args.scene_name, args.scene_category, args.with_annotations, asset_name=args.asset_name)
    else:
        ok, issues, summary = validate_structure(args.src_target)
        if not ok:
            print("invalid structure")
            for i in issues:
                print(i)
            return
        export_materials(args.src_target, args.dst_root)
        export_assets(args.src_target, args.dst_root, args.asset_name, args.with_annotations, rewrite_mdl_paths=True, limit=args.limit)
        export_scenes(args.src_target, args.dst_root, args.scene_name, args.scene_category, args.with_annotations, asset_name=args.asset_name)
    if args.compat_links:
        import os
        scenes_root = os.path.join(args.dst_root, args.scene_name, args.scene_category)
        src_mats = os.path.join(args.dst_root, "Material", "mdl")
        src_models = os.path.join(args.dst_root, args.asset_name)
        if os.path.isdir(scenes_root):
            for sid in os.listdir(scenes_root):
                sp = os.path.join(scenes_root, sid)
                if not os.path.isdir(sp):
                    continue
                mats_link = os.path.join(sp, "Materials")
                models_link = os.path.join(sp, "models")
                try:
                    if not os.path.lexists(mats_link):
                        os.symlink(src_mats, mats_link)
                except Exception:
                    pass
                try:
                    if not os.path.lexists(models_link):
                        os.symlink(src_models, models_link)
                except Exception:
                    pass
    # 完成提示
    print("done")

# 作为脚本执行时调用主函数
if __name__ == "__main__":
    main()
