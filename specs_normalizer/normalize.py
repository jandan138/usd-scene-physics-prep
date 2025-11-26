import argparse
from .exporters.materials import export_materials
from .exporters.assets import export_assets
from .exporters.scenes import export_scenes
from .validators.structure import validate_structure

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src-target", default="target")
    ap.add_argument("--dst-root", default="export_specs")
    ap.add_argument("--asset-name", default="models")
    ap.add_argument("--scene-name", default="Scenes")
    ap.add_argument("--scene-category", default="default")
    ap.add_argument("--with-annotations", action="store_true")
    args = ap.parse_args()
    ok, issues = validate_structure(args.src_target)
    if not ok:
        print("invalid structure")
        for i in issues:
            print(i)
        return
    export_materials(args.src_target, args.dst_root)
    export_assets(args.src_target, args.dst_root, args.asset_name, args.with_annotations)
    export_scenes(args.src_target, args.dst_root, args.scene_name, args.scene_category, args.with_annotations)
    print("done")

if __name__ == "__main__":
    main()
