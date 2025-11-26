import os
import json
from ..utils.fs import ensure_dir, copy_file

def choose_layout(scene_dir):
    fixp = os.path.join(scene_dir, "start_result_fix.usd")
    newp = os.path.join(scene_dir, "start_result_new.usd")
    if os.path.exists(fixp):
        return fixp
    if os.path.exists(newp):
        return newp
    return None

def export_scenes(src_root, dst_root, scene_name, scene_category, with_annotations):
    scenes_root = os.path.join(src_root, "scenes")
    dest_root = os.path.join(dst_root, scene_name, scene_category)
    ensure_dir(dest_root)
    try:
        from pxr import Usd
    except Exception:
        Usd = None
    for sid in os.listdir(scenes_root):
        sp = os.path.join(scenes_root, sid)
        if not os.path.isdir(sp):
            continue
        layout = choose_layout(sp)
        if not layout:
            continue
        out_dir = os.path.join(dest_root, sid)
        ensure_dir(out_dir)
        copy_file(layout, os.path.join(out_dir, "layout.usd"))
        if with_annotations:
            ann = {"sid": sid}
            if Usd:
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

