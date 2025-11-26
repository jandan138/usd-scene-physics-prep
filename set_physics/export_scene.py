import json
import os
import shutil


def copy_file(src, dst):
    try:
        if not os.path.exists(src):
            print(f"Error! File {src} not found!")
            return

        dst_dir = os.path.dirname(dst)
        if not os.path.exists(dst_dir):
            os.makedirs(dst_dir)

        shutil.copyfile(src, dst)
    except Exception as e:
        print(f"Error! Copy file from {src} to {dst} failed!")


def export(src_dir, target_dir):
    model_refs_file = os.path.abspath(os.path.join(src_dir, "model_refs.json"))
    material_refs_file = os.path.abspath(os.path.join(src_dir, "material_refs.json"))

    with open(model_refs_file, "r") as model_f:
        model_refs_list = json.load(model_f)
    
    with open(material_refs_file, "r") as material_f:
        material_refs_list = json.load(material_f)
    
    for model_ref in model_refs_list:
        copy_file(os.path.abspath(os.path.join(src_dir, model_ref)), os.path.abspath(os.path.join(target_dir, model_ref)))
    
    for material_ref in material_refs_list:
        copy_file(os.path.abspath(os.path.join(src_dir, material_ref)), os.path.abspath(os.path.join(target_dir, material_ref)))
    
    copy_file(os.path.abspath(os.path.join(src_dir, "start_result_dynamic.usd")), os.path.abspath(os.path.join(target_dir, "start_result_dynamic.usd")))
    static_usd_file = os.path.abspath(os.path.join(src_dir, "start_result_fix.usd"))
    if os.path.exists(static_usd_file):
        copy_file(static_usd_file, os.path.abspath(os.path.join(target_dir, "start_result_fix.usd")))
    else:
        static_usd_file = os.path.abspath(os.path.join(src_dir, "start_result_new.usd"))
        copy_file(static_usd_file, os.path.abspath(os.path.join(target_dir, "start_result_new.usd")))
    
    for root, dirs, files in os.walk(os.path.abspath(os.path.join(target_dir, "models"))):
        for _ in files:
            link_path = os.path.join(root, "Materials")
            # print("link_path: ", link_path)
            target_material_dir = os.path.abspath(os.path.join(target_dir, "Materials"))
            # print("target_material_dir: ", target_material_dir)
            target_relative_path = os.path.relpath(target_material_dir, root)
            # print("target_relative_path: ", target_relative_path)
            os.symlink(target_relative_path, link_path)


src_dir = "/ssd/tianshihan/fixed/target/scenes_test/MWAX5JYKTKJZ2AABAAAAACA8_usd"
target_dir = "/ssd/tianshihan/export_scene_test"

export(src_dir, target_dir)


