from pxr import Usd
import json
import os


def mapping_model_id(old_stage_path, new_stage_path):
    old_stage = Usd.Stage.Open(old_stage_path)
    new_stage = Usd.Stage.Open(new_stage_path)

    old_meshes = old_stage.GetPrimAtPath("/Root/Meshes")
    new_meshes = new_stage.GetPrimAtPath("/Root/Meshes")

    # key: old_model, e.g. cabinet/SM_00_cabinet_0
    # value: new_model, e.g. cabinet/model_55f8352bfc7ed1f9c2b9ef5704918e6f_0
    mapping_model_result = dict()

    for old_scope, new_scope in zip(old_meshes.GetChildren(), new_meshes.GetChildren()):
        for old_cate, new_cate in zip(old_scope.GetChildren(), new_scope.GetChildren()):
            for old_model, new_model in zip(old_cate.GetChildren(), new_cate.GetChildren()):
                mapping_model_result[f"{str(old_cate.GetName())}/{str(old_model.GetName())}"] = f"{str(new_cate.GetName())}/{str(new_model.GetName())}"
    
    return mapping_model_result


def parse_into_json_file(result, json_file_path):
    with open(json_file_path, "w") as f:
        json.dump(result, f, indent=4)


old_scenes_parent_dir = "/ssd/tianshihan/fixed/home_scenes"
new_scenes_parent_dir = "/g0433_data/tianshihan/target_69_new/scenes"
scene_dirs = os.listdir(old_scenes_parent_dir)
result_parent_dir = "/ssd/tianshihan/test/mapping_model_id_results"
os.makedirs(result_parent_dir, exist_ok=True)

for scene in scene_dirs:
    old_stage_path = os.path.join(old_scenes_parent_dir, scene, "start_result_fix.usd")
    new_stage_path = os.path.join(new_scenes_parent_dir, scene, "start_result_fix.usd")
    if not os.path.exists(old_stage_path):
        old_stage_path = os.path.join(old_scenes_parent_dir, scene, "start_result_new.usd")
        new_stage_path = os.path.join(new_scenes_parent_dir, scene, "start_result_new.usd")
    
    mapping_model_result = mapping_model_id(old_stage_path, new_stage_path)
    result_file_path = os.path.join(result_parent_dir, f"{scene}.json")
    parse_into_json_file(mapping_model_result, result_file_path)