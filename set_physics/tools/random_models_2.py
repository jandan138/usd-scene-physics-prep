from isaacsim import SimulationApp
CONFIG = {"headless": True, "renderer": "RayTracedLighting"}
kit = SimulationApp(launch_config=CONFIG)

import os
import random
import re
import uuid
import matplotlib.pyplot as plt
import numpy as np
from pxr import Usd, Gf, Sdf, UsdGeom, UsdShade
import omni.usd
from omni.kit.viewport.utility import get_active_viewport_window, create_viewport_window
import omni.isaac.sensor


# BASE_MATERIALS = ["OmniUe4Base.mdl", 
#                   "OmniUe4Function.mdl", 
#                   "OmniUe4Translucent.mdl", 
#                   "WorldGridMaterial.mdl",
#                   "DayMaterial.mdl",
#                   "KooPbr_maps.mdl",
#                   "KooPbr.mdl"]


def get_candidate_models(models_dir, model_semantic_name):
    """
    Inputs:
    - models_dir(absolute path): the directory of model assets, e.g. /ssd/tianshihan/target/models
    - model_semantic_name: the semantic name, e.g. teatable
    
    Return:
    - candidate_model_list(relative paths): a list of candidate models
    """
    candidate_model_list = []
    for root, dirs, files in os.walk(models_dir):
        for file in files:
            if file == "instance.usd" and model_semantic_name in root:
                candidate_model_list.append(os.path.relpath(os.path.join(root, file), 
                                                            os.path.dirname(models_dir)))
    
    return candidate_model_list


def get_candidate_materials_in_stage(stage):
    candidate_materials_list = []
    for prim in stage.Traverse():
        if prim.IsA(UsdShade.Material):
            candidate_materials_list.append(prim.GetPath())
    
    return candidate_materials_list


def get_candidate_materials(materials_dir, model_semantic_name):
    """
    Inputs:
    - materials_dir(absolute path): the directory of material assets, e.g. /ssd/tianshihan/material_assets_for_69
    - model_semantic_name: the semantic name, e.g. teatable
    
    Return:
    - candidate_material_list(absolute paths): a list of candidate materials
    """
    candidate_material_dir = os.path.join(materials_dir, model_semantic_name)
    candidate_material_list = [os.path.join(candidate_material_dir, f) for f in os.listdir(candidate_material_dir)]
    
    return candidate_material_list


def update_model_reference(model_prim, new_model_ref):
    model_prim.GetReferences().ClearReferences()
    model_prim.GetReferences().AddReference(new_model_ref)


def find_export_material_name(mdl_file_path):
    with open(mdl_file_path, "r") as file:
        lines = file.readlines()
    pattern = re.compile(r"export\s+material\s+([\w_]+)\(")

    for line in lines:
        match = re.search(pattern, line)
        if match:
            return match.group(1)

"""
# # 1. Define new material and rebind to prim
# # since there are some differences of texture png file paths between the origin and the new mdls
# # sometimes it will report texture referenced error, so do not define a new material for now
# # (this could be fixed in later dataset version)
# origin_binding_material_prim_path = child.GetRelationship("material:binding").GetTargets()[0]
# origin_binding_material_prim = stage.GetPrimAtPath(origin_binding_material_prim_path)
# origin_shader = origin_binding_material_prim.GetChildren()[0]
# origin_mdl = origin_shader.GetAttribute("info:mdl:sourceAsset").Get().path
# origin_mdl_identifier = origin_shader.GetAttribute("info:mdl:sourceAsset:subIdentifier").Get()
# if origin_mdl_identifier not in [mdl.removesuffix(".mdl") for mdl in BASE_MATERIALS]:
#     mdl_files = [file for file in os.listdir(materials_dir) if file.endswith(".mdl") and file not in origin_mdl and file not in BASE_MATERIALS]
#     random_new_mdl = random.choice(mdl_files)
#     new_mdl_identifier = find_export_material_name(os.path.join(materials_dir, random_new_mdl))
#     new_material_prim_path = str(origin_binding_material_prim.GetParent().GetPath()) + "/" + new_mdl_identifier + "_new"
#     new_material_prim = UsdShade.Material.Define(stage, new_material_prim_path)
#     new_shader = UsdShade.Shader.Define(stage, new_material_prim_path + "/" + new_mdl_identifier)
#     new_shader.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
#     new_shader.SetSourceAsset("./Materials/" + random_new_mdl, "mdl")
#     new_shader.SetSourceAssetSubIdentifier(new_mdl_identifier, "mdl")
#     new_material_prim.CreateSurfaceOutput("mdl").ConnectToSource(new_shader.ConnectableAPI(), "out")
#     new_material_prim.CreateDisplacementOutput("mdl").ConnectToSource(new_shader.ConnectableAPI(), "out")
#     new_material_prim.CreateVolumeOutput("mdl").ConnectToSource(new_shader.ConnectableAPI(), "out")
#     child.GetRelationship("material:binding").RemoveTarget(origin_binding_material_prim_path)
#     child.GetRelationship("material:binding").AddTarget(new_material_prim_path)
#     # stage.RemovePrim(origin_binding_material_prim_path)
#     origin_binding_material_prim.SetActive(False)

# # 2. Change the material binding reference to another material which defined already in this stage
# origin_binding_material_prim_path = child.GetRelationship("material:binding").GetTargets()[0]
# new_material_prim_path = random.choice([material for material in candidate_materials if str(material) != origin_binding_material_prim_path])
# child.GetRelationship("material:binding").RemoveTarget(origin_binding_material_prim_path)
# child.GetRelationship("material:binding").AddTarget(new_material_prim_path)
"""
def update_model_material(model_prim, candidate_materials):
    for child in model_prim.GetChildren():
        if child.HasRelationship("material:binding"):
            material_stage_candidates = []
            while len(material_stage_candidates) == 0:
                new_material_usd_path = random.choice(candidate_materials)
                material_stage = Usd.Stage.Open(new_material_usd_path)
                material_stage_candidates = get_candidate_materials_in_stage(material_stage)
            material_stage_prim_path = random.choice(material_stage_candidates)
            stage = omni.usd.get_context().get_stage()
            new_material_prim_path = f"/Root/Looks/_{uuid.uuid4().hex}_{str(child.GetName())}"
            new_material_prim = stage.DefinePrim(new_material_prim_path, "Material")
            new_material_ref = Sdf.Reference(assetPath=new_material_usd_path, primPath=material_stage_prim_path)
            new_material_prim.GetReferences().AddReference(new_material_ref)
            child.GetRelationship("material:binding").SetTargets([new_material_prim.GetPath()])

        update_model_material(child, candidate_materials)


def random_model_pipeline_case(stage, test_model_semantic, candidate_model_list):
    for prim in stage.Traverse():
        if str(prim.GetName()) == test_model_semantic:
            for model_prim in prim.GetChildren():
                update_model_reference(model_prim, random.choice(candidate_model_list))
                # update_model_material()


def random_material_pipeline_case(stage, test_semantic, candidate_materials):
    for prim in stage.Traverse():
        if str(prim.GetName()) == test_semantic:
            for model_prim in prim.GetChildren():
                update_model_material(model_prim, candidate_materials)


def camera_zoom_in(camera_prim, focal_length_diff=1):
    origin_focal_length = camera_prim.GetFocalLengthAttr().Get()
    camera_prim.GetFocalLengthAttr().Set(origin_focal_length + focal_length_diff)


def camera_zoom_out(camera_prim, focal_length_diff=1):
    origin_focal_length = camera_prim.GetFocalLengthAttr().Get()
    camera_prim.GetFocalLengthAttr().Set(origin_focal_length - focal_length_diff)


def camera_rotate(camera: omni.isaac.sensor.Camera, angle_x=0, angle_y=0, angle_z=0):
    orient_attr = camera.prim.GetAttribute("xformOp:orient")

    angle_x_rad = np.radians(angle_x)
    angle_y_rad = np.radians(angle_y)
    angle_z_rad = np.radians(angle_z)

    quat_x = Gf.Quatd(np.cos(angle_x_rad / 2), np.sin(angle_x_rad / 2), 0, 0)
    quat_y = Gf.Quatd(np.cos(angle_y_rad / 2), 0, np.sin(angle_y_rad / 2), 0)
    quat_z = Gf.Quatd(np.cos(angle_z_rad / 2), 0, 0, np.sin(angle_z_rad / 2))

    # merge rotation (order: Z -> Y -> X)
    orient_final_quad = quat_z * quat_y * quat_x
    orient_attr.Set(orient_final_quad)


def camera_translate(camera: omni.isaac.sensor.Camera, trans_x=0, trans_y=0, trans_z=0):
    translate_attr = camera.prim.GetAttribute("xformOp:translate")
    translate_attr.Set(Gf.Vec3d(trans_x, trans_y, trans_z))


def get_camera_rgba(camera: omni.isaac.sensor.Camera, save_path: str):
    if not os.path.exists(os.path.dirname(save_path)):
        os.makedirs(os.path.dirname(save_path), exist_ok=True)

    rgba_data = camera.get_rgba()
    plt.imsave(save_path, rgba_data)


# --- load stage ---
# stage_path = "/ssd/tianshihan/target_69_new_bk/scenes/MWAX5JYKTKJZ2AABAAAAACA8_usd/start_result_new.usd"
stage_path = "/ssd/tianshihan/target_69_new/scenes/MV7J6NIKTKJZ2AABAAAAADA8_usd/start_result_new_bk.usd"
omni.usd.get_context().open_stage(stage_path)
stage = omni.usd.get_context().get_stage()
print("Loading stage...")
# kit._wait_for_viewport()
for _ in range(100):
    kit.update()
print("Loading done!")

# --- create camera ---
camera_path = "/Root/Camera"
camera_prim = UsdGeom.Camera.Define(stage, camera_path)
# 设置相机属性
camera_prim.GetPrim().GetAttribute("focalLength").Set(15)  # 焦距
camera_prim.GetPrim().GetAttribute("horizontalAperture").Set(20.955)  # 水平孔径
camera_prim.GetPrim().GetAttribute("clippingRange").Set((0.1, 1000000.0))  # 裁剪范围

# 设置相机的位置和旋转
camera_origin_translate = Gf.Vec3d(750, 0, 120)
camera_origin_rotate = Gf.Vec3f(90, 0, -120)
xform_api = UsdGeom.XformCommonAPI.Get(stage, camera_path)
xform_api.SetTranslate(camera_origin_translate, Usd.TimeCode.Default())
xform_api.SetRotate(camera_origin_rotate, UsdGeom.XformCommonAPI.RotationOrderYXZ, Usd.TimeCode.Default())

viewport_window = get_active_viewport_window()
viewport_window.viewport_api.camera_path = camera_path
# viewport_window = create_viewport_window(name="test", camera_path=camera_path)
for _ in range(10):
    kit.update()
# NOTE: camera.get_rgba() need to play first!!!
omni.timeline.get_timeline_interface().play()

camera = omni.isaac.sensor.Camera(prim_path=camera_path,
                                  resolution=(1920, 1080))
camera.initialize()


# --- initialize input data ---
random_model_semantics = ["couch", "teatable", "tv", "refrigerator", "pan", "microwave", "electriccooker"]
models_dir = "/ssd/tianshihan/target_69_new/models"

candidate_models = dict()
for semantic in random_model_semantics:
    candidate_models[semantic] = get_candidate_models(models_dir, semantic)

random_material_semantics = ["blanket", "chair", "door", "wall", "table"]
materials_dir = "/ssd/tianshihan/target_69_new_bk/instance_material"

candidate_materials = dict()
for semantic in random_material_semantics:
    candidate_materials[semantic] = get_candidate_materials(materials_dir, semantic)

random_times = 15
rgba_id = 0

camera_trans_x = camera_origin_translate[0]
camera_trans_y = camera_origin_translate[1]
camera_trans_z = camera_origin_translate[2]

camera_angle_x = camera_origin_rotate[0]
camera_angle_y = camera_origin_rotate[1]
camera_angle_z = camera_origin_rotate[2]

for _ in range(random_times):

    for semantic in random_model_semantics:
        random_model_pipeline_case(stage, semantic, candidate_models[semantic])

    for semantic in random_material_semantics:
        random_material_pipeline_case(stage, semantic, candidate_materials[semantic])

    for i in range(370):
        kit.update()

        if i >= 10:
            get_camera_rgba(camera, f"/ssd/tianshihan/test/rgba_data/scene_2_final/data_{rgba_id}.png")
            rgba_id += 1

            camera_angle_z -= 0.05
            camera_rotate(camera, 
                          angle_x=camera_angle_x, 
                          angle_y=camera_angle_y, 
                          angle_z=camera_angle_z)


omni.timeline.get_timeline_interface().stop()
kit.close()