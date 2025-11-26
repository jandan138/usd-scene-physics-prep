from isaacsim import SimulationApp
CONFIG = {"headless": True}
kit = SimulationApp(launch_config=CONFIG)
import os
import tqdm
import shutil
import json
import argparse
from pxr import Usd, UsdPhysics, UsdGeom, Gf, Sdf, PhysxSchema
from omni.isaac.core.utils.semantics import add_update_semantics, get_semantics


SDF = "sdf"
CONVEX_HULL = "convexHull"
CONVEX_DECOMPOSITION = "convexDecomposition"
MESH_SIMPLIFICATION = "meshSimplification"
TRIANGLE_MESH = "none"


COLLISION_APPROXIMATION_OPTIONS = [
    SDF,
    CONVEX_HULL,
    CONVEX_DECOMPOSITION,
    MESH_SIMPLIFICATION,
    TRIANGLE_MESH
]


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


def get_all_joints(prim):
    joint_list = []

    def recurse_prim(current_prim):
        for child in current_prim.GetChildren():
            if child.IsA(UsdPhysics.Joint):
                joint_list.append(UsdPhysics.Joint(child))
            recurse_prim(child)
    
    recurse_prim(prim)

    return joint_list


def get_joint_connected_bodies(joint):
    body0_rel = joint.GetBody0Rel()
    body1_rel = joint.GetBody1Rel()

    body0_paths = body0_rel.GetTargets()
    body1_paths = body1_rel.GetTargets()

    return body0_paths + body1_paths


def get_leaf_meshes(prim):
    leaf_meshes = set()
    
    def recurse_prim(current_prim):
        if current_prim.GetChildren():
            for child in current_prim.GetChildren():
                recurse_prim(child)
        else:
            if current_prim.GetTypeName() == "Mesh":
                leaf_meshes.add(current_prim.GetPath())

    recurse_prim(prim)

    return leaf_meshes


def transform_to_rt(prim):
    if prim.HasAttribute("xformOpOrder"):
        order = prim.GetAttribute("xformOpOrder").Get()

        if order is None:
            return None

        new_order = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
        data_type = [Sdf.ValueTypeNames.Double3, Sdf.ValueTypeNames.Quatf, Sdf.ValueTypeNames.Double3]
        if "xformOp:transform" in order:
            tf_m = prim.GetAttribute("xformOp:transform").Get()
            tf = Gf.Transform()
            tf.SetMatrix(tf_m)

            translation = tf.GetTranslation()
            orientation = Gf.Quatf(tf.GetRotation().GetQuat())
            scale = tf.GetScale()
            data = [translation, orientation, scale]

            for term, dt, d in zip(new_order, data_type, data):
                attr = prim.CreateAttribute(term, dt, custom=False)
                attr.Set(d)

            prim.GetAttribute("xformOpOrder").Set(new_order)
    
    for child in prim.GetChildren():
        transform_to_rt(child)


def set_mesh_merge_collision(prim, includes, excludes):
    if prim.GetTypeName() in ["Xform", "Mesh"]:
        mesh_merge_collision = PhysxSchema.PhysxMeshMergeCollisionAPI.Apply(prim)
        mesh_merge_collection = mesh_merge_collision.GetCollisionMeshesCollectionAPI()
        mesh_merge_collection.GetIncludesRel().SetTargets(includes)
        mesh_merge_collection.GetExcludesRel().SetTargets(excludes)


def set_collider_with_approx(prim, approx):
    if approx not in COLLISION_APPROXIMATION_OPTIONS:
        raise TypeError(f"'{approx}' is not a valid collision approximation option or not supported in our design.")
    if prim.GetTypeName() in ["Xform", "Mesh"]:
        remove_collider(prim)
        collider = UsdPhysics.CollisionAPI.Apply(prim)
        mesh_collider = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_collider.CreateApproximationAttr().Set(approx)
        collider.GetCollisionEnabledAttr().Set(True)
        if approx == SDF:
            physx_collider = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
            physx_collider.CreateSdfResolutionAttr().Set(256)
        elif approx == CONVEX_HULL:
            physx_collider = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
            physx_collider.CreateHullVertexLimitAttr().Set(64)
            physx_collider.CreateMinThicknessAttr().Set(0.001)
        elif approx == CONVEX_DECOMPOSITION:
            physx_collider = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
            physx_collider.CreateHullVertexLimitAttr().Set(64)
            physx_collider.CreateMaxConvexHullsAttr().Set(256)
            physx_collider.CreateMinThicknessAttr().Set(0.001)
            physx_collider.CreateVoxelResolutionAttr().Set(50000)  # default value
            physx_collider.CreateErrorPercentageAttr().Set(0.011)
            physx_collider.CreateShrinkWrapAttr().Set(True)
        elif approx == MESH_SIMPLIFICATION:
            physx_collider = PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI.Apply(prim)
        elif approx == TRIANGLE_MESH:
            physx_collider = PhysxSchema.PhysxTriangleMeshCollisionAPI.Apply(prim)


def set_rigidbody(prim):
    if prim.GetTypeName() in ["Xform", "Mesh"]:
        remove_rigid(prim)
        rigidbody = UsdPhysics.RigidBodyAPI.Apply(prim)
        rigidbody.GetRigidBodyEnabledAttr().Set(True)

        # tranfer xformOpOrder to `translate, orient, scale` for rigid body
        if not prim.HasAttribute("xformOp:transform"):
            prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray, custom=False).Set(["xformOp:transform"])
            prim.CreateAttribute("xformOp:transform", Sdf.ValueTypeNames.Matrix4d, custom=False)
            identity_matrix = Gf.Matrix4d(1.0)
            prim.GetAttribute("xformOp:transform").Set(identity_matrix)
        
        transform_to_rt(prim)

        # Set the center of mass to prevent the dynamic part from falling down due to inertia
        if prim.GetAttribute("semantic:Semantics:params:semanticData").Get().split("/")[0] in ["oven", "dishwasher", "microwave"]:
            set_center_of_mass_for_rigid(prim)


def set_center_of_mass_for_rigid(prim):
    if prim.GetTypeName() in ["Xform", "Mesh"] and prim.HasAPI(UsdPhysics.RigidBodyAPI):
        mass = UsdPhysics.MassAPI.Apply(prim)
        mass.CreateCenterOfMassAttr().Set(prim.GetAttribute("xformOp:translate").Get())


def remove_collider_(prim):
    # --- normal collision api ---
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        prim.RemoveAPI(UsdPhysics.CollisionAPI)
    if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
    if prim.GetProperty("physics:collisionEnabled"):
        prim.RemoveProperty("physics:collisionEnabled") 
    if prim.GetProperty("physics:approximation"):
        prim.RemoveProperty("physics:approximation")

    # --- mesh merge collision api ---
    if prim.HasAPI(PhysxSchema.PhysxMeshMergeCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxMeshMergeCollisionAPI)
    if prim.GetProperty("collection:collisionmeshes"):
        prim.RemoveProperty("collection:collisionmeshes")
    if prim.GetProperty("collection:collisionmeshes:includes"):
        prim.RemoveProperty("collection:collisionmeshes:includes")
    if prim.GetProperty("collection:collisionmeshes:excludes"):
        prim.RemoveProperty("collection:collisionmeshes:excludes")

    # --- collision approx apis ---
    # 1) convex decomposition
    if prim.HasAPI(PhysxSchema.PhysxConvexDecompositionCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxConvexDecompositionCollisionAPI)
    if prim.GetProperty("physxConvexDecompositionCollision:hullVertexLimit"):
        prim.RemoveProperty("physxConvexDecompositionCollision:hullVertexLimit")
    if prim.GetProperty("physxConvexDecompositionCollision:maxConvexHulls"):
        prim.RemoveProperty("physxConvexDecompositionCollision:maxConvexHulls")
    if prim.GetProperty("physxConvexDecompositionCollision:errorPercentage"):
        prim.RemoveProperty("physxConvexDecompositionCollision:errorPercentage")
    if prim.GetProperty("physxConvexDecompositionCollision:minThickness"):
        prim.RemoveProperty("physxConvexDecompositionCollision:minThickness")
    if prim.GetProperty("physxConvexDecompositionCollision:shrinkWrap"):
        prim.RemoveProperty("physxConvexDecompositionCollision:shrinkWrap")
    if prim.GetProperty("physxConvexDecompositionCollision:voxelResolution"):
        prim.RemoveProperty("physxConvexDecompositionCollision:voxelResolution")
    # 2) convex hull
    if prim.HasAPI(PhysxSchema.PhysxConvexHullCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxConvexHullCollisionAPI)
    if prim.GetProperty("physxConvexHullCollision:hullVertexLimit"):
        prim.RemoveProperty("physxConvexHullCollision:hullVertexLimit")
    if prim.GetProperty("physxConvexHullCollision:minThickness"):
        prim.RemoveProperty("physxConvexHullCollision:minThickness")
    # 3) sdf mesh
    if prim.HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxSDFMeshCollisionAPI)
    if prim.GetProperty("physxSDFMeshCollision:sdfResolution"):
        prim.RemoveProperty("physxSDFMeshCollision:sdfResolution")
    # 4) mesh simplification
    if prim.HasAPI(PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI)
    # 5) triangle mesh
    if prim.HasAPI(PhysxSchema.PhysxTriangleMeshCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxTriangleMeshCollisionAPI)


def remove_collider(prim):
    remove_collider_(prim)
    for child in prim.GetChildren():
        remove_collider(child)


def remove_rigid_(prim):
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        prim.RemoveAPI(UsdPhysics.RigidBodyAPI)

    if prim.IsA(UsdPhysics.Joint):
        prim.GetAttribute("physics:jointEnabled").Set(False)


def remove_rigid(prim):
    remove_rigid_(prim)
    for child in prim.GetChildren():
        remove_rigid(child)


def bind_rigid_for_merged_mesh(prim, approx):
    set_rigidbody(prim)
    set_collider_with_approx(prim, approx)
    set_mesh_merge_collision(prim, includes=get_leaf_meshes(prim), excludes=[])


def bind_static_for_merged_mesh(prim, approx):
    set_collider_with_approx(prim, approx)
    set_mesh_merge_collision(prim, includes=get_leaf_meshes(prim), excludes=[])


def bind_articulation(stage, prim, pickable=False, approx=CONVEX_DECOMPOSITION):
    dynamic_collision_group_path_list =[]

    for child in prim.GetChildren():
        childName = child.GetName()

        if not childName.lower() in ['group_static', 'group_00'] and child.IsA(UsdGeom.Xform):
            bind_rigid_for_merged_mesh(child, approx)
            dynamic_collision_group = UsdPhysics.CollisionGroup.Define(stage, f"{str(prim.GetPath())}/Collisiion_{childName}")
            dynamic_includes_rel = dynamic_collision_group.GetCollidersCollectionAPI().CreateIncludesRel()
            dynamic_includes_rel.AddTarget(child.GetPath())
            dynamic_collision_group_path_list.append(dynamic_collision_group.GetPath())

        if childName.lower() == 'group_00':
            if pickable:
                bind_rigid_for_merged_mesh(child, approx)
            else:
                bind_static_for_merged_mesh(child, approx=TRIANGLE_MESH)
        
        if childName.lower() == 'group_static':
            bind_static_for_merged_mesh(child, approx=TRIANGLE_MESH)
    
    for joint in get_all_joints(prim):
        joint.GetJointEnabledAttr().Set(True)

    for dynamic_collision_group_path in dynamic_collision_group_path_list:
        filtered_groups = [_ for _ in dynamic_collision_group_path_list if _ != dynamic_collision_group_path]
        dynamic_collision_group = UsdPhysics.CollisionGroup.Get(stage, dynamic_collision_group_path)
        dynamic_collision_group.CreateFilteredGroupsRel().SetTargets(filtered_groups)

    return dynamic_collision_group_path_list


def set_semantic_label(prim, label):
    if prim.GetTypeName() in ["Mesh", "Xform"]:
        add_update_semantics(prim, semantic_label=label, type_label="class")

    for child in prim.GetAllChildren():
        set_semantic_label(child, label)


PICKABLE_OBJECTS = [
    # "backpack", # 背包
    "basket", # 篮子
    "pan", # 平底锅
    "pot", # 锅
    "bottle", # 瓶子
    # "clock", # 钟
    "laptop", # 笔记本电脑
    "mouse", # 鼠标
    # "pen", # 笔
    # "pillow", # 枕头
    # "towel", # 毛巾
    # "blanket", # 毯子
    "bowl", # 碗
    # "light", # 灯
    "Musical_instrument", # 乐器
    "toy", # 玩具
    "keyboard", # 键盘
    "plate", # 盘子
    "telephone", # 电话
    "cup", # 杯子
    # "picture", # 图片
    "tray", # 托盘
    "lamp", # 台灯
    "box", # 盒子
    "plant" # 盆栽
]


def process_scene_for_interaction(scene_path, dest_scene_path, logging_interactive_obj=False):
    copy_file(scene_path, dest_scene_path)

    stage = Usd.Stage.Open(dest_scene_path)
    meshes = stage.GetPrimAtPath("/Root/Meshes")
    pickable_object_list = []
    articulated_object_list = []

    base_collision_group_targets = []
    dynamic_collision_group_path_list = []
    for scope in meshes.GetChildren():
        scope_name = scope.GetName() # BaseAnimation, Animation, Base, Furniture
        for cate in scope.GetChildren():
            cate_name = cate.GetName() # object category, such as 'oven', 'door'
            for model_prim in tqdm.tqdm(cate.GetChildren(), desc=f"Scope {scope_name} Category {cate_name}"): # model_xxxx
                label = f"{cate_name}/{model_prim.GetName()}"
                set_semantic_label(model_prim, label)
                instance_prim = model_prim.GetPrimAtPath("Instance")
                if scope_name in ["BaseAnimation", "Animation"]:
                    pickable = cate_name in PICKABLE_OBJECTS
                    dynamic_colliison_group_paths = bind_articulation(stage, instance_prim, pickable, approx=CONVEX_DECOMPOSITION)
                    dynamic_collision_group_path_list.extend(dynamic_colliison_group_paths)
                    articulated_object_list.append(str(model_prim.GetPath()))
                    if pickable:
                        pickable_object_list.append(str(model_prim.GetPath()))
                elif scope_name == "Base":
                    bind_static_for_merged_mesh(instance_prim, approx=TRIANGLE_MESH)
                    if cate_name != "ground":
                        base_collision_group_targets.append(model_prim.GetPath())
                else:
                    if cate_name in PICKABLE_OBJECTS:
                        bind_rigid_for_merged_mesh(instance_prim, approx=CONVEX_DECOMPOSITION)
                        pickable_object_list.append(str(model_prim.GetPath()))
                    else:
                        bind_static_for_merged_mesh(instance_prim, approx=TRIANGLE_MESH)

    base_collision_group = UsdPhysics.CollisionGroup.Define(stage, "/Root/Meshes/Base/Collision_Group_base")
    base_includes_rel = base_collision_group.GetCollidersCollectionAPI().CreateIncludesRel()
    base_includes_rel.SetTargets(base_collision_group_targets)
    for dynamic_collision_group_path in dynamic_collision_group_path_list:
        dynamic_collision_group = UsdPhysics.CollisionGroup.Get(stage, dynamic_collision_group_path)
        dynamic_collision_group.GetFilteredGroupsRel().AddTarget(base_collision_group.GetPath())

    stage.GetRootLayer().Save()

    if logging_interactive_obj:
        with open(os.path.join(os.path.dirname(dest_scene_path), "interactive_obj_list.json"), "w") as f:
            total_objects = {"articulated": articulated_object_list,
                             "pickable": pickable_object_list}
            json.dump(total_objects, f, indent=4)

    return len(pickable_object_list), len(articulated_object_list)


def process_scene_for_navigation(scene_path, dest_scene_path):
    copy_file(scene_path, dest_scene_path)

    stage = Usd.Stage.Open(dest_scene_path)
    meshes = stage.GetPrimAtPath("/Root/Meshes")

    for scope in meshes.GetChildren():
        scope_name = scope.GetName() # BaseAnimation, Animation, Base, Furniture
        for cate in scope.GetChildren():
            cate_name = cate.GetName() # object category, such as 'oven', 'door'
            for model_prim in tqdm.tqdm(cate.GetChildren(), desc=f"Scope {scope_name} Category {cate_name}"): # model_xxxx
                label = f"{cate_name}/{model_prim.GetName()}"
                set_semantic_label(model_prim, label)
                instance_prim = model_prim.GetPrimAtPath("Instance")
                if cate_name == "door":
                    with open("./main_door_prim_path_new.json", "r") as f:
                        main_door_path_dict = json.load(f)
                    if not str(model_prim.GetPath()) in main_door_path_dict.values():
                        # stage.RemovePrim(model_prim.GetPath())
                        model_prim.SetActive(False)
                    else:
                        bind_static_for_merged_mesh(instance_prim, approx=TRIANGLE_MESH)
                else:
                    joint_list = get_all_joints(instance_prim)
                    jointed_prims_set = set()
                    for joint in joint_list:
                        joint.GetJointEnabledAttr().Set(False)
                        jointed_prims_set.update(get_joint_connected_bodies(joint))
                    if jointed_prims_set:
                        for jointed_prim_path in jointed_prims_set:
                            jointed_prim = stage.GetPrimAtPath(jointed_prim_path)
                            bind_static_for_merged_mesh(jointed_prim, approx=TRIANGLE_MESH)
                    else:
                        bind_static_for_merged_mesh(instance_prim, approx=TRIANGLE_MESH)

    stage.GetRootLayer().Save()


def test_scene_for_interaction():
    scene_path = "/ssd/tianshihan/target_69_new/scenes/MVUCSQAKTKJ5EAABAAAAAAY8_usd/start_result_fix.usd"
    dest_scene_path = "/ssd/tianshihan/target_69_new/scenes/MVUCSQAKTKJ5EAABAAAAAAY8_usd/start_result_dynamic.usd"
    pickable_object_num, articulated_object_num = process_scene_for_interaction(scene_path, dest_scene_path)
    print((f"{pickable_object_num} pickable objects\n"
           f"{articulated_object_num} articulated objects"))


def test_scene_for_navigation():
    scene_path = "/ssd/tianshihan/target_69_new/scenes/MV7J6NIKTKJZ2AABAAAAADA8_usd/start_result_new.usd"
    dest_scene_path = "/ssd/tianshihan/target_69_new/scenes/MV7J6NIKTKJZ2AABAAAAADA8_usd/start_result_navigation.usd"
    process_scene_for_navigation(scene_path, dest_scene_path)


def test_all_scenes_for_interaction():
    src_folder = "/ssd/tianshihan/target_69_new/scenes"
    scene_dirs = [_ for _ in os.listdir(src_folder) if os.path.isdir(os.path.join(src_folder, _))]
    for scene_dir in tqdm.tqdm(scene_dirs):
        scene_path = os.path.join(src_folder, scene_dir, "start_result_fix.usd")
        if not os.path.exists(scene_path):
            scene_path = os.path.join(src_folder, scene_dir, "start_result_new.usd")
        dest_scene_path = os.path.join(src_folder, scene_dir, "start_result_dynamic.usd")
        print(dest_scene_path)
        pickable_object_num, articulated_object_num = process_scene_for_interaction(scene_path, dest_scene_path)
        print((f"{pickable_object_num} pickable objects\n"
            f"{articulated_object_num} articulated objects"))


def test_all_scenes_for_navigation():
    src_folder = "/ssd/tianshihan/target_69_new/scenes"
    scene_dirs = [_ for _ in os.listdir(src_folder) if os.path.isdir(os.path.join(src_folder, _))]
    for scene_dir in tqdm.tqdm(scene_dirs):
        scene_path = os.path.join(src_folder, scene_dir, "start_result_fix.usd")
        if not os.path.exists(scene_path):
            scene_path = os.path.join(src_folder, scene_dir, "start_result_new.usd")
        dest_scene_path = os.path.join(src_folder, scene_dir, "start_result_navigation.usd")
        print(dest_scene_path)
        process_scene_for_navigation(scene_path, dest_scene_path)


# parser = argparse.ArgumentParser(description="Preprocess scenes such as binding physics rigid bodies and colliders.")
# parser.add_argument("-i", "--interaction", required=True, help="preprocess scene for interaction")
# parser.add_argument("-n", "--navigation", required=True, help="preprocess scene for navigation")
# parser.add_argument("-n", "--navigation", required=True, nargs="*", help="the scene names")

# args = parser.parse_args()

test_scene_for_interaction()
# test_all_scenes_for_interaction()
# test_scene_for_navigation()
# test_all_scenes_for_navigation()