import os
import tqdm
from isaacsim import SimulationApp
CONFIG = {"sync_loads": True, "headless": True, "renderer": "RayTracedLighting"}
kit = SimulationApp(launch_config=CONFIG)

from pxr import Usd, UsdPhysics, UsdGeom, Gf, Sdf, PhysxSchema
from pxr_utils.usd_physics import copyfile


SDF_COLLISION_APPROXIMATION_NAME = "sdf"
CONVEX_HULL_COLLISION_APPROXIMATION_NAME = "convexHull"
CONVEX_DECOMPOSITION_COLLISION_APPROXIMATION_NAME = "convexDecomposition"
MESH_SIMPLIFICATION_COLLISION_APPROXIMATION_NAME = "meshSimplification"
TRIANGLE_MESH_COLLISION_APPROXIMATION_NAME = "none"


COLLISION_APPROXIMATION_OPTIONS_SET = [
    SDF_COLLISION_APPROXIMATION_NAME,
    CONVEX_HULL_COLLISION_APPROXIMATION_NAME,
    CONVEX_DECOMPOSITION_COLLISION_APPROXIMATION_NAME,
    MESH_SIMPLIFICATION_COLLISION_APPROXIMATION_NAME,
    TRIANGLE_MESH_COLLISION_APPROXIMATION_NAME
]


def get_all_joints(instance_prim):
    joint_list = []

    def recurse_prim(current_prim):
        for child in current_prim.GetChildren():
            if child.IsA(UsdPhysics.Joint):
                joint_list.append(UsdPhysics.Joint(child))
            recurse_prim(child)
    
    recurse_prim(instance_prim)

    return joint_list


def get_joint_connected_bodies(joint):
    body0_rel = joint.GetBody0Rel()
    body1_rel = joint.GetBody1Rel()

    body0_paths = body0_rel.GetTargets()
    body1_paths = body1_rel.GetTargets()

    return body0_paths + body1_paths


def get_leaf_meshes(item):
    leaf_meshes = set()
    
    def recurse_prim(current_prim):
        if current_prim.GetChildren():
            for child in current_prim.GetChildren():
                recurse_prim(child)
        else:
            if current_prim.GetTypeName() == "Mesh":
                leaf_meshes.add(current_prim.GetPath())

    recurse_prim(item)

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
        for mesh in includes:
            mesh_merge_collection.GetIncludesRel().AddTarget(mesh)
        for mesh in excludes:
            mesh_merge_collection.GetExcludesRel().AddTarget(mesh)


def set_collider_with_approx(prim, approx):
    if approx not in COLLISION_APPROXIMATION_OPTIONS_SET:
        raise TypeError(f"'{approx}' is not a valid collision approximation option or not supported in our design.")
    if prim.GetTypeName() in ["Xform", "Mesh"]:
        collider = UsdPhysics.CollisionAPI.Apply(prim)
        mesh_collider = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_collider.CreateApproximationAttr(approx)
        collider.GetCollisionEnabledAttr().Set(True)
        if approx == SDF_COLLISION_APPROXIMATION_NAME:
            physx_collider = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
            physx_collider.CreateSdfResolutionAttr().Set(256)
        elif approx == CONVEX_HULL_COLLISION_APPROXIMATION_NAME:
            physx_collider = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
            physx_collider.CreateHullVertexLimitAttr().Set(64)
        elif approx == CONVEX_DECOMPOSITION_COLLISION_APPROXIMATION_NAME:
            physx_collider = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
            physx_collider.CreateHullVertexLimitAttr().Set(64)
            physx_collider.CreateMaxConvexHullsAttr().Set(len(get_leaf_meshes(prim)))
        elif approx == MESH_SIMPLIFICATION_COLLISION_APPROXIMATION_NAME:
            physx_collider = PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI.Apply(prim)
        elif approx == TRIANGLE_MESH_COLLISION_APPROXIMATION_NAME:
            physx_collider = PhysxSchema.PhysxTriangleMeshCollisionAPI.Apply(prim)


def set_rigidbody(prim, init_state=True):
    if prim.GetTypeName() in ["Xform", "Mesh"]:
        rigidbody = UsdPhysics.RigidBodyAPI.Apply(prim)
        rigidbody.GetRigidBodyEnabledAttr().Set(init_state)

        # set xformOp Attribute for rigid body
        if not prim.HasAttribute("xformOp:transform"):
            prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray, custom=False).Set(["xformOp:transform"])
            transform_attr = prim.CreateAttribute("xformOp:transform", Sdf.ValueTypeNames.Matrix4d, custom=False)
            identity_matrix = Gf.Matrix4d(1.0)
            transform_attr.Set(identity_matrix)
            # xformOp:transform 单位矩阵初始化
            # identity_matrix = Gf.Matrix4d(1.0)
            prim.GetAttribute("xformOp:transform").Set(identity_matrix)

        transform_to_rt(prim)


# def set_center_of_mass_for_rigid(prim):
#     if prim.GetTypeName() in ["Xform", "Mesh"] and prim.HasAPI(UsdPhysics.RigidBodyAPI):
#         mass = UsdPhysics.MassAPI.Apply(prim)
#         mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(0, 0, 0))


def remove_collider_(prim):
    # --- normal collision api ---
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        prim.RemoveAPI(UsdPhysics.CollisionAPI)
    if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
    if prim.GetAttribute("physics:collisionEnabled"):
        prim.GetAttribute("physics:collisionEnabled").Clear()    
    if prim.GetAttribute("physics:approximation"):
        prim.GetAttribute("physics:approximation").Clear()

    # --- mesh merge collision api ---
    if prim.HasAPI(PhysxSchema.PhysxMeshMergeCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxMeshMergeCollisionAPI)
    collection_api = Usd.CollectionAPI.GetCollection(prim, "collection:collisionmeshes")
    if collection_api:
        collection_api.ResetCollection()

    # --- collision approx apis ---
    # 1) convex decomposition
    if prim.HasAPI(PhysxSchema.PhysxConvexDecompositionCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxConvexDecompositionCollisionAPI)
    if prim.GetAttribute("physxConvexDecompositionCollision:hullVertexLimit"):
        prim.GetAttribute("physxConvexDecompositionCollision:hullVertexLimit").Clear()
    if prim.GetAttribute("physxConvexDecompositionCollision:maxConvexHulls"):
        prim.GetAttribute("physxConvexDecompositionCollision:maxConvexHulls").Clear()
    # 2) convex hull
    if prim.HasAPI(PhysxSchema.PhysxConvexHullCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxConvexHullCollisionAPI)
    if prim.GetAttribute("physxConvexHullCollision:hullVertexLimit"):
        prim.GetAttribute("physxConvexHullCollision:hullVertexLimit").Clear()
    # 3) sdf mesh
    if prim.HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxSDFMeshCollisionAPI)
    if prim.GetAttribute("physxSDFMeshCollision:sdfResolution"):
        prim.GetAttribute("physxSDFMeshCollision:sdfResolution").Clear()
    # 4) mesh simplification
    if prim.HasAPI(PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI)
    # 5) triangle mesh
    if prim.HasAPI(PhysxSchema.PhysxTriangleMeshCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxTriangleMeshCollisionAPI)


def remove_collider(item):
    remove_collider_(item)
    for i in item.GetChildren():
        remove_collider(i)


def remove_rigid_(prim):
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        prim.RemoveAPI(UsdPhysics.RigidBodyAPI)

    if prim.IsA(UsdPhysics.Joint):
        prim.GetAttribute("physics:jointEnabled").Set(False)


def remove_rigid(item):
    remove_rigid_(item)
    for i in item.GetChildren():
        remove_rigid(i)


def bind_rigid_for_merged_mesh(prim, approx):
    set_rigidbody(prim)
    set_collider_with_approx(prim, approx)
    set_mesh_merge_collision(prim, includes=get_leaf_meshes(prim), excludes=[])


def bind_static_for_merged_mesh(prim, approx):
    set_collider_with_approx(prim, approx)
    set_mesh_merge_collision(prim, includes=get_leaf_meshes(prim), excludes=[])


def bind_articulation(instance_prim, pickable=False, approx=SDF_COLLISION_APPROXIMATION_NAME):
    children = instance_prim.GetChildren()

    joint_list = get_all_joints(instance_prim)
    # jointed_prims_set = set()
    # for joint in joint_list:
    #     jointed_prims_set.update(get_joint_connected_bodies(joint)) 

    # start binding
    for child in children:
        childName = str(child.GetName())

        if not childName.lower() in ['group_static', 'group_00'] and child.IsA(UsdGeom.Xform):
            bind_rigid_for_merged_mesh(child, approx)

        if childName.lower() == 'group_00':
            if pickable:
                bind_rigid_for_merged_mesh(child, approx)
            else:
                bind_static_for_merged_mesh(child, approx)
        
        if childName.lower() == 'group_static':
            bind_static_for_merged_mesh(child, approx)
    
    # enable joint after binding
    for joint in joint_list:
        joint.GetJointEnabledAttr().Set(True)


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
    # "bowl", # 碗
    # "light", # 灯
    "Musical_instrument", # 乐器
    "toy", # 玩具
    "keyboard", # 键盘
    # "plate", # 盘子
    "telephone", # 电话
    "cup", # 杯子
    # "picture", # 图片
    "tray", # 托盘
    "lamp",
    "box",
    "plant"
]


def solve_scene(scene_path, dest_scene_path):
    copyfile(scene_path, dest_scene_path)

    stage = Usd.Stage.Open(dest_scene_path)
    meshes = stage.GetPrimAtPath("/Root/Meshes")
    total_active_objects = 0
    articulated_objects = 0
    max_articulated_obj = 40
    max_active_objects = 40
    for scope in meshes.GetChildren():
        scope_name = str(scope.GetName()) # BaseAnimation, Animation, Base, Furniture
        for cate in scope.GetChildren():
            cate_name = str(cate.GetName()) # object category, such as 'oven', 'door'
            for instance in tqdm.tqdm(cate.GetChildren(), desc=f"Scope {scope_name} Category {cate_name}"): # object model
                remove_collider(instance)
                remove_rigid(instance)
                instance_prim = instance.GetPrimAtPath("Instance")
                if scope_name in ["BaseAnimation", "Animation"]:
                    pickable = cate_name in PICKABLE_OBJECTS
                    bind_articulation(instance_prim, pickable, approx=CONVEX_DECOMPOSITION_COLLISION_APPROXIMATION_NAME)
                    articulated_objects += 1
                elif scope_name == "Base":
                    bind_static_for_merged_mesh(instance_prim, approx=TRIANGLE_MESH_COLLISION_APPROXIMATION_NAME)
                else:
                    if cate_name in PICKABLE_OBJECTS:
                        # if cate_name in ["plate", "bowl"]:
                        #     bind_rigid_for_merged_mesh(instance_prim, approx=CONVEX_DECOMPOSITION_COLLISION_APPROXIMATION_NAME)
                        # else:
                        #     bind_rigid_for_merged_mesh(instance_prim, approx=SDF_COLLISION_APPROXIMATION_NAME)
                        bind_rigid_for_merged_mesh(instance_prim, approx=CONVEX_DECOMPOSITION_COLLISION_APPROXIMATION_NAME)
                        total_active_objects += 1
                    else:
                        bind_static_for_merged_mesh(instance_prim, approx=TRIANGLE_MESH_COLLISION_APPROXIMATION_NAME)

    stage.GetRootLayer().Save()
    return total_active_objects, articulated_objects

def test_solve_scene():
    scene_path = "/ssd/tianshihan/target_home_3/scenes/MV7J6NIKTKJZ2AABAAAAADQ8_usd/start_result_new.usd"
    dest_scene_path = "/ssd/tianshihan/target_home_3/scenes/MV7J6NIKTKJZ2AABAAAAADQ8_usd/start_result_dynamic_new.usd"
    total_active_objects, articulated_objects = solve_scene(scene_path, dest_scene_path)
    print(total_active_objects, "total_active_objects", articulated_objects, "articulated_objects")

def test_solve_all_scene():
    src_folder = "/ssd/tianshihan/target_69_new/scenes"
    scene_dirs = [_ for _ in os.listdir(src_folder) if os.path.isdir(os.path.join(src_folder, _))]
    for scene_dir in tqdm.tqdm(scene_dirs):
        scene_path = os.path.join(src_folder, scene_dir, "start_result_fix.usd")
        if not os.path.exists(scene_path):
            scene_path = os.path.join(src_folder, scene_dir, "start_result_new.usd")
        dest_scene_path = os.path.join(src_folder, scene_dir, "start_result_dynamic.usd")
        print(dest_scene_path)
        total_active_objects, articulated_objects = solve_scene(scene_path, dest_scene_path)
        print(total_active_objects, "total_active_objects", articulated_objects, "articulated_objects")

test_solve_scene()
# test_solve_all_scene()
