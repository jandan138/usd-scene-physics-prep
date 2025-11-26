from isaacsim import SimulationApp
CONFIG = {"headless": True}
kit = SimulationApp(launch_config=CONFIG)
import os
import tqdm
import shutil
from pxr import Usd, UsdPhysics, PhysxSchema
from omni.isaac.core.utils.semantics import add_update_semantics


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
            physx_collider.CreateMinThicknessAttr().Set(0.0001)
        elif approx == CONVEX_DECOMPOSITION:
            physx_collider = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
            physx_collider.CreateHullVertexLimitAttr().Set(64)
            physx_collider.CreateMaxConvexHullsAttr().Set(256)
            physx_collider.CreateMinThicknessAttr().Set(0.0001)
            physx_collider.CreateVoxelResolutionAttr().Set(100000)
            physx_collider.CreateErrorPercentageAttr().Set(0.01)
            physx_collider.CreateShrinkWrapAttr().Set(True)
        elif approx == MESH_SIMPLIFICATION:
            physx_collider = PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI.Apply(prim)
        elif approx == TRIANGLE_MESH:
            physx_collider = PhysxSchema.PhysxTriangleMeshCollisionAPI.Apply(prim)


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


def bind_static_for_merged_mesh(prim, approx):
    set_collider_with_approx(prim, approx)
    set_mesh_merge_collision(prim, includes=get_leaf_meshes(prim), excludes=[])


def set_semantic_label(prim, label):
    if prim.GetTypeName() in ["Mesh", "Xform"]:
        add_update_semantics(prim, semantic_label=label, type_label="class")

    all_children = prim.GetAllChildren()
    for child in all_children:
        set_semantic_label(child, label)


def solve_scene(scene_path, dest_scene_path):
    copyfile(scene_path, dest_scene_path)

    stage = Usd.Stage.Open(dest_scene_path)
    meshes = stage.GetPrimAtPath("/Root/Meshes")

    for scope in meshes.GetChildren():
        scope_name = str(scope.GetName()) # BaseAnimation, Animation, Base, Furniture
        for cate in scope.GetChildren():
            cate_name = str(cate.GetName()) # object category, such as 'oven', 'door'
            for instance in tqdm.tqdm(cate.GetChildren(), desc=f"Scope {scope_name} Category {cate_name}"): # object model
                instname = str(instance.GetName())
                label = f"{cate_name}/{instname}"
                set_semantic_label(instance, label)
                if cate_name == "door":
                    # stage.RemovePrim(instance.GetPath())
                    instance.SetActive(False)
                else:
                    remove_collider(instance)
                    remove_rigid(instance)
                    instance_prim = instance.GetPrimAtPath("Instance")
                    joint_list = get_all_joints(instance_prim)
                    jointed_prims_set = set()
                    for joint in joint_list:
                        joint.GetJointEnabledAttr().Set(False)
                        jointed_prims_set.update(get_joint_connected_bodies(joint))
                    if jointed_prims_set:
                        for jointed_prim_path in jointed_prims_set:
                            jointed_prim = stage.GetPrimAtPath(jointed_prim_path)
                            bind_static_for_merged_mesh(jointed_prim, approx=TRIANGLE_MESH_COLLISION_APPROXIMATION_NAME)
                    else:
                        bind_static_for_merged_mesh(instance_prim, approx=TRIANGLE_MESH_COLLISION_APPROXIMATION_NAME)

    stage.GetRootLayer().Save()


scenes_list = [
    # "/ssd/tianshihan/fixed/target/scenes/MVUCSQAKTKJ5EAABAAAAAAY8_usd/start_result_fix.usd",
    # "/ssd/tianshihan/fixed/target/scenes/MVUCSQAKTKJ5EAABAAAAAAI8_usd/start_result_fix.usd",
    # "/ssd/tianshihan/fixed/target/scenes/MVUCSQAKTKJ5EAABAAAAAAA8_usd/start_result_fix.usd",
    # "/ssd/tianshihan/fixed/target/scenes/MV7J6NIKTKJZ2AABAAAAAEA8_usd/start_result_new.usd",
    # "/ssd/tianshihan/fixed/target/scenes/MV7J6NIKTKJZ2AABAAAAADA8_usd/start_result_new.usd",
    # "/ssd/tianshihan/fixed/target/scenes/MVUCSQAKTKJ5EAABAAAAAAQ8_usd/start_result_fix.usd",
    # "/ssd/tianshihan/fixed/target/scenes/MV7J6NIKTKJZ2AABAAAAADQ8_usd/start_result_new.usd",
    # "/ssd/tianshihan/fixed/target/scenes/MV7J6NIKTKJZ2AABAAAAAEI8_usd/start_result_new.usd",
    # "/ssd/tianshihan/fixed/target/scenes/MV7J6NIKTKJZ2AABAAAAADY8_usd/start_result_new.usd",
    # "/ssd/tianshihan/fixed/target/scenes/MV7J6NIKTKJZ2AABAAAAADI8_usd/start_result_new.usd"
    "/ssd/tianshihan/target_69_new_bk/scenes/MV7J6NIKTKJZ2AABAAAAADY8_usd/start_result_new.usd"
]


for scene_path in scenes_list:
    scene_dir = os.path.dirname(scene_path)
    dest_scene_path = os.path.join(scene_dir, "semantic_static_without_door.usd")
    print(f"Scene path: {dest_scene_path}")
    solve_scene(scene_path, dest_scene_path)