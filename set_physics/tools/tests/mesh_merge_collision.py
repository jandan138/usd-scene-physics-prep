# -----
# 我这里为了 import PhysxSchema，必须先初始化 isaac sim runtime（不太确定为啥）
# 也许其他人直接 source isaacsim setup_conda 那个也可以，或者直接用 isaac sim 的 python.sh 运行脚本？
from isaacsim import SimulationApp

CONFIG = {"sync_loads": True, "headless": True, "renderer": "RayTracedLighting"}

kit = SimulationApp(launch_config=CONFIG)

# -----

from pxr import Usd, UsdPhysics, UsdGeom, Gf, Sdf, PhysxSchema


SDF_COLLISION_APPROXIMATION_NAME = "sdf"
CONVEX_HULL_COLLISION_APPROXIMATION_NAME = "convexHull"
CONVEX_DECOMPOSITION_COLLISION_APPROXIMATION_NAME = "convexDecomposition"
MESH_SIMPLIFICATION_COLLISION_APPROXIMATION_NAME = "meshSimplification"

COLLISION_APPROXIMATION_OPTIONS_SET = [
    SDF_COLLISION_APPROXIMATION_NAME,
    CONVEX_HULL_COLLISION_APPROXIMATION_NAME,
    CONVEX_DECOMPOSITION_COLLISION_APPROXIMATION_NAME,
    MESH_SIMPLIFICATION_COLLISION_APPROXIMATION_NAME
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
        data_type = [Sdf.ValueTypeNames.Double3, Sdf.ValueTypeNames.Quatd, Sdf.ValueTypeNames.Double3]
        if "xformOp:transform" in order:
            tf_m = prim.GetAttribute("xformOp:transform").Get()
            tf = Gf.Transform()
            tf.SetMatrix(tf_m)

            translation = tf.GetTranslation()
            orientation = tf.GetRotation().GetQuat()
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
            physx_collider.CreateMaxConvexHullsAttr().Set(32)
        elif approx == MESH_SIMPLIFICATION_COLLISION_APPROXIMATION_NAME:
            physx_collider = PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI.Apply(prim)


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


# # 部分物体（例如烤箱这种门是垂直于地面的，可能会因为惯性的原因？，启动仿真时门会自动打开直至平行于地面，测试发现给它设一个质心可以避免这种现象，有时设 damping 也可以，但不确定是否存在更好的方案）
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
    # 4) triangle mesh simplification
    if prim.HasAPI(PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI)


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

# 示例使用
stage_path = '/ssd/tianshihan/test/Collected_0b6e34007296443cbdd10546f24db5eb_0/0b6e34007296443cbdd10546f24db5eb_0.usd'
stage = Usd.Stage.Open(stage_path)
instance_prim = stage.GetPrimAtPath("/World/instance/Instance")

# 处理之前先清理干净，避免冲突
remove_collider(instance_prim)
remove_rigid(instance_prim)
bind_articulation(instance_prim, pickable=False, approx=SDF_COLLISION_APPROXIMATION_NAME)

stage.GetRootLayer().Save()