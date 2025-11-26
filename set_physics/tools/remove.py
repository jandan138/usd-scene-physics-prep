from pxr import Usd, UsdPhysics, PhysxSchema


def remove_collider_(prim):
    # --- normal collision api ---
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        prim.RemoveAPI(UsdPhysics.CollisionAPI)
    if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
    if prim.HasProperty("physics:collisionEnabled"):
        prim.RemoveProperty("physics:collisionEnabled")   
    if prim.HasProperty("physics:approximation"):
        prim.RemoveProperty("physics:approximation")

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
    if prim.HasProperty("physxConvexDecompositionCollision:hullVertexLimit"):
        prim.RemoveProperty("physxConvexDecompositionCollision:hullVertexLimit")
    if prim.HasProperty("physxConvexDecompositionCollision:maxConvexHulls"):
        prim.RemoveProperty("physxConvexDecompositionCollision:maxConvexHulls")
    # 2) convex hull
    if prim.HasAPI(PhysxSchema.PhysxConvexHullCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxConvexHullCollisionAPI)
    if prim.HasProperty("physxConvexHullCollision:hullVertexLimit"):
        prim.RemoveProperty("physxConvexHullCollision:hullVertexLimit")
    # 3) sdf mesh
    if prim.HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxSDFMeshCollisionAPI)
    if prim.HasProperty("physxSDFMeshCollision:sdfResolution"):
        prim.RemoveProperty("physxSDFMeshCollision:sdfResolution")
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