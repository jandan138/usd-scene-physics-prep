import os
import sys

# Add project root to python path to allow importing set_physics
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.append(project_root)

from pxr import Usd, UsdGeom, UsdPhysics
from set_physics.pxr_utils import usd_physics

INPUT_PATH = "/shared/smartbot/zzh/my_dev/usd-selfpack/output/task9_interaction_dynamic_v6_pack_glb_fix_20251229_v2/scene_interaction_dynamic_v6.usd"
OUTPUT_PATH = "/shared/smartbot/zzh/my_dev/usd-selfpack/output/task9_interaction_dynamic_v6_pack_glb_fix_20251229_v2/scene_interaction_dynamic_v6_separated.usd"

TARGET_PARENT = "/root/obj__02"
TARGET_CHILDREN = [
    "/root/obj__02/geometry_0/geometry_0",
    "/root/obj__02/geometry_0/geometry_01"
]

def main():
    if not os.path.exists(INPUT_PATH):
        print(f"Error: Input file not found: {INPUT_PATH}")
        return

    print(f"Opening stage: {INPUT_PATH}")
    stage = Usd.Stage.Open(INPUT_PATH)
    if not stage:
        print("Error: Failed to open stage")
        return

    # 1. Handle Parent: Remove RigidBodyAPI if present
    parent_prim = stage.GetPrimAtPath(TARGET_PARENT)
    if parent_prim.IsValid():
        if parent_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            print(f"Removing RigidBodyAPI from parent: {TARGET_PARENT}")
            parent_prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            # Also remove MassAPI if present, as it might confuse things if left on a non-rigid body
            if parent_prim.HasAPI(UsdPhysics.MassAPI):
                 parent_prim.RemoveAPI(UsdPhysics.MassAPI)
        else:
            print(f"Parent {TARGET_PARENT} does not have RigidBodyAPI (skipping removal)")
    else:
        print(f"Warning: Parent prim {TARGET_PARENT} not found!")

    # 2. Handle Children: Add RigidBodyAPI and Colliders
    for child_path in TARGET_CHILDREN:
        child_prim = stage.GetPrimAtPath(child_path)
        if not child_prim.IsValid():
            print(f"Error: Child prim {child_path} not found")
            continue

        print(f"Processing child: {child_path}")
        
        # Apply RigidBodyAPI
        print(f"  - Applying RigidBodyAPI")
        usd_physics.set_rigidbody(child_prim, init_state=True)
        
        # Apply CollisionAPI (ConvexDecomposition)
        print(f"  - Applying CollisionAPI (ConvexDecomposition)")
        usd_physics.set_collider(child_prim, approx=UsdPhysics.Tokens.convexDecomposition, init_state=True)

    # 3. Save
    print(f"Saving modified stage to: {OUTPUT_PATH}")
    stage.GetRootLayer().Export(OUTPUT_PATH)
    print("Done.")

if __name__ == "__main__":
    main()
