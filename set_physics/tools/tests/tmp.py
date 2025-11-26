from isaacsim import SimulationApp

CONFIG = {"sync_loads": True, "headless": True, "renderer": "RayTracedLighting"}

kit = SimulationApp(launch_config=CONFIG)

from pxr import Usd, UsdPhysics, PhysxSchema


scene_path = "/ssd/tianshihan/fixed/target/scenes/MV7J6NIKTKJZ2AABAAAAADA8_usd/fridge.usd"
stage = Usd.Stage.Open(scene_path)
prim = stage.GetPrimAtPath("/Root/model_9de6a2262b9cbe3bd4f15ba484a7d035_0/Instance/Group_438")

collider = UsdPhysics.CollisionAPI.Apply(prim)
mesh_collider = UsdPhysics.MeshCollisionAPI.Apply(prim)
mesh_collider.CreateApproximationAttr("none")
collider.GetCollisionEnabledAttr().Set(True)
physx_collider = PhysxSchema.PhysxTriangleMeshCollisionAPI.Apply(prim)

stage.GetRootLayer().Save()
