"""
Just for test. Because removing targets by hand in UI is tooo slow.
"""
from isaacsim import SimulationApp
CONFIG = {"sync_loads": True, "headless": True, "renderer": "RayTracedLighting"}
kit = SimulationApp(launch_config=CONFIG)

from pxr import Usd, PhysxSchema

stage_path = "/ssd/tianshihan/target_69_new/scenes/MV7J6NIKTKJZ2AABAAAAADA8_usd/start_result_dynamic.usd"
prim_477_path = "/Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0/Instance/Group_477"
component_477_path = "/Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0/Instance/Group_477/Component_8"
prim_520_path = "/Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0/Instance/Group_520"
component_520_path = "/Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0/Instance/Group_520/Component_25"

stage = Usd.Stage.Open(stage_path)
prim_477 = stage.GetPrimAtPath(prim_477_path)
prim_520 = stage.GetPrimAtPath(prim_520_path)


mesh_merge_collision_477 = PhysxSchema.PhysxMeshMergeCollisionAPI.Get(stage, prim_477.GetPath())
mesh_merge_collection_477 = mesh_merge_collision_477.GetCollisionMeshesCollectionAPI()
mesh_merge_collection_477.GetIncludesRel().ClearTargets(removeSpec=False)
mesh_merge_collection_477.GetIncludesRel().AddTarget(component_477_path)

mesh_merge_collision_520 = PhysxSchema.PhysxMeshMergeCollisionAPI.Get(stage, prim_520.GetPath())
mesh_merge_collection_520 = mesh_merge_collision_520.GetCollisionMeshesCollectionAPI()
mesh_merge_collection_520.GetIncludesRel().ClearTargets(removeSpec=False)
mesh_merge_collection_520.GetIncludesRel().AddTarget(component_520_path)

stage.GetRootLayer().Save()