from pxr import Usd, UsdGeom, Sdf

from isaacsim import SimulationApp

CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}

kit = SimulationApp(launch_config=CONFIG)

import omni

def create_world_with_references(scene_usd_path, robot_usd_path, output_usd_path):
    stage = Usd.Stage.CreateNew(output_usd_path)

    # 定义 World prim
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # 在 World 下创建 scene Xform 并引用场景 USD 文件
    scene_xform = UsdGeom.Xform.Define(stage, "/World/scene")
    scene_xform.GetPrim().GetReferences().AddReference(scene_usd_path)

    # 在 World 下创建 robot Xform 并引用机器人 USD 文件
    robot_xform = UsdGeom.Xform.Define(stage, "/World/robot")
    robot_xform.GetPrim().GetReferences().AddReference(robot_usd_path)

    # 保存 USD 文件
    stage.GetRootLayer().Save()
    print(f"USD 文件已保存: {output_usd_path}")

    # 打开 USD 文件并开启仿真
    omni.usd.get_context().open_stage(output_usd_path)

# 示例文件路径
scene_usd_path = "/ssd/tianshihan/fixed/target_home_1/scenes/MVUCSQAKTKJ5EAABAAAAAAI8_usd/start_result_dynamic_fix.usd"
robot_usd_path = "/ssd/zhaohui/workspace/w61_grutopia_1220/assets/robots/h1/h1_vln_pano_pointcloud_200.usd"
output_usd_path = "/ssd/tianshihan/fixed/target_home_1/scenes/MVUCSQAKTKJ5EAABAAAAAAI8_usd/test.usd"

# 运行脚本
create_world_with_references(scene_usd_path, robot_usd_path, output_usd_path)

from omni.isaac.core import World

world = World()
world.get_physics_context().enable_gpu_dynamics(True)
world.get_physics_context().set_broadphase_type("GPU")
world.get_physics_context().enable_fabric(True)