from isaacsim import SimulationApp

# 启动 Isaac Sim
simulation_app = SimulationApp({"headless": False})

# 导入必要的模块
from pxr import Gf, Usd
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core import World

# 启用 Viewport 扩展
enable_extension("omni.kit.viewport.window")

# 获取当前 Stage
stage_path = '/ssd/zhaohui/workspace/w61_grutopia_1220/assets/robots/h1/h1_vln_pano_pointcloud_200.usd'
stage = Usd.Stage.Open(stage_path)

# 设置 Viewport 的分辨率
viewport_window_1 = simulation_app.create_viewport_window("viewport1")
viewport_window_1.set_texture_resolution(1280, 720)  # 设置分辨率为 1280x720

# 设置 Viewport 的背景颜色
viewport_window_1.set_window_background_color(0.1, 0.1, 0.1, 1.0)  # 设置为深灰色

# 设置相机视角
camera_path_1 = "/h1_description/topdown_camera_500"  # 相机路径
set_camera_view(
    eye=Gf.Vec3f(5, 5, 5),  # 相机位置
    target=Gf.Vec3f(0, 0, 0),  # 目标位置
    camera_prim_path=camera_path_1,  # 相机路径
)


# 设置 Viewport 的分辨率
viewport_window_2 = simulation_app.create_viewport_window("viewport2")
viewport_window_2.set_texture_resolution(1280, 720)  # 设置分辨率为 1280x720

# 设置 Viewport 的背景颜色
viewport_window_2.set_window_background_color(0.1, 0.1, 0.1, 1.0)  # 设置为深灰色

# 设置相机视角
camera_path_2 = "/h1_description/topdown_camera_50"  # 相机路径
set_camera_view(
    eye=Gf.Vec3f(5, 5, 5),  # 相机位置
    target=Gf.Vec3f(0, 0, 0),  # 目标位置
    camera_prim_path=camera_path_2,  # 相机路径
)

# # 运行模拟
# world.reset()
# for _ in range(100):  # 模拟 100 步
#     world.step(render=True)  # 更新物理和渲染

# # 关闭 Isaac Sim
# simulation_app.close()