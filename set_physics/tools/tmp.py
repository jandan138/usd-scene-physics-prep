from isaacsim import SimulationApp
CONFIG = {"headless": True, "renderer": "RayTracedLighting"}
kit = SimulationApp(launch_config=CONFIG)
import omni.isaac.sensor
from omni.kit.viewport.utility import get_active_viewport_window
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles

from pxr import Usd, Gf, Sdf, UsdGeom, UsdShade
import numpy as np

stage_path = "/ssd/tianshihan/target_69_new_bk/scenes/MWAX5JYKTKJZ2AABAAAAACA8_usd/start_result_new.usd"
# stage = Usd.Stage.Open(stage_path)
omni.usd.get_context().open_stage(stage_path)
stage = omni.usd.get_context().get_stage()

# stage.Save()

# create camera
camera_livingroom_path = "/Root/camera_livingroom_3"
camera_livingroom_prim = UsdGeom.Camera.Define(stage, camera_livingroom_path)
# 设置相机属性
camera_livingroom_prim.GetPrim().GetAttribute("focalLength").Set(10)  # 焦距 18
camera_livingroom_prim.GetPrim().GetAttribute("horizontalAperture").Set(20.955)  # 水平孔径 20.955
camera_livingroom_prim.GetPrim().GetAttribute("clippingRange").Set((0.01, 10000000.0))  # 裁剪范围 (0.01, 10000000.0)

# 设置相机的位置和旋转
xform_api = UsdGeom.XformCommonAPI.Get(stage, camera_livingroom_path)
xform_api.SetTranslate(Gf.Vec3d(300, 50, 220), Usd.TimeCode.Default())  # 设置相机位置 (159.79, 58.97, 193.11)
xform_api.SetRotate(Gf.Vec3f(60, 0, 180), UsdGeom.XformCommonAPI.RotationOrderXYZ, Usd.TimeCode.Default())  # 设置相机旋转 (69.05806, 0, -143.09053)


print("origin camera ratate XYZ: ", camera_livingroom_prim.GetPrim().GetAttribute('xformOp:rotateXYZ').Get())

camera = omni.isaac.sensor.Camera(prim_path=str(camera_livingroom_prim.GetPath()),
                                  resolution=(1920, 1080),
                                  frequency=60)
camera.initialize()

quat = camera.prim.GetAttribute('xformOp:orient').Get()
print("after init, camera orient: ", quat)
print("quat to vec: ", quat_to_euler_angles(np.ndarray(quat)))


def camera_rotate(camera: omni.isaac.sensor.Camera, rotate_x=True, rotate_y=True, rotate_z=True, angle_x=0, angle_y=0, angle_z=0):
    orient_attr = camera.prim.GetAttribute("xformOp:orient")
    # orient_origin_quad = orient_attr.Get()
    # print("rotate_origin_quad: ", orient_origin_quad)

    angle_x_rad = np.radians(angle_x)
    angle_y_rad = np.radians(angle_y)
    angle_z_rad = np.radians(angle_z)

    quat_x = Gf.Quatd(np.sin(angle_x_rad / 2), 0, 0, np.cos(angle_x_rad / 2))
    quat_y = Gf.Quatd(0, np.sin(angle_y_rad / 2), 0, np.cos(angle_y_rad / 2))
    quat_z = Gf.Quatd(0, 0, np.sin(angle_z_rad / 2), np.cos(angle_z_rad / 2))

    # merge rotation (order: X -> Y -> Z)
    orient_final_quad = quat_x * quat_y * quat_z
    print("merge: ", orient_final_quad)
    orient_attr.Set(orient_final_quad)


camera_rotate(camera, True, False, False, 58, 0, 180)
quat = camera.prim.GetAttribute('xformOp:orient').Get()
print("after rotation, camera orient: ", quat)
print("quat to vec: ", quat_to_euler_angles(np.ndarray(quat)))

# print(camera.prim.GetAttribute("xformOp:orient"))
# from pxr import Gf, UsdGeom
# import omni.isaac.core
# import omni.usd

# # 获取当前 Stage
# stage = omni.usd.get_context().get_stage()

# # 获取相机 Prim
# camera_path = "/World/Camera"  # 替换为你的相机路径
# camera_prim = stage.GetPrimAtPath(camera_path)
# if not camera_prim:
#     print(f"Camera at path {camera_path} not found.")
#     exit()

# # 获取相机的 Xform 对象
# camera_xform = UsdGeom.Xformable(camera_prim)

# # 定义相机位置和目标点
# eye = Gf.Vec3f(0.0, 0.0, 5.0)  # 相机位置
# target = Gf.Vec3f(0.0, 0.0, 0.0)  # 目标点
# up = Gf.Vec3f(0.0, 1.0, 0.0)  # 上方向向量

# # 计算相机的旋转四元数
# quatf = omni.isaac.core.lookat_to_quatf(eye, target, up)

# # 将旋转四元数应用到相机
# camera_xform.ClearXformOpOrder()
# camera_xform.AddRotateXYZOp().Set(quatf.GetAxis(), quatf.GetAngle())

# # 保存 Stage
# stage.GetRootLayer().Save()
