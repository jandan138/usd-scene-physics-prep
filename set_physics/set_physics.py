import os
import json
from pxr import Usd, UsdPhysics
from utils.read_info import read_physicsCsv
from utils.usd_physics import set_collider_, set_rigidbody, set_collider

if __name__ == "__main__":
    physicsCsv_path = "./info/physics_list.csv"
    physics_dict = read_physicsCsv(physicsCsv_path)

    models_dir = "./dest_usd/models"
    interactive_dir = os.path.join(models_dir, "interactive")
    static_dir = os.path.join(models_dir, "static")

    """Interactive"""
    for inter in os.listdir(interactive_dir):
        # print(inter)
        pass


    """Static"""
    for stat in os.listdir(static_dir):
        print(stat)
        pass
    # for item in items_list:     # bed
    #     instances = os.listdir(os.path.join(models_dir, item))
    #     for instance in instances:      # SM
    #         physics_property = physics_dict[instance]
    #         instance_path = os.path.join(models_dir, item, instance, 'instance.usd')
    #         stage = Usd.Stage.Open(instance_path)
    #         world = stage.GetPrimAtPath("/Root")
    #         xform_Instance = world.GetChild("Instance")
    #         for group in xform_Instance.GetChildren():
    #             if group.GetTypeName() == "Xform":
    #                 ## set rigidbody
    #                 if physics_property["rigidbody"] == True:
    #                     set_rigidbody(group)
    #                 ## set collision
    #                 if physics_property["collider"] == True:
    #                     set_collider_recursion(group)
    #         stage.Export(os.path.join(models_dir, item, instance, 'instance_physics_2.usda'))
