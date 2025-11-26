from pxr import Usd
import os


def clear_physics(usd_path):
    stage = Usd.Stage.Open(usd_path)
    for prim in stage.Traverse():
        if prim.GetAttribute("physics:collisionEnabled"):
            prim.GetAttribute("physics:collisionEnabled").Set(False)
        if prim.GetAttribute("physics:rigidBodyEnabled"):
            prim.GetAttribute("physics:rigidBodyEnabled").Set(False)
    
    stage.GetRootLayer().Save()


models_path = "/ssd/tianshihan/target_30/models"

for root, _, files in os.walk(models_path):
    for file_name in files:
        instance_usd_path = os.path.join(root, file_name)
        clear_physics(instance_usd_path)
    
# clear_physics("/ssd/tianshihan/target_30/models/object/others/other/e388fdd79aff01bb1eaae7cb78808de8/instance.usd")