import os
import json
from pxr import Usd, UsdPhysics, UsdGeom
# from utils.read_info import read_physicsCsv
from pxr_utils.usd_physics import set_collider_, set_rigidbody, set_collider, remove_collider

import tqdm

def copy(a, b):
    cmd = f"cp -r {a} {b}"
    os.system(cmd)

def solve():
    src_path_list = []
    for i in range(7):
        root_path = os.path.join("target/scenes", f"{i}")
        files = [os.path.join(root_path,_) for _ in os.listdir(root_path) if os.path.isdir(os.path.join(root_path, _))]
        src_path_list += files
    for src_path in src_path_list:
        stage_path = os.path.join(src_path, "start_result.usd")
        new_stage_path = os.path.join(src_path, "start_result_new.usd")
        copy(stage_path, new_stage_path)
        stage = Usd.Stage.Open(new_stage_path)
        # doors = stage.GetPrimAtPath("/Root/Meshes/BaseAnimation/door")
        stage.RemovePrim("/Root/Meshes/BaseAnimation/door")
        # remove_collider(meshes)

        # export_path = os.path.join(folder_path, instance, "instance_d.usda")
        # if flag:
        #     movable_path_list.append(export_path)
        # print(export_path)
        # stage.Export(export_path)
        # break
        stage.GetRootLayer().Save()
            

        # break
    # with open('interactive_objects.txt','w') as fp:
    #     for l in movable_path_list:
    #         fp.write(l+'\n')


def bind_rigid(prim):
    pass


def deactivate_rigid():
    ROOT_PATH="home_scenes"
    model_path = os.path.join(ROOT_PATH, )


def solve_remove_colliders_and_rigid_bodies():
    src_path = "/home/PJLAB/wanghanqing/data/data/usd/commercial/market/scenes/MV4AFHQKTKJZ2AABAAAAAEI8_usd"
    stage_path = os.path.join(src_path, "start_result.usd")
    new_stage_path = os.path.join(src_path, "start_result_new.usd")
    copy(stage_path, new_stage_path)

    stage = Usd.Stage.Open(new_stage_path)
    meshes = stage.GetPrimAtPath("/Root/Meshes")
    remove_collider(meshes)
    
def bind_basic_colliders():
    root_path = "target/models"
    # scopes = ["interactive", "static"]
    scopes = ['object/others']
    paths = []
    for scope in scopes:
         scope_path = os.path.join(root_path, scope)
         categories = [_ for _ in os.listdir(scope_path)]
         for cate in categories:
            cate_path = os.path.join(scope_path, cate)
            instances = [_ for _ in os.listdir(cate_path)]
            for inst in instances:
                inst_path = os.path.join(cate_path, inst, "instance.usd")
                paths.append(inst_path)


    for path in tqdm.tqdm(paths):
        try:
            stage = Usd.Stage.Open(path)
            meshes = stage.GetPrimAtPath("/Root/Instance")
            remove_collider(meshes)
            set_collider(meshes)
            stage.GetRootLayer().Save()
        except:
            print("error", path)


def bind_basic_rigid():
    root_path = "target/models"
    scopes = ["interactive", "static"]
    paths = []
    for scope in scopes:
         scope_path = os.path.join(root_path, scope)
         categories = [_ for _ in os.listdir(scope_path)]
         for cate in categories:
            cate_path = os.path.join(scope_path, cate)
            instances = [_ for _ in os.listdir(cate_path)]
            for inst in instances:
                inst_path = os.path.join(cate_path, inst, "instance.usd")
                paths.append(inst_path)

    for path in tqdm.tqdm(paths):
        try:
            scope = path.split('/')[2]
            stage = Usd.Stage.Open(path)
            meshes = stage.GetPrimAtPath("/Root/Instance")
            instance_usd = stage.GetPrimAtPath("/Root/Instance")
            if not Usd.Object.IsValid(instance_usd):
                continue
            if scope == 'interactive':
                children = instance_usd.GetChildren()
                for child in children:
                    childName = str(child.GetName())
                    if not childName.lower() == 'group_static' and child.IsA(UsdGeom.Xform):
                        set_rigidbody(child)
                    if child.IsA(UsdPhysics.PrismaticJoint) or child.IsA(UsdPhysics.RevoluteJoint):
                        attr = child.GetAttribute("physics:jointEnabled")
                        attr.Set(True)
                        # flag = True
            else:
                set_rigidbody(instance_usd)
            
            stage.GetRootLayer().Save()
        except:
            print("error", path)

         

if __name__ == '__main__':
    # solve()
    bind_basic_colliders()
    # bind_basic_rigid()
