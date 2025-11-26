import os
import json
from pxr import Usd, UsdPhysics, UsdGeom, Sdf
from pxr_utils.read_info import read_physicsCsv
from pxr_utils.usd_physics import set_collider_, set_rigidbody, set_collider, remove_rigid

import tqdm

def copy(a, b):
    cmd = f"cp -r {a} {b}"
    os.system(cmd)

static_base_category = [
    "faucet",
    "toilet",
    "oven"
]

def solve():
    src_path = "./target/models/layout/articulated"
    cates = [_ for _ in os.listdir(src_path)]
    # print(files)
    for cate in cates:
        folder_path = os.path.join(src_path, cate)
        obj_ids = [_ for _ in os.listdir(folder_path)]
        # print(obj_ids)
        # break
        for instance in tqdm.tqdm(obj_ids,desc=f"{cate}"):
            
            export_path = os.path.join(folder_path, instance, "instance.usd")
            # export_path = os.path.join(folder_path, instance, "instance_d.usd")
            
            # copy(instance_path, export_path)
            # print(instance_path)
            stage = Usd.Stage.Open(export_path)
            instance_usd = stage.GetPrimAtPath("/Root/Instance")

            group_path =  "/Root/Instance/CollisionGroup"

            # create a collision group
            group = UsdPhysics.CollisionGroup.Define(stage, group_path)
            # group.CreateFilteredGroupsRel()
            # print(group.GetFilteredGroupsRel())
            
            # Add the current collision group to the filtered group
            group.GetFilteredGroupsRel().AddTarget(group_path)

            # Add all parts of this instance to the collision group
            group.GetCollidersCollectionAPI().GetIncludesRel().AddTarget("/Root/Instance")


            # print(instance_usd)
            if not Usd.Object.IsValid(instance_usd):
                continue
            children = instance_usd.GetChildren()
            flag = False
            for child in children:
                childName = str(child.GetName())
                # print(childName)

                if not childName.lower() == 'group_static' and child.IsA(UsdGeom.Xform):
                    # print(childName)
                    set_rigidbody(child)
                    set_collider(child)

                if childName.lower() == 'group_static' or (childName.lower() == 'group_00' and cate in static_base_category):
                    # set_collider(child)
                    set_collider(child, "meshSimplification")
                    remove_rigid(child)

                if child.IsA(UsdPhysics.PrismaticJoint) or child.IsA(UsdPhysics.RevoluteJoint):
                    attr = child.GetAttribute("physics:jointEnabled")
                    # print(attr.Get())
                    attr.Set(True)
                    flag = True

            # export_path = os.path.join(folder_path, instance, "instance_d.usd")
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


if __name__ == '__main__':
    solve()