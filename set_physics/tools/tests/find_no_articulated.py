import os
from pxr import Usd

def find_usd_files_with_no_joints(directory):
    usd_files_with_no_joints = []

    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(".usd"):
                usd_file_path = os.path.join(root, file)

                stage = Usd.Stage.Open(usd_file_path)
                if not stage:
                    print(f"无法打开文件: {usd_file_path}")
                    continue

                # 检查是否存在 joint
                has_joint = False
                for prim in stage.Traverse():
                    if "joint" in prim.GetTypeName().lower():
                        has_joint = True
                        break

                if not has_joint:
                    usd_files_with_no_joints.append(usd_file_path)

    return usd_files_with_no_joints


import argparse

parser = argparse.ArgumentParser()

parser.add_argument('-s', '--search', type=str,
                    help='the dir path which you want to search')
parser.add_argument('-o', '--out', type=str,
                    help='the path to output the searched results')

args = parser.parse_args()

joint_usd_files = find_usd_files_with_no_joints(args.search)
if joint_usd_files:
    with open(file=args.out, mode="w") as f:
        for file_path in joint_usd_files:
            print(file_path, file=f)
else:
    print("未找到不包含 joint 的 USD 文件")