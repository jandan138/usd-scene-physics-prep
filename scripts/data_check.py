import os
import os.path as osp
import json
import argparse
from typing import List, Dict, Optional

def check_asset_structure(asset_dir: str, progress: Optional[Dict[str, int]] = None) -> Dict[str, List[str]]:
    """
    :param asset_dir: Asset path
    :return: Dict with all error info
    """
    result = {}
    assert osp.isdir(asset_dir)
    
    for asset_category in sorted(os.listdir(asset_dir)):
        asset_category_path = osp.join(asset_dir, asset_category)
        if not osp.isdir(asset_category_path):
            continue
        
        for uid in sorted(os.listdir(asset_category_path)):
            uid_path = osp.join(asset_category_path, uid)
            if not osp.isdir(uid_path):
                continue

            if progress is not None:
                progress["scanned"] = progress.get("scanned", 0) + 1
                if progress["scanned"] % progress.get("every", 1000) == 0:
                    print(f"[progress] scanned_assets={progress['scanned']} errors={progress.get('errors', 0)}")
            
            errors = []
            asset_key = f"{asset_dir}/{asset_category}/{uid}"
            
            # 1. whether contain "usd" folder
            usd_dir = osp.join(uid_path, "usd")
            if not osp.isdir(usd_dir):
                errors.append(f"missing usd folder in {uid}")
            else:
                # whether {uid}.usd in the "usd" folder
                usd_file = osp.join(usd_dir, f"{uid}.usd")
                if not osp.isfile(usd_file):
                    errors.append(f"no usd file in {uid}/usd folder")
            
            # 2. whether contain "glb" or "urdf" folder
            glb_dir = osp.join(uid_path, "glb")
            urdf_dir = osp.join(uid_path, "urdf")
            if not osp.isdir(glb_dir) and not osp.isdir(urdf_dir):
                errors.append(f"missing glb or urdf folder in {uid}")
            else:
                if osp.isdir(glb_dir):
                    glb_file = osp.join(glb_dir, f"{uid}.glb")
                    if not osp.isfile(glb_file):
                        errors.append(f"no glb file in {uid}/glb folder")
                if osp.isdir(urdf_dir):
                    urdf_file = osp.join(urdf_dir, f"{uid}.urdf")
                    if not osp.isfile(urdf_file):
                        errors.append(f"no urdf file in {uid}/urdf folder")
            
            # 3. check the format of {uid}_annotation.json
            annotation_file = osp.join(uid_path, f"{uid}_annotation.json")
            if osp.isfile(annotation_file):
                required_keys = [
                    "uid", "category", "description", "material", 
                    "dimensions", "mass", "placement", "asset_type", 
                    "usd_size", "usd_material_softlink"
                ]
                try:
                    with open(annotation_file, "r", encoding="utf-8") as f:
                        data = json.load(f)
                    missing_keys = [key for key in required_keys if key not in data]
                    if missing_keys:
                        errors.append(f"{uid}_annotation.json miss keys: {', '.join(missing_keys)}")
                except json.JSONDecodeError:
                    errors.append("Not a valid json file")
                except Exception as e:
                    errors.append(f"Read json file error: {str(e)}")
            else:
                errors.append(f"no annotation file: {uid}_annotation.json")
            
            # 4. check rendering images
            required_images = ["front.png", "left.png", "right.png", "back.png"]
            for img in required_images:
                img_path = osp.join(uid_path, img)
                # consider both 'png' & 'jpg' format
                if osp.isfile(img_path) or osp.isfile(img_path.replace('png', 'jpg')):
                    continue
                else:
                    errors.append(f"{uid} miss the {img[:-4]} view image")
            
            if errors:
                if progress is not None:
                    progress["errors"] = progress.get("errors", 0) + 1
                result[asset_key] = errors
    
    return result

def check_scene_structure(scene_dir: str) -> Dict[str, List[str]]:
    """
    :param scene_dir: Scene path
    :return: Dict with all error info
    """
    result = {}
    assert osp.isdir(scene_dir)

    for scene_category in sorted(os.listdir(scene_dir)):
        scene_category_path = osp.join(scene_dir, scene_category)
        if not osp.isdir(scene_category_path):
            continue

        for sid in sorted(os.listdir(scene_category_path)):
            sid_path = osp.join(scene_category_path, sid)
            if not osp.isdir(sid_path):
                continue

            errors = []
            scene_key = f"{scene_dir}/{scene_category}/{sid}"

            # 1. whether contain layout.json
            layout_json_file = osp.join(sid_path, 'layout.json')
            if not osp.isfile(layout_json_file):
                errors.append(f"{sid} miss layout.json")
            
            # 2. whether contain layout.usd
            layout_usd_file = osp.join(sid_path, 'layout.usd')
            if not osp.isfile(layout_usd_file):
                errors.append(f"{sid} miss layout.usd")

            # 3. whether contain front.png
            img_file_png = osp.join(sid_path, 'front.png')
            img_file_jpg = osp.join(sid_path, 'front.jpg')
            if not (osp.isfile(img_file_png) or osp.isfile(img_file_jpg)):
                errors.append(f"{sid} miss the rendering image")

            # 4. check the format of {sid}_annotation.json
            annotation_file = osp.join(sid_path, f"{sid}_annotation.json")
            if osp.isfile(annotation_file):
                required_keys = ["sid", "category", "description"]
                try:
                    with open(annotation_file, "r", encoding="utf-8") as f:
                        data = json.load(f)
                    missing_keys = [key for key in required_keys if key not in data]
                    if missing_keys:
                        errors.append(f"{sid}_annotation.json miss keys: {', '.join(missing_keys)}")
                except json.JSONDecodeError:
                    errors.append("Not a valid json file")
                except Exception as e:
                    errors.append(f"Read json file error: {str(e)}")
            else:
                errors.append(f"no annotation file: {sid}_annotation.json")
            
            if errors:
                result[scene_key] = errors
    
    return result

def main(data_dir, progress_every: int = 1000):
    folders = os.listdir(data_dir)

    all_error_msg = []
    for folder in folders:
        cur_folder = osp.join(data_dir, folder)
        if 'asset' in folder.lower():  # only check object assets
            print("Check assets:", cur_folder)
            progress = {"scanned": 0, "errors": 0, "every": progress_every}
            check_results = check_asset_structure(cur_folder, progress=progress)
            print(f"[summary] assets_scanned={progress['scanned']} assets_with_errors={progress['errors']}")
        else:
            continue

        # parse error info
        if check_results:
            for asset_path, errors in check_results.items():
                all_error_msg.append(f"\n Current path: {asset_path}\n")
                for error in errors:
                    all_error_msg.append(f"  - {error}\n")

    with open('error_info.log', 'w') as fn:
        fn.write("".join(all_error_msg))

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="MesaTask-10K/final/test")
    ap.add_argument("--progress-every", type=int, default=1000)
    args = ap.parse_args()
    main(args.data_dir, progress_every=args.progress_every)