from set_physics.pxr_utils.data_clean import parse_scene
import os
import tqdm

def clean():
    # src_folder = '../../../data/usd/commercial/merge'
    # dest_folder = '../../../data/usd/commercial/dest_merge'
    # src_folder = '../../../data/home_scenes/merge'
    # dest_folder = '../../../data/home_scenes/dest_merge'
    src_folder = "home_scenes"
    dest_folder= "target"
    files = [_ for _ in os.listdir(src_folder) if os.path.isdir(os.path.join(src_folder, _))]
    if not os.path.exists(dest_folder):
        os.makedirs(dest_folder)
    for f in tqdm.tqdm(files):
        # print(f)
        # if f != 'MV4AFHQKTKJZ2AABAAAAAEA8_usd':
        #     continue
        file_path = os.path.join(src_folder, f, "start_result_fix.usd")
        if not os.path.exists(file_path):
            file_path = os.path.join(src_folder, f, "start_result_new.usd")

        parse_scene(file_path, dest_folder, f)
        # break

def clean_demo():
    # src_path = '../../demo_usd/demo_0000_merge.usda'
    src_path = "home_scenes/MV7J6NIKTKJZ2AABAAAAADA8_usd/start_result_new.usd"
    dest_folder = 'target'
    parse_scene(src_path, dest_folder, "MV7J6NIKTKJZ2AABAAAAADA8_usd")


def count_objects():
    from data_processing import count_object
    from collections import defaultdict

    src_folder = '../../../data/usd/commercial/merge'
    
    files = [_ for _ in os.listdir(src_folder) if os.path.isdir(os.path.join(src_folder, _))]
    res_commercial = defaultdict(int)
    for f in files:
        file_path = os.path.join(src_folder, f, "start_result.usd")
        tmp = count_object(file_path)
        for k,v in tmp.items():
            res_commercial[k] += v

    

    src_folder = '../../../data/home_scenes/merge'

    files = [_ for _ in os.listdir(src_folder) if os.path.isdir(os.path.join(src_folder, _))]
    res_home = defaultdict(int)
    for f in files:
        file_path = os.path.join(src_folder, f, "start_result.usd")
        tmp = count_object(file_path)
        for k,v in tmp.items():
            res_home[k] += v

    print('home statitics')
    for k,v in res_home.items():
        print(k,v)

    print('commercial statitics')
    for k,v in res_commercial.items():
        print(k,v)


# clean_demo()
clean()
# count_objects()