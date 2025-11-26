# from pxr import Usd

# def save_cooking_cache_to_usd(stage_path):
#     # 打开USD文件
#     stage = Usd.Stage.Open(stage_path)
#     if not stage:
#         print(f"Failed to open stage at {stage_path}.")
#         return

#     # 保存烹饪缓存
#     stage.Save()

#     # 保存烹饪缓存到USD文件
#     stage.SetMetadata("cooking:cache", True)
#     stage.SetMetadata("cooking:cache:enabled", True)
#     stage.SetMetadata("cooking:cache:location", "embedded")

#     # 保存更改
#     stage.Save()

# # 定义USD文件路径
# usd_file_path = '/path/to/your/usd/file.usd'

# # 调用函数保存烹饪缓存到USD文件
# save_cooking_cache_to_usd(usd_file_path)