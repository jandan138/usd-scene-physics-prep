"""
文件系统工具 fs

整体介绍：
- 提供跨平台的目录创建、单文件复制、整目录复制功能。
- 所有复制均使用 `shutil.copy2` 保留元数据；在执行前确保目标目录存在。
"""

# 导入标准库 os 与 shutil，用于路径与复制操作
import os
import shutil

# 确保目录存在（不存在则创建）
def ensure_dir(p):
    os.makedirs(p, exist_ok=True)

# 复制单个文件到目标路径，同时保证目标目录已创建
def copy_file(src, dst):
    ensure_dir(os.path.dirname(dst))
    shutil.copy2(src, dst)

# 递归复制整个目录
def copy_dir(src, dst):
    for root, dirs, files in os.walk(src):
        # 计算相对路径，定位目标子目录
        rel = os.path.relpath(root, src)
        target = os.path.join(dst, rel) if rel != "." else dst
        # 确保目标子目录存在
        ensure_dir(target)
        # 逐个复制文件
        for f in files:
            copy_file(os.path.join(root, f), os.path.join(target, f))
