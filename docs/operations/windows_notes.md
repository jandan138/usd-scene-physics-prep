# Windows 使用注意

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../set_physics/pxr_utils/data_clean.py
> - ../../set_physics/pxr_utils/usd_physics.py
>
> 总索引：../overview/docs_index.md

## 索引
- [文件复制](#文件复制)
- [软链接](#软链接)
- [路径分隔符](#路径分隔符)

## 文件复制
- 代码中有 `os.system("cp …")`（如 `set_physics/pxr_utils/data_clean.py:37`、`set_physics/pxr_utils/usd_physics.py:47`）。在 Windows 环境建议替换为 `shutil.copyfile` 或等效 Python API。

## 软链接
- 使用 `os.symlink`（`set_physics/pxr_utils/data_clean.py:535-542`）：在 Windows 可能需要管理员权限或启用开发者模式；也可改为复制目录或使用相对路径查找。

## 路径分隔符
- 源代码多使用 `/` 分隔符（USD/Omni API 通常兼容），在纯文件系统操作中注意选择合适的分隔符或统一使用 `os.path.join`。

