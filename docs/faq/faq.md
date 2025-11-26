# 常见问题

## Windows 下 `cp` 命令不可用
- 替换为 Python 的 `shutil.copyfile`，或使用 PowerShell 的 `Copy-Item`，并在代码中统一封装复制函数。

## 软链接创建失败
- 检查管理员权限/开发者模式；必要时将软链接改为相对路径引用或复制目录。

## 贴图或材质路径失效
- 检查 `asset` 属性是否已去除父级 `..` 前缀并复制到目标（`set_physics/pxr_utils/data_clean.py:367-397`）。

## Isaac Sim 模块未找到
- 确认在 Omniverse 环境中运行脚本，或通过 `SimulationApp` 启动（参考 `set_physics/preprocess_for_interaction.py:3-6`）。

## 碰撞近似计算慢
- 减少复杂近似（如将 `convexDecomposition` 改为 `convexHull` 或 `none`），或对大对象采用静态三角网格近似。

