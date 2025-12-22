# 常见问题

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../set_physics/pxr_utils/data_clean.py
> - ../../set_physics/preprocess_for_interaction.py
> - ../../set_physics/preprocess_for_navigation.py
> - ../../set_physics/simready.py
>
> 总索引：../overview/docs_index.md

## 索引
- [Windows 下 `cp` 命令不可用](#windows-下-cp-命令不可用)
- [软链接创建失败](#软链接创建失败)
- [贴图或材质路径失效](#贴图或材质路径失效)
- [Isaac Sim 模块未找到](#isaac-sim-模块未找到)
- [碰撞近似计算慢](#碰撞近似计算慢)
- [外部数据集（`/root` 结构）如何做交互预处理？](#外部数据集root-结构如何做交互预处理)
- [如何判断哪些对象可 Shift+左键拖拽？](#如何判断哪些对象可-shift左键拖拽)

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

## 外部数据集（`/root` 结构）如何做交互预处理？
- 如果输入场景根节点是 `/root`（例如 SimBench GRSceneUSD/task10），并且 `simready.py` 因 `Materials/` 缺失或目录结构不匹配而失败：
	- 直接使用 `scripts/prep_interaction_root_scene.py` 在原始 USD 上绑定 collider/rigid。
	- 参数推荐与常见坑位见：`docs/operations/troubleshooting_interaction_preprocess.md`。

## 如何判断哪些对象可 Shift+左键拖拽？
- 经验上需要：刚体启用（`physics:rigidBodyEnabled=True`）且子树里存在 collider（`physics:collisionEnabled=True`）。
- 可以直接运行：`scripts/list_draggable_prims.py`（输出“可拖拽候选”以及“刚体但无 collider”的对象）。

