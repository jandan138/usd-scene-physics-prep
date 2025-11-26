# 导航预处理

## 目标
- 将场景处理为适合导航仿真的静态版本：禁用非主门、对象采用静态三角网格近似、写语义标签。

## 关键函数
- 语义标注：`set_semantic_label`（`set_physics/preprocess_for_navigation.py:189-196,198-231`）。
- 门处理：依据 `main_door_prim_path_new.json` 保留主门，其余禁用（`set_physics/preprocess_for_navigation.py:403-411`）。
- 静态近似绑定：`bind_static_for_merged_mesh`（`set_physics/preprocess_for_navigation.py:184-187,210-231`）。

## 输出
- 保存为 `start_result_navigation.usd`（`set_physics/preprocess_for_navigation.py:424-425`）。

