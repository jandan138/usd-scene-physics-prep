# 模块说明：data_clean

## 入口
- `clean_data.py:5-24`：遍历 `home_scenes/<scene_id>`，寻找 `start_result_fix.usd` 或 `start_result_new.usd`，调用 `parse_scene`（`set_physics/pxr_utils/data_clean.py:517-705`）。

## parse_scene
- 建立输出目录与软链接：`target/Materials`、`target/models`、`target/scenes/<scene_id>`（`set_physics/pxr_utils/data_clean.py:527-542`）。
- 复制 `.mdl` 文件到场景目录材质链接下（`set_physics/pxr_utils/data_clean.py:545-549`）。
- 创建新场景 `new_stage` 并复制根结构；跳过 `Meshes/Looks/PhysicsMaterial`（`set_physics/pxr_utils/data_clean.py:551-572,568-571`）。
- 遍历 `Meshes` 下的 scope 与类别，将实例抽取为独立模型：
  - 计算稳定指纹并用 MD5 作为 `model_hash`（`set_physics/pxr_utils/data_clean.py:612-616`）。
  - 生成 `dest_model_folder_path`、`instance.usd` 并复制材质；在场景中以 reference 引用（`set_physics/pxr_utils/data_clean.py:642-666`）。
  - 将实例的缩放迁移到模型层，使实例保持统一尺度（`set_physics/pxr_utils/data_clean.py:668-683`）。

## recursive_copy
- 关系：保留 `material:binding`，并将材质复制到 `/Root/Looks` ；对 `physics:body0/1` 目标转指到 `/Root/Instance`（`set_physics/pxr_utils/data_clean.py:167-234`）。
- 属性：跳过 `xformOp*` 变换，复制其他属性与连接；若为关节，将 `physics:jointEnabled` 置 0（`set_physics/pxr_utils/data_clean.py:308-366,337-343`）。
- 资产：相对路径统一去除父级 `..` 前缀并复制到目标；更新属性为新路径（`set_physics/pxr_utils/data_clean.py:367-397`）。
- 变换：如需复制，创建 `xformOpOrder` 与 `xformOp:transform`（`set_physics/pxr_utils/data_clean.py:295-305`）。
- 几何 primvars：同步插值（`set_physics/pxr_utils/data_clean.py:407-420`）。

## create_instance
- 为单个实例生成独立 `instance.usd`，仅拷贝必要几何与材质，并将 `xformOp:transform` 规范化为 `translate/orient/scale`（`set_physics/pxr_utils/data_clean.py:435-465,461`）。

## 其他
- 指纹获取：`unique_id`（`set_physics/pxr_utils/data_clean.py:467-503`）。
- 引用统计：`get_inst_model_mapping`、`count_object`（`set_physics/pxr_utils/data_clean.py:706-742`）。

