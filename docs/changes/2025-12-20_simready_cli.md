# 2025-12-20：新增一键生成 Sim-Ready USD 的 CLI

## 背景与目标
项目原本的典型流程是：
- 先用 `clean_data.py` 将原始场景拆分/规范化到 `target/`；
- 再分别用 `set_physics/preprocess_for_interaction.py` 或 `set_physics/preprocess_for_navigation.py` 在 Isaac Sim 环境中给场景写入物理/语义信息；
- （可选）通过 `set_physics/get_all_references.py` + `set_physics/export_scene.py` 打包分发。

在“只给一个 USD，想直接得到一个物理化后的 sim-ready USD”这一需求下，上述流程需要手动改脚本路径、以及手动串联多个步骤。

本次新增一个 CLI：`set_physics/simready.py`，将 **输入一个 USD → 输出一个物理化好的 sim-ready USD** 封装成一条命令（Linux / Isaac Sim 环境）。

## 新增内容
- 新增入口脚本：`set_physics/simready.py`

它提供两个模式：
- `interaction`：生成动态交互用场景（为 pickable 对象绑定刚体 + 碰撞近似；Base 静态三角网格；动画/关节对象启用 joints）。
- `navigation`：生成导航用场景（静态三角网格碰撞 + 语义标签；可选禁用 door）。

默认会先调用 repo 既有的拆分逻辑 `set_physics/pxr_utils/data_clean.py:parse_scene`，在 `--out-root` 下创建类似 `target/` 的结构，然后再在该结构的场景 USD 上写入物理（以及可选语义）。

## 运行环境
- Linux
- 使用 Isaac Sim 自带 Python 启动（确保 `pxr`、`PhysxSchema`、`isaacsim`、`omni.*` 可用）

## 使用方法

### 1）交互（dynamic / interaction）
```bash
./python.sh -m set_physics.simready \
  --input-usd /path/to/start_result_new.usd \
  --out-root /path/to/out \
  --mode interaction \
  --headless
```
输出通常位于：
- `/path/to/out/scenes/<scene_id>/start_result_dynamic.usd`

### 2）导航（static + semantics / navigation）
```bash
./python.sh -m set_physics.simready \
  --input-usd /path/to/start_result_new.usd \
  --out-root /path/to/out \
  --mode navigation \
  --headless \
  --disable-doors
```
输出通常位于：
- `/path/to/out/scenes/<scene_id>/start_result_navigation.usd`

### 3）跳过拆分（输入已是规范化场景）
如果你传入的 USD 已经满足：
- 具备 `/Root/Meshes` 层级；
- 引用/材质路径已经可解析；

可以使用 `--skip-clean` 只做物理化：
```bash
./python.sh -m set_physics.simready \
  --input-usd /path/to/already_normalized.usd \
  --out-root /path/to/out \
  --mode interaction \
  --headless \
  --skip-clean
```

## 输入数据约束（重要）
- 默认（不加 `--skip-clean`）要求：输入 USD 所在目录下存在 `Materials/*.mdl`，因为 `parse_scene` 会从同级 `Materials/` 复制材质到输出结构。
- 输出结构会包含 `Materials/` 与 `models/`，并在 `scenes/<scene_id>/` 下创建指向它们的链接（Linux 下可用）。

## 相关提交
- Commit: `c17f640`（Add simready one-shot USD physics CLI）
