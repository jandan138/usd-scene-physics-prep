# Isaac Sim / Omniverse Kit 的 MDL 渲染与路径配置（通俗版）

> 最后更新：2026-01-23

这篇文档解释 3 件事：

1) Isaac Sim 渲染时 MDL 是怎么“被找到并编译”的
2) 为什么你看到的报错是 `could not find module '::KooPbr'`
3) 为什么 `MDL_SEARCH_PATH` 经常“不生效”，`MDL_SYSTEM_PATH` 可能“能救急”，以及最稳的做法是什么

---

## 0. 一句话结论（建议记住）

- **最稳的方式**：用 Kit 的 settings 显式把数据集的 `Material/mdl` 加进 **MDL 搜索路径**（例如 `/app/mdl/additionalSystemPaths` 或 `/renderer/mdl/searchPaths/required`）。
- **不推荐只靠环境变量**：`MDL_SEARCH_PATH` 在 Isaac Sim/Kit 的渲染链路里经常根本没被读取；`MDL_SYSTEM_PATH` 虽然可能被 MDL SDK 读取，但对“启动方式/继承环境/覆盖优先级”更敏感。

---

## 1. 你遇到的错误到底是什么意思

典型报错：

- `C120 could not find module '::KooPbr'`

在 MDL 语言里，`::KooPbr` 表示**一个 MDL 模块名**（不是文件路径）。MDL 编译器会在“MDL 搜索路径列表”里查找对应文件：

- `KooPbr.mdl`（或与模块名匹配的 `.mdl`）

因此，如果你的数据集里确实有：

- `GRScenes-test1/Material/mdl/KooPbr.mdl`

那你需要确保 **`GRScenes-test1/Material/mdl` 这个目录本身**在 MDL 搜索路径里。

> 注意：很多人会误以为“USD 引用了 `MI_xxx.mdl`，就会自动找到它 import 的其他 `.mdl`”。实际不是：MDL 内部的 `import ::KooPbr` 仍然要靠 MDL 搜索路径去解析。

---

## 2. Isaac Sim 渲染时，MDL 大致工作流（简化版）

把它当成两段链路：

### 2.1 USD 阶段：材质里引用了某个 `.mdl`

- USD 的材质 shader（例如 MDL shader/OmniPBR 等）会在某个属性里指向 `.../Material/mdl/MI_xxx.mdl`（asset path）。
- 这一步只是“告诉渲染器要用哪个 MDL 文件/模块”。

### 2.2 渲染阶段：Kit/Renderer 触发 MDL 编译

当 viewport/renderer 真正需要这个材质时：

1) Kit 调用 MDL SDK（或其封装）去加载并编译该 MDL
2) 在编译过程中，MDL 文件里可能 `import ::KooPbr`、`import ::KooPbr_maps` 等
3) MDL SDK 会在“它认为的搜索路径列表”里找这些模块
4) 找不到就报 `could not find module ...`

关键点在第 3 步：**MDL SDK 用的搜索路径列表，主要来自 Kit settings，而不是你 shell 里 export 的变量。**

---

## 3. Kit 到底从哪里读 MDL 搜索路径（你这次踩坑的根因）

在 Isaac Sim/Omniverse Kit 里，MDL 搜索路径通常由 settings 驱动，常见相关 key：

- `/renderer/mdl/searchPaths/required`
  - 形如：`/isaac-sim/kit/mdl/rtx/;/isaac-sim/kit/mdl/rtx/iray/;...`
  - **分号 `;` 分隔**的一条字符串
- `/renderer/mdl/searchPaths/templates`
  - 模板/基础模块相关路径（通常是 Kit 自带的 Base 模块模板）
- `/app/mdl/additionalSystemPaths`
  - **string array**（列表），用于追加“系统级”MDL 目录
- `/app/mdl/additionalUserPaths`
  - **string array**（列表），用于追加“用户级”MDL 目录

你的现象“在终端里设了 `MDL_SEARCH_PATH` 但 Isaac Sim 还是找不到 `::KooPbr`”，常见原因是：

- Isaac Sim 的启动脚本/Kit 启动器没有把 `MDL_SEARCH_PATH` 传到最终负责渲染/编译 MDL 的进程（或进程启动前后被覆盖）
- Kit 的 MDL 配置是 settings 驱动的，相关 extension 更信任 settings，而不是每次运行都去读 env

---

## 4. 为什么 `MDL_SEARCH_PATH` 往往“不行”

你可以把它理解成：

- `MDL_SEARCH_PATH` 是“某些 MDL 工具链/SDK 可能会读的环境变量”
- 但 Isaac Sim/Kit 的实际渲染链路里，MDL 搜索路径通常是 **由 settings 管控并组装** 的

所以即使你 export 了：

- `MDL_SEARCH_PATH=/path/to/GRScenes-test1/Material/mdl`

Kit 也可能：

- 根本没读它
- 或者读了但被 settings 覆盖
- 或者你设置的环境变量没有进入最终生效的进程

结果就是：你认为你“设置了”，但渲染器编译 MDL 时依然用的是默认搜索路径。

---

## 5. 为什么 `MDL_SYSTEM_PATH` 可能“能行”（但不够稳）

`MDL_SYSTEM_PATH` 往往对应 MDL SDK 的“系统路径”概念。

如果你的 Isaac Sim 版本/某些 extension 正好把 MDL SDK 的 system path 直接交给了环境变量机制，那么 `MDL_SYSTEM_PATH` 可能会被读到，从而让 `::KooPbr` 解析成功。

但它“不够稳”，主要是因为：

- **环境变量继承链**不透明：不同启动方式（桌面快捷方式、脚本、容器、远程、Launcher）继承到的环境不同
- **优先级/覆盖**不透明：settings、用户配置、extension 逻辑可能覆盖/追加/重写这些值
- **可审计性差**：排障时你很难一眼看出最终生效的搜索路径到底是什么（而 settings 是可读可打印的）

因此：`MDL_SYSTEM_PATH` 可以当“临时救急”，但长期维护建议用 settings。

---

## 6. 推荐的稳定配置方式（可复制粘贴）

下面两种方式选一种即可。

### 6.1 方式 A（推荐）：追加 `/app/mdl/additionalSystemPaths`

把数据集目录加到 additional system paths（array setting）。

示例（把 `<ABS_SUBSET_ROOT>` 换成你的绝对路径）：

```bash
/isaac-sim/isaac-sim.sh \
  --allow-root \
  --/app/mdl/additionalSystemPaths/0=<ABS_SUBSET_ROOT>/GRScenes-test1/Material/mdl
```

说明：

- `/0` 表示数组第 0 项（如果你已经有其他项，可以改成 `/1`、`/2`）
- 这通常是“最不容易跟默认值打架”的追加方式

### 6.2 方式 B：直接改 `/renderer/mdl/searchPaths/required`

把 required 搜索路径字符串里追加一个目录（用 `;` 分隔）：

```bash
/isaac-sim/isaac-sim.sh \
  --allow-root \
  --/renderer/mdl/searchPaths/required="/isaac-sim/kit/mdl/rtx/;/isaac-sim/kit/mdl/rtx/iray/;<ABS_SUBSET_ROOT>/GRScenes-test1/Material/mdl"
```

说明：

- 这更“硬”，因为你要写出原本的默认 required，再追加自己的路径
- 优点是直观；缺点是 Isaac Sim 升级后默认路径变了，你需要同步更新命令

---

## 7. 快速自检：确认路径真的生效

如果你想在 Isaac Sim 里确认 settings 是否真的被设置上了，可以在 Script Editor 里运行：

```python
import carb
s = carb.settings.get_settings()
print("required:", s.get("/renderer/mdl/searchPaths/required"))
print("templates:", s.get("/renderer/mdl/searchPaths/templates"))
print("additionalSystemPaths:", s.get("/app/mdl/additionalSystemPaths"))
print("additionalUserPaths:", s.get("/app/mdl/additionalUserPaths"))
```

看到 `additionalSystemPaths` 里包含你的 `.../Material/mdl`，基本就说明 `::KooPbr` 的解析路径已经进来了。

---

## 8. 常见误区

- 误区 1：把 `MI_xxx.mdl` 的路径加进去就够了
  - 不够。你要加的是“包含 `KooPbr.mdl` 的目录”，让 `import ::KooPbr` 能找到模块。

- 误区 2：USD 路径都对了，MDL 一定能编译
  - 不一定。MDL 的 `import` 解析与 USD asset path 是两套东西。

- 误区 3：只在启动前 export 环境变量就一定能影响 Isaac Sim
  - 不一定。Kit 更倾向 settings 管控；而且不同启动方式环境继承链不同。

---

## 9. 针对 GRScenes-test1 的建议做法（维护向）

- 把 `GRScenes-test1/Material/mdl` 作为“系统追加 MDL 目录”（方式 A）。
- 需要长期稳定时，考虑把这条 setting 固化到你们的启动脚本/任务脚本里（比如一个统一的 `isaacsim_with_dataset_mdl.sh`），避免每个人手动配置导致复现困难。
