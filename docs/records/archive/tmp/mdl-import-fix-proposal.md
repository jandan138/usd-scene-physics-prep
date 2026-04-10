---
title: MDL 导入方式修复方案 — 技术调研报告
created_at: 2026-03-11
updated_at: 2026-03-11
maintainer: OpenCode
status: completed
doc_class: archive
---

# MDL 导入方式修复方案 — 技术调研报告

> 日期: 2026-03-11
> 状态: 调研完成，待决策
> 前置阅读: MDL 材质修复报告（仓库内未收录）

## 1. 问题回顾

GRScenes-test0 数据集中约 1.2% 的资产（~633 个，涉及 18 个类别）在 Isaac Sim GUI 中渲染为红色。

**根因**：这些资产使用的 MI_*.mdl 材质文件通过**绝对导入**（`import ::KooPbr::KooMtl;`）引用自定义 KooPbr 模块，而 KooPbr.mdl 不在 Isaac Sim 的默认 MDL 搜索路径中。详见 mdl-material-fix-correction.md（仓库内未收录）。

**现有 workaround**：render-usd CLI 管道通过 `carb.settings` 在运行时注册 `Material/mdl/` 到搜索路径。但 Isaac Sim GUI 直接打开仍然泛红。

---

## 2. MDL 的三种导入方式

| 语法 | 名称 | 解析规则 | 引入版本 |
|------|------|---------|---------|
| `import ::Module::Symbol;` | 绝对导入 | 只搜索 MDL search paths（系统路径、用户路径、carb.settings 注册路径） | MDL 1.0 |
| `using .::Module import Symbol;` | 严格相对导入 | 从**当前 .mdl 文件所在目录**查找，不查搜索路径 | MDL 1.3 |
| `import Module::Symbol;` | 弱相对导入 | 先尝试相对，再回退到绝对（MDL 1.9+ 已废弃，会报错） | MDL 1.0 |

**关键区别**：

```
绝对 ::   → "去所有注册的搜索路径里找"     → 类似 Python 的 import xxx（需要在 sys.path 中）
相对 .::  → "就在我旁边同目录下找"          → 类似 Python 的 from . import xxx（当前包内）
```

---

## 3. 为什么 OmniUe4 能用、KooPbr 不能

所有 MDL 文件都在同一个目录 `Material/mdl/` 下：

```
Material/mdl/
├── KooPbr.mdl              ← KooPbr 模块定义
├── KooPbr_maps.mdl         ← KooPbr 纹理工具
├── OmniUe4Base.mdl         ← OmniUe4 基础模块
├── OmniUe4Function.mdl     ← OmniUe4 函数库
├── MI_xxx.mdl              ← 使用 KooPbr（绝对导入 ::）  ❌
├── Num_xxx.mdl             ← 使用 OmniUe4（相对导入 .::） ✅
└── ...
```

```
Num*.mdl 第 7-8 行:
  using .::OmniUe4Function import *;     ← 相对，找同目录 → ✅ 始终成功
  using .::OmniUe4Base import *;

MI_*.mdl 第 6 行:
  import ::KooPbr::KooMtl;              ← 绝对，找搜索路径 → ❌ 默认没注册
```

**OmniUe4 还有双保险**：`OmniUe4Base.mdl` 同时也是 Isaac Sim 内置模块（位于 `/isaac-sim/kit/mdl/core/Ue4/`），即使 `.::` 相对导入失败，绝对路径也能兜底。

KooPbr 则完全依赖搜索路径——`KooPbr.mdl` 不是内置模块，`Material/mdl/` 也不在默认搜索路径中。

---

## 4. 修复方案：改为相对导入

### 4.1 修改内容

只需修改 `import` 行，将绝对导入改为严格相对导入：

```mdl
// ====== 改前（绝对导入，需要搜索路径配置）======
import ::KooPbr::KooMtl;
import ::KooPbr_maps::KooPbr_bitmap;

// ====== 改后（相对导入，同目录自动解析）======
using .::KooPbr import KooMtl;
using .::KooPbr_maps import KooPbr_bitmap;
```

**不需要修改的部分**：
- 材质体中的调用代码（`= KooPbr::KooMtl(...)` 已经是无前缀形式，兼容 `using` 导入）
- `KooPbr.mdl` 和 `KooPbr_maps.mdl` 自身（它们只 import MDL 标准库 `::anno`、`::df`、`::math` 等内置模块）

### 4.2 需替换的 11 种 import 模式

| 原始模式 | 替换为 | 出现次数 |
|---------|--------|---------|
| `import ::KooPbr::KooMtl;` | `using .::KooPbr import KooMtl;` | 1,412 |
| `import ::KooPbr_maps::KooPbr_bitmap;` | `using .::KooPbr_maps import KooPbr_bitmap;` | 436 |
| `import ::KooPbr_maps::KooPbr_rgb_output;` | `using .::KooPbr_maps import KooPbr_rgb_output;` | 436 |
| `import ::KooPbr_maps::KooPbr_mono_output;` | `using .::KooPbr_maps import KooPbr_mono_output;` | 436 |
| `import ::KooPbr_maps::KooPbr_alpha_source;` | `using .::KooPbr_maps import KooPbr_alpha_source;` | 436 |
| `import ::KooPbr_maps::KooPbr_bitmap_bump;` | `using .::KooPbr_maps import KooPbr_bitmap_bump;` | 157 |
| `import ::KooPbr_maps::KooPbr_falloff;` | `using .::KooPbr_maps import KooPbr_falloff;` | 136 |
| `import ::KooPbr::KooTranslucentMtl;` | `using .::KooPbr import KooTranslucentMtl;` | 126 |
| `import ::KooPbr_maps::NormalMap_bump;` | `using .::KooPbr_maps import NormalMap_bump;` | 60 |
| `import ::KooPbr::KooLightMtl;` | `using .::KooPbr import KooLightMtl;` | 28 |
| `import ::KooPbr::KooMtl2Sided;` | `using .::KooPbr import KooMtl2Sided;` | 20 |
| **合计** | | **3,683** |

### 4.3 修改规模

| 指标 | 数值 |
|------|------|
| 需修改的文件数 | 1,567 个（MI_*.mdl + 裸 hex-hash.mdl） |
| 无需修改的文件 | KooPbr.mdl、KooPbr_maps.mdl、所有 Num*.mdl、OmniUe4*.mdl |
| 总替换次数 | 3,683 处 import 行 |
| body 调用代码 | 无需修改 |

### 4.4 执行命令

```bash
MDL_DIR="/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0/Material/mdl"

# 两条 sed 规则覆盖全部 11 种模式（KooPbr 和 KooPbr_maps 两个模块）
sed -i \
  's/import ::KooPbr::\([A-Za-z0-9_]*\);/using .::KooPbr import \1;/g;
   s/import ::KooPbr_maps::\([A-Za-z0-9_]*\);/using .::KooPbr_maps import \1;/g' \
  "$MDL_DIR"/MI_*.mdl "$MDL_DIR"/[0-9a-f]*.mdl
```

### 4.5 验证方法

```bash
# 1. 确认没有残留的绝对 KooPbr 导入
grep -r "import ::KooPbr" "$MDL_DIR"/*.mdl
# 预期：无输出（KooPbr.mdl 和 KooPbr_maps.mdl 中没有自引用）

# 2. 确认相对导入已就位
grep -c "using .::KooPbr" "$MDL_DIR"/*.mdl | grep -v ":0$" | wc -l
# 预期：1,567（修改过的文件数）

# 3. Isaac Sim 渲染验证（不设置 MDL_SYSTEM_PATH）
# 打开 layout.usd，之前泛红的 electriccooker/faucet 应该正常渲染
```

---

## 5. 方案对比

| | 方案 A：改 MDL 导入方式（本方案） | 方案 B：carb.settings 注册搜索路径（现有方案） |
|---|---|---|
| **修改范围** | 1,567 个 MDL 文件（数据集侧） | 2 个代码文件 + 1 个 shell 脚本（管道侧） |
| **运行时配置** | 无需任何配置 | 每个消费者都要配 `additionalSystemPaths` 或 `MDL_SYSTEM_PATH` |
| **Isaac Sim GUI** | 直接打开即正常 | 需要先 `export MDL_SYSTEM_PATH=...` |
| **数据集更新** | 更新后需重新执行 sed | 不受影响 |
| **侵入性** | 修改共享数据集文件 | 零侵入 |
| **适用角色** | 数据集维护者 | 数据集使用者 |
| **可回退性** | git revert 或重新拉取原始数据 | 删除代码即可 |

**推荐**：
- 如果你是 GRScenes-test0 数据集的维护者 → **方案 A**（一次修改，永久生效）
- 如果你只是数据集的消费者 → **方案 B**（不动数据集，在管道侧配置）
- **两者可以并存**：执行方案 A 后，方案 B 的 carb.settings 配置仍然保留作为兜底

---

## 6. 风险评估

| 风险 | 影响 | 缓解措施 |
|------|------|---------|
| MDL 编译器不支持 `.::` 语法 | 极低。`.::` 从 MDL 1.3 引入，Isaac Sim 4.x 使用 MDL 1.6+，Num*.mdl 已验证可用 | 先在单个文件上测试 |
| `using import` 与 `import` 语义差异 | 低。`using .::M import S;` 和 `import ::M::S;` 引入的符号完全相同 | body 代码无需修改（已确认） |
| 数据集被其他工具重新生成 | 中。`pbrjson2mdl` 工具可能重新生成绝对导入 | 修改 `pbrjson2mdl` 模板，或在 CI 中加 post-processing |
| 其他项目依赖绝对导入行为 | 低。改为相对导入后功能完全等价，只是查找方式不同 | — |

---

## 参考资料

- [NVIDIA MDL Specification (v1.6.1)](https://mdlhandbook.com/pdf/MDL_spec_1.6.1_16Dec2019.pdf)
- [MDL Path Resolution Changes — Omniverse](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/materials_release-notes/MDL_resolution_changes.html)
- [MDL Search Path — Omniverse](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/mdl_search_path.html)
- [Referencing MDL in OpenUSD — Omniverse](https://docs.omniverse.nvidia.com/usd/latest/technical_reference/referencing_mdl.html)
- [NVIDIA MDL SDK GitHub](https://github.com/NVIDIA/MDL-SDK)
- Carbonite 入门指南（本项目，仓库内未收录）
