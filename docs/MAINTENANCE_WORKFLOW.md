---
title: 项目维护与更新流程 (Maintenance Workflow)
code_reference: scripts/doc_manager.py
created_at: '2025-12-28'
updated_at: '2025-12-28'
maintainer: Codex
status: Active
---

# 项目维护与更新流程 (Maintenance Workflow)

本文档定义了本项目在代码与文档更新时的标准化流程，旨在确保文档系统的一致性与时效性。

> 核心原则：**代码变更驱动文档更新，文档变更驱动索引更新。**

---

## 1. 工具准备

本项目提供 `scripts/doc_manager.py` 辅助维护工作。请确保已安装依赖：
```bash
pip install pyyaml
```

---

## 2. 场景 A：我更新了代码 (Code Update)

当你修改了核心逻辑代码（如 `simready.py` 或 `clean_data.py`）时，请遵循以下步骤：

### 步骤 1：查询关联文档
使用工具反向查找哪些文档引用了该代码：
```bash
# 示例：查找引用了 simready.py 的文档
python scripts/doc_manager.py --find-refs simready.py
```
*输出示例*：
```text
Found 2 documents referencing 'simready.py':
- usage/simready.md (SimReady CLI Usage)
- modules/simready.md (SimReady Module)
```

### 步骤 2：更新文档内容
打开上述找到的文档：
1.  检查技术细节是否需要同步更新（如参数变更、逻辑调整）。
2.  更新文档头部的 `updated_at` 字段为当天日期。

### 步骤 3：验证一致性
在提交前，运行校验命令确保文档格式正确：
```bash
python scripts/doc_manager.py --validate
```

---

## 3. 场景 B：我更新了文档 (Doc Update)

当你新增或修改了 `.md` 文件时：

### 步骤 1：完善元数据 (Metadata)
确保文档头部包含符合规范的 YAML 信息（新增文档必须添加）：
```yaml
---
title: 文档标题
code_reference: 相关代码路径 (如 set_physics/simready.py)
created_at: YYYY-MM-DD
updated_at: YYYY-MM-DD
maintainer: 维护者/团队
status: Active
---
```

### 步骤 2：更新全局索引
运行以下命令，工具会自动扫描所有文档并重新生成 `docs/INDEX.md`：
```bash
python scripts/doc_manager.py --gen-index
```

---

## 4. 常见问题 (FAQ)

**Q: 为什么找不到我的文档？**
A: 请检查文档是否包含了正确的 YAML 头部。`doc_manager.py` 只会索引包含有效元数据的文档。

**Q: `code_reference` 应该填什么？**
A: 填写该文档最核心关联的代码文件路径（相对于项目根目录）。如果是概念性文档，可留空或填 `-`。

**Q: 我应该把这个流程加到 CI 吗？**
A: 强烈建议。可以在 PR Check 中运行 `python scripts/doc_manager.py --validate`，确保所有新提交的文档都符合规范。
