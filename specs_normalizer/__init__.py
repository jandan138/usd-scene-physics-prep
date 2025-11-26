"""
specs_normalizer 包

整体介绍：
- 本包用于将项目在 `target/` 下的原始输出结构，导出为规范化的数据集结构（Materials / Assets / Scenes）。
- 仅执行复制与重排，不修改 USD 内容、不创建软链接，保持主处理流程不变。
"""

