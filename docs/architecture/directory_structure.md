# 目录结构

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../clean_data.py
> - ../../set_physics/pxr_utils/data_clean.py
>
> 总索引：../overview/docs_index.md

## 索引
- [源目录（示例）](#源目录示例)
- [目标目录（生成）](#目标目录生成)

## 源目录（示例）
```
home_scenes/
  <scene_id>/
    start_result_fix.usd | start_result_new.usd
    Materials/
      *.mdl
      Textures/
        ...
```

## 目标目录（生成）
```
target/
  Materials/
    Textures/
    *.mdl
  models/
    layout/
      articulated/
        <category>/<model_hash>/instance.usd
      others/
        <category>/<model_hash>/instance.usd
    object/
      articulated/
        <category>/<model_hash>/instance.usd
      others/
        <category>/<model_hash>/instance.usd
  scenes/
    <scene_id>/
      Materials -> ../../Materials  (软链接)
      models    -> ../../models     (软链接)
      start_result_new.usd | start_result_fix.usd
```

软链接创建见 `set_physics/pxr_utils/data_clean.py:535-542`。

