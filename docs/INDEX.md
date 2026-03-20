# Project Documentation Index

> Generated at: 2026-03-14

## Overview

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [Project Overview](overview/project_overview.md) | Active | 2025-12-22 | [`clean_data.py`](clean_data.py) |

## Architecture

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [Codex Agent Team Playbook](architecture/codex_agent_team_playbook.md) | active | 2026-03-12 | [`CODEX.md`](CODEX.md) |
| [Directory Structure](architecture/directory_structure.md) | Active | 2025-12-22 | [`data_clean.py`](set_physics/pxr_utils/data_clean.py) |
| [Processing Pipeline](architecture/pipeline.md) | Active | 2025-12-22 | [`clean_data.py`](clean_data.py) |
| [项目深度分析报告](architecture/project_deep_analysis.md) | Active | 2025-12-28 | [`preprocess_for_interaction.py`](set_physics/preprocess_for_interaction.py), [`data_clean.py`](set_physics/pxr_utils/data_clean.py) |

## Usage

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [Asset Transform Normalization (Recenter + Y-up to Z-up)](usage/normalize_asset_transforms.md) | Active | 2026-03-05 | [`normalize_asset_transforms.py`](scripts/normalize_asset_transforms.py) |
| [Interaction Preprocessing Usage](usage/interaction_preprocessing.md) | Active | 2025-12-22 | [`preprocess_for_interaction.py`](set_physics/preprocess_for_interaction.py) |
| [Navigation Preprocessing Usage](usage/navigation_preprocessing.md) | Active | 2025-12-22 | [`preprocess_for_navigation.py`](set_physics/preprocess_for_navigation.py) |
| [Quickstart Guide](usage/quickstart.md) | Active | 2025-12-22 | [`clean_data.py`](clean_data.py) |
| [Rebuild GRScenes-test0 From Legacy GRScenes](usage/rebuild_test0_from_legacy.md) | Active | 2026-03-14 | [`rebuild_test0_from_legacy.py`](scripts/rebuild_test0_from_legacy.py), [`run_task.sh`](scripts/dlc/run_task.sh), [`normalize.py`](specs_normalizer/normalize.py) |
| [Scene Export Usage](usage/export_scenes.md) | Active | 2025-12-22 | [`export_scene.py`](set_physics/export_scene.py) |
| [Scene 子集导出（按场景 UID 打包场景 + 所有引用资产）](usage/scene_subset_package.md) | Active | 2026-01-19 | [`build_scene_uid_subset_package.py`](scripts/build_scene_uid_subset_package.py) |
| [SimReady CLI Usage](usage/simready.md) | Active | 2025-12-22 | [`simready.py`](set_physics/simready.py) |
| [UID 子集导出（只打包指定资产）](usage/uid_subset_package.md) | Active | 2026-01-20 | [`build_uid_subset_package.py`](scripts/build_uid_subset_package.py) |
| [`/root` 场景交互 sim-ready：`prep_interaction_root_scene.py`（用法）](usage/prep_interaction_root_scene.md) | Active | 2025-12-22 | [`prep_interaction_root_scene.py`](scripts/prep_interaction_root_scene.py), [`isaac_python.sh`](scripts/isaac_python.sh), [`list_draggable_prims.py`](scripts/list_draggable_prims.py), [`oneoff_force_draggable.py`](scripts/oneoff_force_draggable.py), [`oneoff_make_static_collider_only.py`](scripts/oneoff_make_static_collider_only.py) |
| [layout.usd → layout.json：流程原理 + 代码解析（非常详细）](usage/layout_usd_to_layout_json_deep_dive.md) | Active | 2026-01-20 | [`generate_layout_json_from_usd.py`](scripts/generate_layout_json_from_usd.py) |
| [layout.usd 资产引用归一 + transform 补偿（Step 3B 工具）](usage/rewrite_layout_asset_refs_with_compensation.md) | Active | 2026-01-28 | [`rewrite_layout_asset_refs_with_compensation.py`](scripts/rewrite_layout_asset_refs_with_compensation.py) |
| [为只有 USD 的资产目录生成 GLB（按目录标准输出）](usage/usd_to_glb_in_subset.md) | Active | 2026-01-20 | [`convert_subset_usd_to_glb.py`](scripts/convert_subset_usd_to_glb.py) |
| [从 layout.usd 生成 layout.json，并在 Blender 中加载（基于 GLB 资产）](usage/layout_json_for_blender.md) | Active | 2026-01-20 | [`convert_subset_usd_to_glb.py`](scripts/convert_subset_usd_to_glb.py), [`generate_layout_json_from_usd.py`](scripts/generate_layout_json_from_usd.py) |

## Modules

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [Data Clean Module](modules/data_clean.md) | Active | 2025-12-22 | [`data_clean.py`](set_physics/pxr_utils/data_clean.py) |
| [Interaction Preprocessing Module](modules/preprocess_interaction.md) | Active | 2025-12-22 | [`preprocess_for_interaction.py`](set_physics/preprocess_for_interaction.py) |
| [Navigation Preprocessing Module](modules/preprocess_navigation.md) | Active | 2025-12-22 | [`preprocess_for_navigation.py`](set_physics/preprocess_for_navigation.py) |
| [SimReady Module](modules/simready.md) | Active | 2025-12-22 | [`simready.py`](set_physics/simready.py) |
| [USD Physics Module](modules/usd_physics.md) | Active | 2025-12-22 | [`usd_physics.py`](set_physics/pxr_utils/usd_physics.py) |
| [`prep_interaction_root_scene.py`（原理与代码导读）](modules/prep_interaction_root_scene.md) | Active | 2025-12-22 | [`prep_interaction_root_scene.py`](scripts/prep_interaction_root_scene.py), [`list_draggable_prims.py`](scripts/list_draggable_prims.py), [`oneoff_force_draggable.py`](scripts/oneoff_force_draggable.py), [`preprocess_for_interaction.py`](set_physics/preprocess_for_interaction.py) |
| [物理原理与实现解析（PhysX / Isaac Sim / USD）](modules/physics_principles.md) | Active | 2025-12-28 | [`preprocess_for_interaction.py`](set_physics/preprocess_for_interaction.py), [`preprocess_for_navigation.py`](set_physics/preprocess_for_navigation.py), [`usd_physics.py`](set_physics/pxr_utils/usd_physics.py) |

## Specs

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [导出统一结构对齐报告（阶段一至三）](specs/export_specs_unified_alignment.md) | Active | 2026-01-13 | [`check_phase2_assets.py`](scripts/check_phase2_assets.py), [`check_phase3_scenes.py`](scripts/check_phase3_scenes.py), [`fix_mdl_textures_case.py`](scripts/fix_mdl_textures_case.py), [`normalize.py`](specs_normalizer/normalize.py) |
| [操作记录（Materials / Assets / Scenes）](specs/operations_log.md) | Active | 2026-01-13 | [`normalize.py`](specs_normalizer/normalize.py), [`check_phase2_assets.py`](scripts/check_phase2_assets.py), [`check_phase3_scenes.py`](scripts/check_phase3_scenes.py), [`fix_mdl_textures_case.py`](scripts/fix_mdl_textures_case.py) |
| [数据目录结构规范解读（Materials / Assets / Scenes）](specs/dataset_structure_interpretation.md) | Active | 2026-01-13 | [`data_clean.py`](set_physics/pxr_utils/data_clean.py), [`preprocess_for_interaction.py`](set_physics/preprocess_for_interaction.py), [`preprocess_for_navigation.py`](set_physics/preprocess_for_navigation.py), [`thumb_img.py`](set_physics/tools/thumb_img.py) |
| [目录结构对比：原始输出 vs 规范结构](specs/structure_comparison.md) | Active | 2026-01-13 | [`data_clean.py`](set_physics/pxr_utils/data_clean.py), [`preprocess_for_interaction.py`](set_physics/preprocess_for_interaction.py), [`preprocess_for_navigation.py`](set_physics/preprocess_for_navigation.py), [`normalize.py`](specs_normalizer/normalize.py) |
| [统一导出流水线（三阶段说明与预期）](specs/pipeline_phases_overview.md) | Active | 2026-01-14 | [`normalize.py`](specs_normalizer/normalize.py), [`materials.py`](specs_normalizer/exporters/materials.py), [`assets.py`](specs_normalizer/exporters/assets.py), [`scenes.py`](specs_normalizer/exporters/scenes.py) |
| [规范化导出工具使用说明（specs_normalizer）](specs/normalizer_usage.md) | Active | 2026-01-13 | [`__main__.py`](specs_normalizer/__main__.py), [`normalize.py`](specs_normalizer/normalize.py), [`materials.py`](specs_normalizer/exporters/materials.py), [`assets.py`](specs_normalizer/exporters/assets.py), [`scenes.py`](specs_normalizer/exporters/scenes.py), [`structure.py`](specs_normalizer/validators/structure.py) |

## Operations

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [Anomalous Asset Mesh Investigation: test0 vs normalized](operations/anomalous_assets_mesh_investigation.md) | completed | 2026-03-12 | [`normalize_asset_transforms.py`](scripts/normalize_asset_transforms.py) |
| [Bottle 去重失败深度调查报告](operations/bottle_dedup_investigation_report.md) | active | 2026-03-11 | [`report_asset_mesh_dedup.py`](scripts/report_asset_mesh_dedup.py) |
| [C1 规模化执行与审核流程（可控批处理版）](operations/asset_dedup_c1_scaling_workflow.md) | Active | 2026-01-31 | [`c1_build_bulk_mapping_from_dedup_report.py`](scripts/c1_build_bulk_mapping_from_dedup_report.py), [`c1_bulk_apply_layout_dedup.py`](scripts/c1_bulk_apply_layout_dedup.py), [`c1_bulk_step6_category_promote_scan_soft_delete.py`](scripts/c1_bulk_step6_category_promote_scan_soft_delete.py) |
| [Dedup V3 Full-Run Execution: topo_filesize + 3-Way Union Merge + C1 Soft-Delete](operations/dedup_v3_fullrun_execution.md) | completed | 2026-03-12 | [`union_dedup_reports.py`](scripts/union_dedup_reports.py) |
| [Dedup V3: Topology-Invariant + File-Size Mode Implementation Plan](operations/dedup_v3_topo_filesize_plan.md) | implemented | 2026-03-11 | [`union_dedup_reports.py`](scripts/report_asset_mesh_dedup.py, scripts/union_dedup_reports.py) |
| [Dedup V3: topo_filesize Mode Implementation](operations/dedup_v3_implementation.md) | implemented | 2026-03-11 | [`dedup_by_category.py`](scripts/report_asset_mesh_dedup.py, scripts/union_dedup_reports.py, scripts/dlc/dedup_by_category.py) |
| [Design: MDL Path Fix (absolute → relative) for Normalized Dataset](operations/mdl_path_fix_design.md) | approved | 2026-03-10 | [`fix_normalized_mdl_paths.py`](specs_normalizer/utils/mdl_rewrite.py, scripts/fix_normalized_mdl_paths.py) |
| [Fix: C1 Transitive Canonical Conflict (Abs/Rel Path Mismatch)](operations/c1_transitive_canonical_fix.md) | completed | 2026-03-11 | [`union_dedup_reports.py`](scripts/union_dedup_reports.py) |
| [GRScenes OSS Rclone Runbook](operations/grscenes_oss_rclone_runbook.md) | Active | 2026-03-14 | [`SKILL.md`](skills/oss-rclone-ops/SKILL.md) |
| [GRScenes 资产 Mesh 去重报告（仅分析，不修改数据）](operations/asset_mesh_dedup_report.md) | Active | 2026-01-26 | [`report_asset_mesh_dedup.py`](scripts/report_asset_mesh_dedup.py), [`summarize_asset_mesh_dedup_report.py`](scripts/summarize_asset_mesh_dedup_report.py) |
| [GRScenes-test0 Full Dedup Handoff Runbook](operations/grscenes_test0_full_dedup_handoff_runbook.md) | active | 2026-03-12 | [`dedup_by_category.py`](scripts/dlc/dedup_by_category.py), [`union_dedup_reports.py`](scripts/union_dedup_reports.py), [`c1_build_bulk_mapping_from_dedup_report.py`](scripts/c1_build_bulk_mapping_from_dedup_report.py), [`c1_autorun_categories.py`](scripts/c1_autorun_categories.py), [`c1_bulk_step6_category_promote_scan_soft_delete.py`](scripts/c1_bulk_step6_category_promote_scan_soft_delete.py) |
| [GRScenes-test1 类别别名合并操作指南（维护向）](operations/grscenes_test1_category_merge.md) | Active | 2026-01-23 | [`merge_asset_categories_test1.py`](scripts/merge_asset_categories_test1.py) |
| [GRScenes-test1 资产去重评估与行动方案](operations/grscenes_test1_dedup_assessment.md) | active | 2026-03-08 | [`normalize_asset_transforms.py`](scripts/report_asset_mesh_dedup.py, scripts/normalize_asset_transforms.py) |
| [Isaac Sim / Omniverse Kit 的 MDL 渲染与路径配置（通俗版）](operations/isaacsim_mdl_workflow.md) | Active | 2026-01-23 | [`isaac_python.sh`](scripts/isaac_python.sh), [`fix_usd_paths_api.py`](scripts/fix_usd_paths_api.py) |
| [MDL Path Fix: Absolute to Relative Path Rewriting for Normalized Dataset](operations/mdl_path_fix.md) | active | 2026-03-10 | [`mdl_rewrite.py`](scripts/fix_normalized_mdl_paths.py, specs_normalizer/utils/mdl_rewrite.py) |
| [Matrix Multiply Order Bug Fix in normalize_asset_transforms.py](operations/matrix_multiply_bug_fix.md) | active | 2026-03-10 | [`normalize_asset_transforms.py`](scripts/normalize_asset_transforms.py) |
| [Phase 2: Normalized 全量去重扫描与 C1 执行](operations/normalized_dedup_phase2_execution.md) | active | 2026-03-09 | [`c1_autorun_categories.py`](scripts/c1_autorun_categories.py) |
| [Placement Investigation V2: Dedup Compensation Bug (test0 Baseline)](operations/placement_investigation_v2_report.md) | confirmed | 2026-03-12 | [`rewrite_layout_asset_refs_with_compensation.py`](scripts/rewrite_layout_asset_refs_with_compensation.py) |
| [Placement Investigation: Normalized Layout vs Original Layout](operations/placement_investigation_report.md) | completed | 2026-03-12 | [`normalize_asset_transforms.py`](scripts/normalize_asset_transforms.py) |
| [Plan C Execution: Patch Dedup Placement Displacement](operations/placement_fix_plan_c_execution.md) | approved | 2026-03-12 | [`rewrite_layout_asset_refs_with_compensation.py`](scripts/rewrite_layout_asset_refs_with_compensation.py) |
| [Plan C Placement Patch: Execution Results & Root Cause Reframe](operations/placement_fix_plan_c_results.md) | complete | 2026-03-12 | [`verify_all_scenes_vs_test0.py`](scripts/patch_dedup_placement.py, scripts/verify_all_scenes_vs_test0.py) |
| [Re-normalization V2 Pipeline Execution](operations/renormalization_v2_execution.md) | active | 2026-03-11 | [`normalize_asset_transforms.py`](scripts/normalize_asset_transforms.py) |
| [Scale-Invariant Dedup Proposal: --mode shape_invariant](operations/dedup_scale_invariant_proposal.md) | draft | 2026-03-11 | [`report_asset_mesh_dedup.py`](scripts/report_asset_mesh_dedup.py) |
| [Shape-Invariant Dedup Mode Implementation](operations/shape_invariant_implementation.md) | implemented | 2026-03-11 | [`report_asset_mesh_dedup.py`](scripts/report_asset_mesh_dedup.py) |
| [Shape-Invariant Dedup Mode Proposal](operations/dedup_shape_invariant_proposal.md) | implemented | 2026-03-11 | [`report_asset_mesh_dedup.py`](scripts/report_asset_mesh_dedup.py) |
| [Shape-Invariant Full-Run Execution: DLC Scan + Union Merge + C1 Soft-Delete](operations/shape_invariant_fullrun_execution.md) | in_progress | 2026-03-11 | [`report_asset_mesh_dedup.py`](scripts/report_asset_mesh_dedup.py) |
| [SimBench（GRSceneUSD task9/task10）交互预处理：实战踩坑汇总](operations/simbench_interaction_preprocess_field_notes.md) | Active | 2025-12-23 | [`isaac_python.sh`](scripts/isaac_python.sh), [`prep_interaction_root_scene.py`](scripts/prep_interaction_root_scene.py), [`list_draggable_prims.py`](scripts/list_draggable_prims.py), [`oneoff_force_draggable.py`](scripts/oneoff_force_draggable.py), [`oneoff_make_static_collider_only.py`](scripts/oneoff_make_static_collider_only.py), [`inspect_usd_physics_props.py`](scripts/inspect_usd_physics_props.py), [`check_usd_external_assets.py`](scripts/check_usd_external_assets.py), [`oneoff_fix_mass_invalid_values.py`](scripts/oneoff_fix_mass_invalid_values.py), [`oneoff_bind_physics_material.py`](scripts/oneoff_bind_physics_material.py), [`oneoff_add_proxy_box_collider.py`](scripts/oneoff_add_proxy_box_collider.py), [`oneoff_stabilize_contact_ccd_damping.py`](scripts/oneoff_stabilize_contact_ccd_damping.py), [`oneoff_add_spoon_multi_box_proxy.py`](scripts/oneoff_add_spoon_multi_box_proxy.py), [`simready.py`](set_physics/simready.py), [`preprocess_for_interaction.py`](set_physics/preprocess_for_interaction.py) |
| [Windows 使用注意](operations/windows_notes.md) | Active | 2025-12-22 | [`data_clean.py`](set_physics/pxr_utils/data_clean.py), [`usd_physics.py`](set_physics/pxr_utils/usd_physics.py) |
| [`/root` 交互预处理：自检与排错清单（`prep_interaction_root_scene.py`）](operations/prep_interaction_root_scene_checklist.md) | Active | 2025-12-22 | [`prep_interaction_root_scene.py`](scripts/prep_interaction_root_scene.py), [`list_draggable_prims.py`](scripts/list_draggable_prims.py), [`isaac_python.sh`](scripts/isaac_python.sh) |
| [`report_asset_mesh_dedup.py` 代码导读（中文）](operations/asset_mesh_dedup_code_guide.md) | Active | 2026-01-26 | [`report_asset_mesh_dedup.py`](scripts/report_asset_mesh_dedup.py), [`report_asset_mesh_dedup_zh_annotated.py`](scripts/report_asset_mesh_dedup_zh_annotated.py) |
| [交互预处理排错（SimBench/GRSceneUSD task10）](operations/troubleshooting_interaction_preprocess.md) | Active | 2025-12-23 | [`simready.py`](set_physics/simready.py), [`preprocess_for_interaction.py`](set_physics/preprocess_for_interaction.py), [`isaac_python.sh`](scripts/isaac_python.sh), [`prep_interaction_root_scene.py`](scripts/prep_interaction_root_scene.py), [`list_draggable_prims.py`](scripts/list_draggable_prims.py), [`oneoff_make_static_collider_only.py`](scripts/oneoff_make_static_collider_only.py), [`inspect_usd_physics_props.py`](scripts/inspect_usd_physics_props.py), [`oneoff_stabilize_contact_ccd_damping.py`](scripts/oneoff_stabilize_contact_ccd_damping.py) |
| [去重量化桶边界问题分析与修复](operations/dedup_quantize_boundary_analysis.md) | active | 2026-03-08 | [`report_asset_mesh_dedup.py`](scripts/report_asset_mesh_dedup.py) |
| [排错：GLB payload 多 Mesh（`geometry_01` 等）未绑定 collider 导致穿透](operations/troubleshooting_glb_payload_multimesh.md) | Active | 2025-12-22 | [`prep_interaction_root_scene.py`](scripts/prep_interaction_root_scene.py), [`list_draggable_prims.py`](scripts/list_draggable_prims.py), [`oneoff_force_draggable.py`](scripts/oneoff_force_draggable.py) |
| [环境准备建议](operations/environment_setup.md) | Active | 2025-12-22 | [`simready.py`](set_physics/simready.py), [`preprocess_for_interaction.py`](set_physics/preprocess_for_interaction.py), [`preprocess_for_navigation.py`](set_physics/preprocess_for_navigation.py) |
| [资产去重落地方案 C1 执行手册（Scene 侧引用归一 + Instancing）](operations/asset_dedup_c1_scene_instancing_runbook.md) | Active | 2026-01-28 | [`rewrite_layout_asset_refs_with_compensation.py`](scripts/rewrite_layout_asset_refs_with_compensation.py), [`c1_build_bulk_mapping_from_dedup_report.py`](scripts/c1_build_bulk_mapping_from_dedup_report.py), [`c1_bulk_apply_layout_dedup.py`](scripts/c1_bulk_apply_layout_dedup.py) |

## References

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [Assets and Materials Reference](references/assets_and_materials.md) | Active | 2025-12-22 | [`data_clean.py`](set_physics/pxr_utils/data_clean.py) |
| [Dependencies and Environment](references/dependencies.md) | Active | 2025-12-22 | [`requirements.txt`](requirements.txt) |

## Others

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [2025-12-20：新增一键生成 Sim-Ready USD 的 CLI](changes/2025-12-20_simready_cli.md) | completed | 2025-12-28 | [`simready.py`](set_physics/simready.py) |
| [2026-01-20 — Generate layout.json for Blender (scene subset)](changes/2026-01-20_layout_json_blender.md) | completed | 2026-01-20 | [`generate_layout_json_from_usd.py`](scripts/generate_layout_json_from_usd.py) |
| [2026-01-23 — GRScenes-test1 类别别名合并（coffeemaker/sofachair/tvstand）](changes/2026-01-23_test1_category_alias_merge.md) | completed | 2026-01-23 | [`merge_asset_categories_test1.py`](scripts/merge_asset_categories_test1.py) |
| [2026-02-09 — GRScenes-test1: 目录别名统一、door_* 缺失引用清理、Material/mdl 缺失排查](changes/2026-02-09_test1_missing_cleanup.md) | completed | 2026-02-09 | [`oneoff_clear_missing_door_references.py`](scripts/oneoff_clear_missing_door_references.py), [`oneoff_fix_missing_mdl_assets.py`](scripts/oneoff_fix_missing_mdl_assets.py) |
| [Claude Code Agent Team 系统设计与复现手册](agent-team-playbook.md) | Active | 2026-03-03 | [`codex_agent_team_playbook.md`](docs/architecture/codex_agent_team_playbook.md) |
| [Commercial Scene Export Fixes (2026-01-09)](changes/2026-01-09_commercial_scene_fixes.md) | completed | 2026-01-09 | [`scenes.py`](specs_normalizer/exporters/scenes.py), [`fix_usd_paths_api.py`](scripts/fix_usd_paths_api.py) |
| [DLC Normalize Mode Failure Analysis / DLC Normalize 模式故障分析](dlc/debug_normalize_failure.md) | Active | 2026-03-04 | [`run_task.sh`](scripts/dlc/run_task.sh), [`mdl_rewrite.py`](specs_normalizer/utils/mdl_rewrite.py), [`scenes.py`](specs_normalizer/exporters/scenes.py) |
| [GRScenes-test0 Normalize-Only Continuation Execution Plan](test0_full/grscenes_test0_normalize_only_continuation_execution_plan_20260314.md) | active | 2026-03-14 | [`normalize_asset_transforms.py`](scripts/normalize_asset_transforms.py), [`create_pre_c1_normalize_only_snapshots.py`](scripts/create_pre_c1_normalize_only_snapshots.py), [`placement_pairwise_compare.py`](scripts/placement_pairwise_compare.py), [`audit_normalize_phase2.py`](scripts/audit_normalize_phase2.py), [`assemble_normalize_gate_bundle.py`](scripts/assemble_normalize_gate_bundle.py) |
| [GRScenes-test0 Normalize-Only Full Runbook](test0_full/grscenes_test0_normalize_only_runbook.md) | active | 2026-03-12 | [`normalize_asset_transforms.py`](scripts/normalize_asset_transforms.py), [`placement_pairwise_compare.py`](scripts/placement_pairwise_compare.py), [`audit_normalize_phase2.py`](scripts/audit_normalize_phase2.py), [`check_normalize_gate_from_reports.py`](scripts/check_normalize_gate_from_reports.py), [`build_normalize_gate_verdict.py`](scripts/build_normalize_gate_verdict.py) |
| [GRScenes-test0 Phase1 Status And Phase2 Probe Plan](test0_full/grscenes_test0_phase1_status_and_phase2_probe_plan_20260314.md) | active | 2026-03-14 | [`normalize_asset_transforms.py`](scripts/normalize_asset_transforms.py), [`assemble_normalize_gate_bundle.py`](scripts/assemble_normalize_gate_bundle.py), [`placement_pairwise_compare.py`](scripts/placement_pairwise_compare.py), [`audit_normalize_phase2.py`](scripts/audit_normalize_phase2.py) |
| [Rebuilt GRScenes Temp Outputs Investigation](tmp/rebuilt_grscenes_layout_empty_investigation_20260314.md) | completed | 2026-03-14 07:20:28+00:00 | [`GRScenes-test0-rebuilt_home_tmp`](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp), [`GRScenes-test0-rebuilt_commercial_tmp`](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp) |
| [Scene UID Downloader (scene + all referenced assets)](scripts/scene_asset_downloader.md) | Active | 2026-01-19 | [`build_scene_uid_subset_package.py`](scripts/build_scene_uid_subset_package.py) |
| [Specs Normalizer Enhancements and Fixes (2026-01-08)](changes/2026-01-08_specs_normalizer_fixes.md) | completed | 2026-01-08 | [`normalize.py`](specs_normalizer/normalize.py), [`fix_usd_textures_case.py`](scripts/fix_usd_textures_case.py) |
| [Specs Normalizer Guide](specs_normalizer/specs_normalizer_guide.md) | Active | 2026-01-13 | [`normalize.py`](specs_normalizer/normalize.py) |
| [Test0 Smoke Baseline And S1 Subset Status](test0_smoke/20260312_074731/run_baseline_and_subset.md) | active | 2026-03-12 | [`baseline_brief.json`](check_reports/test0_smoke/20260312_074731/baseline/baseline_brief.json), [`subset_s1_status.json`](check_reports/test0_smoke/20260312_074731/summary/subset_s1_status.json) |
| [UID Asset Downloader (build subset package)](scripts/uid_asset_downloader.md) | Active | 2026-01-19 | [`build_uid_subset_package.py`](scripts/build_uid_subset_package.py) |
| [UID 资产下载脚本（按 UID 构建子集包）](scripts/uid_asset_downloader.zh-CN.md) | Active | 2026-01-19 | [`build_uid_subset_package.py`](scripts/build_uid_subset_package.py) |
| [Worker 2 S1 Subset Build](test0_smoke/20260312_074731/worker-2-subset-s1.md) | completed | 2026-03-12 | [`build_scene_uid_subset_package.py`](scripts/build_scene_uid_subset_package.py) |
| [Worker-4 Normalize Command Plan for S1 Smoke](test0_smoke/20260312_074731/worker-4-normalize-plan.md) | done | 2026-03-12T07:49:55Z | [`normalize_asset_transforms.py`](scripts/normalize_asset_transforms.py), [`build_scene_uid_subset_package.py`](scripts/build_scene_uid_subset_package.py) |
| [specs_normalizer Scene Ref Rewrite Investigation](tmp/specs_normalizer_scene_ref_rewrite_investigation_20260314.md) | completed | 2026-03-14 08:06:00+00:00 | [`scenes.py`](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/specs_normalizer/exporters/scenes.py), [`scene_rewrite.py`](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/specs_normalizer/utils/scene_rewrite.py), [`normalize.py`](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/specs_normalizer/normalize.py), [`run_task.sh`](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/dlc/run_task.sh), [`rebuild_test0_from_legacy.py`](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/rebuild_test0_from_legacy.py) |
| [test0 smoke descendant override remap plan](test0_smoke/20260312_074731/descendant_override_remap_plan.md) | draft | 2026-03-12 | [`normalize_asset_transforms.py`](scripts/normalize_asset_transforms.py), [`audit_normalize_phase2.py`](scripts/audit_normalize_phase2.py), [`check_normalize_gate_from_reports.py`](scripts/check_normalize_gate_from_reports.py) |
| [test0 smoke normalize descendant override research](test0_smoke/20260312_074731/research.md) | completed | 2026-03-12 | [`normalize_asset_transforms.py`](scripts/normalize_asset_transforms.py), [`audit_normalize_phase2.py`](scripts/audit_normalize_phase2.py), [`check_normalize_gate_from_reports.py`](scripts/check_normalize_gate_from_reports.py) |
| [test0 smoke s1 descendant override remap execution draft](test0_smoke/20260312_074731/execution.md) | completed | 2026-03-12 | - |
| [test0 smoke s1 descendant override remap summary draft](test0_smoke/20260312_074731/summary.md) | completed | 2026-03-12 | - |
| [test0 smoke s1 descendant override remap test draft](test0_smoke/20260312_074731/test.md) | completed | 2026-03-12 | - |
| [test0 smoke s1 lead run note](test0_smoke/20260312_074731/lead-run-note.md) | draft | 2026-03-12 | [`normalize_asset_transforms.py`](scripts/normalize_asset_transforms.py) |
| [场景 UID 下载脚本（场景 + 所有引用的资产 + 材质）](scripts/scene_asset_downloader.zh-CN.md) | Active | 2026-01-19 | [`build_scene_uid_subset_package.py`](scripts/build_scene_uid_subset_package.py) |
| [工作流示例](examples/workflow_examples.md) | Active | 2025-12-22 | [`preprocess_for_interaction.py`](set_physics/preprocess_for_interaction.py), [`preprocess_for_navigation.py`](set_physics/preprocess_for_navigation.py), [`get_all_references.py`](set_physics/get_all_references.py), [`export_scene.py`](set_physics/export_scene.py) |
| [常见问题](faq/faq.md) | Active | 2025-12-22 | [`data_clean.py`](set_physics/pxr_utils/data_clean.py), [`preprocess_for_interaction.py`](set_physics/preprocess_for_interaction.py), [`preprocess_for_navigation.py`](set_physics/preprocess_for_navigation.py), [`simready.py`](set_physics/simready.py) |
| [项目维护与更新流程 (Maintenance Workflow)](MAINTENANCE_WORKFLOW.md) | Active | 2025-12-28 | [`doc_manager.py`](scripts/doc_manager.py) |

