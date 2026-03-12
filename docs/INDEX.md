# Project Documentation Index

> Generated at: 2026-03-12

## Overview

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [Project Overview](overview/project_overview.md) | Active | 2025-12-22 | [`clean_data.py`](clean_data.py) |

## Architecture

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [Directory Structure](architecture/directory_structure.md) | Active | 2025-12-22 | [`data_clean.py`](set_physics/pxr_utils/data_clean.py) |
| [Processing Pipeline](architecture/pipeline.md) | Active | 2025-12-22 | [`clean_data.py`](clean_data.py) |
| [project_deep_analysis.md](architecture/project_deep_analysis.md) | Unknown | N/A | - |

## Usage

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [Asset Transform Normalization (Recenter + Y-up to Z-up)](usage/normalize_asset_transforms.md) | Active | 2026-03-05 | [`normalize_asset_transforms.py`](scripts/normalize_asset_transforms.py) |
| [Interaction Preprocessing Usage](usage/interaction_preprocessing.md) | Active | 2025-12-22 | [`preprocess_for_interaction.py`](set_physics/preprocess_for_interaction.py) |
| [Navigation Preprocessing Usage](usage/navigation_preprocessing.md) | Active | 2025-12-22 | [`preprocess_for_navigation.py`](set_physics/preprocess_for_navigation.py) |
| [Quickstart Guide](usage/quickstart.md) | Active | 2025-12-22 | [`clean_data.py`](clean_data.py) |
| [Scene Export Usage](usage/export_scenes.md) | Active | 2025-12-22 | [`export_scene.py`](set_physics/export_scene.py) |
| [SimReady CLI Usage](usage/simready.md) | Active | 2025-12-22 | [`simready.py`](set_physics/simready.py) |
| [layout_json_for_blender.md](usage/layout_json_for_blender.md) | Unknown | N/A | - |
| [layout_usd_to_layout_json_deep_dive.md](usage/layout_usd_to_layout_json_deep_dive.md) | Unknown | N/A | - |
| [prep_interaction_root_scene.md](usage/prep_interaction_root_scene.md) | Unknown | N/A | - |
| [rewrite_layout_asset_refs_with_compensation.md](usage/rewrite_layout_asset_refs_with_compensation.md) | Unknown | N/A | - |
| [scene_subset_package.md](usage/scene_subset_package.md) | Unknown | N/A | - |
| [uid_subset_package.md](usage/uid_subset_package.md) | Unknown | N/A | - |
| [usd_to_glb_in_subset.md](usage/usd_to_glb_in_subset.md) | Unknown | N/A | - |

## Modules

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [Data Clean Module](modules/data_clean.md) | Active | 2025-12-22 | [`data_clean.py`](set_physics/pxr_utils/data_clean.py) |
| [Interaction Preprocessing Module](modules/preprocess_interaction.md) | Active | 2025-12-22 | [`preprocess_for_interaction.py`](set_physics/preprocess_for_interaction.py) |
| [Navigation Preprocessing Module](modules/preprocess_navigation.md) | Active | 2025-12-22 | [`preprocess_for_navigation.py`](set_physics/preprocess_for_navigation.py) |
| [SimReady Module](modules/simready.md) | Active | 2025-12-22 | [`simready.py`](set_physics/simready.py) |
| [USD Physics Module](modules/usd_physics.md) | Active | 2025-12-22 | [`usd_physics.py`](set_physics/pxr_utils/usd_physics.py) |
| [physics_principles.md](modules/physics_principles.md) | Unknown | N/A | - |
| [prep_interaction_root_scene.md](modules/prep_interaction_root_scene.md) | Unknown | N/A | - |

## Specs

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [dataset_structure_interpretation.md](specs/dataset_structure_interpretation.md) | Unknown | N/A | - |
| [export_specs_unified_alignment.md](specs/export_specs_unified_alignment.md) | Unknown | N/A | - |
| [normalizer_usage.md](specs/normalizer_usage.md) | Unknown | N/A | - |
| [operations_log.md](specs/operations_log.md) | Unknown | N/A | - |
| [pipeline_phases_overview.md](specs/pipeline_phases_overview.md) | Unknown | N/A | - |
| [structure_comparison.md](specs/structure_comparison.md) | Unknown | N/A | - |

## Operations

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [Bottle 去重失败深度调查报告](operations/bottle_dedup_investigation_report.md) | active | 2026-03-11 | [`report_asset_mesh_dedup.py`](scripts/report_asset_mesh_dedup.py) |
| [Dedup V3 Full-Run Execution: topo_filesize + 3-Way Union Merge + C1 Soft-Delete](operations/dedup_v3_fullrun_execution.md) | completed | 2026-03-12 | [`union_dedup_reports.py`](scripts/union_dedup_reports.py) |
| [Dedup V3: Topology-Invariant + File-Size Mode Implementation Plan](operations/dedup_v3_topo_filesize_plan.md) | implemented | 2026-03-11 | [`union_dedup_reports.py`](scripts/report_asset_mesh_dedup.py, scripts/union_dedup_reports.py) |
| [Dedup V3: topo_filesize Mode Implementation](operations/dedup_v3_implementation.md) | implemented | 2026-03-11 | [`dedup_by_category.py`](scripts/report_asset_mesh_dedup.py, scripts/union_dedup_reports.py, scripts/dlc/dedup_by_category.py) |
| [Design: MDL Path Fix (absolute → relative) for Normalized Dataset](operations/mdl_path_fix_design.md) | approved | 2026-03-10 | [`fix_normalized_mdl_paths.py`](specs_normalizer/utils/mdl_rewrite.py, scripts/fix_normalized_mdl_paths.py) |
| [Fix: C1 Transitive Canonical Conflict (Abs/Rel Path Mismatch)](operations/c1_transitive_canonical_fix.md) | completed | 2026-03-11 | [`union_dedup_reports.py`](scripts/union_dedup_reports.py) |
| [GRScenes-test1 资产去重评估与行动方案](operations/grscenes_test1_dedup_assessment.md) | active | 2026-03-08 | [`normalize_asset_transforms.py`](scripts/report_asset_mesh_dedup.py, scripts/normalize_asset_transforms.py) |
| [MDL Path Fix: Absolute to Relative Path Rewriting for Normalized Dataset](operations/mdl_path_fix.md) | active | 2026-03-10 | [`mdl_rewrite.py`](scripts/fix_normalized_mdl_paths.py, specs_normalizer/utils/mdl_rewrite.py) |
| [Matrix Multiply Order Bug Fix in normalize_asset_transforms.py](operations/matrix_multiply_bug_fix.md) | active | 2026-03-10 | [`normalize_asset_transforms.py`](scripts/normalize_asset_transforms.py) |
| [Phase 2: Normalized 全量去重扫描与 C1 执行](operations/normalized_dedup_phase2_execution.md) | active | 2026-03-09 | [`c1_autorun_categories.py`](scripts/c1_autorun_categories.py) |
| [Re-normalization V2 Pipeline Execution](operations/renormalization_v2_execution.md) | active | 2026-03-11 | [`normalize_asset_transforms.py`](scripts/normalize_asset_transforms.py) |
| [Scale-Invariant Dedup Proposal: --mode shape_invariant](operations/dedup_scale_invariant_proposal.md) | draft | 2026-03-11 | [`report_asset_mesh_dedup.py`](scripts/report_asset_mesh_dedup.py) |
| [Shape-Invariant Dedup Mode Implementation](operations/shape_invariant_implementation.md) | implemented | 2026-03-11 | [`report_asset_mesh_dedup.py`](scripts/report_asset_mesh_dedup.py) |
| [Shape-Invariant Dedup Mode Proposal](operations/dedup_shape_invariant_proposal.md) | implemented | 2026-03-11 | [`report_asset_mesh_dedup.py`](scripts/report_asset_mesh_dedup.py) |
| [Shape-Invariant Full-Run Execution: DLC Scan + Union Merge + C1 Soft-Delete](operations/shape_invariant_fullrun_execution.md) | in_progress | 2026-03-11 | [`report_asset_mesh_dedup.py`](scripts/report_asset_mesh_dedup.py) |
| [asset_dedup_c1_scaling_workflow.md](operations/asset_dedup_c1_scaling_workflow.md) | Unknown | N/A | - |
| [asset_dedup_c1_scene_instancing_runbook.md](operations/asset_dedup_c1_scene_instancing_runbook.md) | Unknown | N/A | - |
| [asset_mesh_dedup_code_guide.md](operations/asset_mesh_dedup_code_guide.md) | Unknown | N/A | - |
| [asset_mesh_dedup_report.md](operations/asset_mesh_dedup_report.md) | Unknown | N/A | - |
| [environment_setup.md](operations/environment_setup.md) | Unknown | N/A | - |
| [grscenes_test1_category_merge.md](operations/grscenes_test1_category_merge.md) | Unknown | N/A | - |
| [isaacsim_mdl_workflow.md](operations/isaacsim_mdl_workflow.md) | Unknown | N/A | - |
| [prep_interaction_root_scene_checklist.md](operations/prep_interaction_root_scene_checklist.md) | Unknown | N/A | - |
| [simbench_interaction_preprocess_field_notes.md](operations/simbench_interaction_preprocess_field_notes.md) | Unknown | N/A | - |
| [troubleshooting_glb_payload_multimesh.md](operations/troubleshooting_glb_payload_multimesh.md) | Unknown | N/A | - |
| [troubleshooting_interaction_preprocess.md](operations/troubleshooting_interaction_preprocess.md) | Unknown | N/A | - |
| [windows_notes.md](operations/windows_notes.md) | Unknown | N/A | - |
| [去重量化桶边界问题分析与修复](operations/dedup_quantize_boundary_analysis.md) | active | 2026-03-08 | [`report_asset_mesh_dedup.py`](scripts/report_asset_mesh_dedup.py) |

## References

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [Assets and Materials Reference](references/assets_and_materials.md) | Active | 2025-12-22 | [`data_clean.py`](set_physics/pxr_utils/data_clean.py) |
| [Dependencies and Environment](references/dependencies.md) | Active | 2025-12-22 | [`requirements.txt`](requirements.txt) |

## Others

| Document | Status | Last Updated | Related Code |
| :--- | :--- | :--- | :--- |
| [2025-12-20_simready_cli.md](changes/2025-12-20_simready_cli.md) | Unknown | N/A | - |
| [2026-01-08_specs_normalizer_fixes.md](changes/2026-01-08_specs_normalizer_fixes.md) | Unknown | N/A | - |
| [2026-01-09_commercial_scene_fixes.md](changes/2026-01-09_commercial_scene_fixes.md) | Unknown | N/A | - |
| [2026-01-20_layout_json_blender.md](changes/2026-01-20_layout_json_blender.md) | Unknown | N/A | - |
| [2026-01-23_test1_category_alias_merge.md](changes/2026-01-23_test1_category_alias_merge.md) | Unknown | N/A | - |
| [2026-02-09_test1_missing_cleanup.md](changes/2026-02-09_test1_missing_cleanup.md) | Unknown | N/A | - |
| [MAINTENANCE_WORKFLOW.md](MAINTENANCE_WORKFLOW.md) | Unknown | N/A | - |
| [Specs Normalizer Guide](specs_normalizer/specs_normalizer_guide.md) | Active | 2026-01-13 | [`normalize.py`](specs_normalizer/normalize.py) |
| [agent-team-playbook.md](agent-team-playbook.md) | Unknown | N/A | - |
| [debug_normalize_failure.md](dlc/debug_normalize_failure.md) | Unknown | N/A | - |
| [faq.md](faq/faq.md) | Unknown | N/A | - |
| [scene_asset_downloader.md](scripts/scene_asset_downloader.md) | Unknown | N/A | - |
| [scene_asset_downloader.zh-CN.md](scripts/scene_asset_downloader.zh-CN.md) | Unknown | N/A | - |
| [uid_asset_downloader.md](scripts/uid_asset_downloader.md) | Unknown | N/A | - |
| [uid_asset_downloader.zh-CN.md](scripts/uid_asset_downloader.zh-CN.md) | Unknown | N/A | - |
| [workflow_examples.md](examples/workflow_examples.md) | Unknown | N/A | - |

