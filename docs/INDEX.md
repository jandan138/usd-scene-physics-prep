# Project Documentation Index

> Generated at: 2026-03-05

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

