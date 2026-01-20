# Project Documentation Index

> Generated at: 2026-01-20

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
| [Interaction Preprocessing Usage](usage/interaction_preprocessing.md) | Active | 2025-12-22 | [`preprocess_for_interaction.py`](set_physics/preprocess_for_interaction.py) |
| [Navigation Preprocessing Usage](usage/navigation_preprocessing.md) | Active | 2025-12-22 | [`preprocess_for_navigation.py`](set_physics/preprocess_for_navigation.py) |
| [Quickstart Guide](usage/quickstart.md) | Active | 2025-12-22 | [`clean_data.py`](clean_data.py) |
| [Scene Export Usage](usage/export_scenes.md) | Active | 2025-12-22 | [`export_scene.py`](set_physics/export_scene.py) |
| [UID Subset Package Usage](usage/uid_subset_package.md) | Active | 2026-01-15 | [`build_uid_subset_package.py`](scripts/build_uid_subset_package.py) |
| [Scene Subset Package Usage](usage/scene_subset_package.md) | Active | 2026-01-19 | [`build_scene_uid_subset_package.py`](scripts/build_scene_uid_subset_package.py) |
| [USD to GLB in Subset](usage/usd_to_glb_in_subset.md) | Active | 2026-01-20 | [`convert_subset_usd_to_glb.py`](scripts/convert_subset_usd_to_glb.py) |
| [从 layout.usd 生成 layout.json（Blender）](usage/layout_json_for_blender.md) | Active | 2026-01-20 | [`generate_layout_json_from_usd.py`](scripts/generate_layout_json_from_usd.py) |
| [layout.usd → layout.json 原理与代码解析](usage/layout_usd_to_layout_json_deep_dive.md) | Active | 2026-01-20 | [`generate_layout_json_from_usd.py`](scripts/generate_layout_json_from_usd.py) |
| [Script Guides (External)](scripts/index.md) | Active | 2026-01-19 | - |
| [脚本指南（对外）](scripts/index.md) | Active | 2026-01-19 | - |
| [SimReady CLI Usage](usage/simready.md) | Active | 2025-12-22 | [`simready.py`](set_physics/simready.py) |
| [prep_interaction_root_scene.md](usage/prep_interaction_root_scene.md) | Unknown | N/A | - |

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
| [environment_setup.md](operations/environment_setup.md) | Unknown | N/A | - |
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
| [Specs Normalizer Guide](specs_normalizer/specs_normalizer_guide.md) | Active | 2025-12-22 | [`normalize.py`](specs_normalizer/normalize.py) |
| [faq.md](faq/faq.md) | Unknown | N/A | - |
| [workflow_examples.md](examples/workflow_examples.md) | Unknown | N/A | - |

