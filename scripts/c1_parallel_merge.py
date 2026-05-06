"""C1 Parallel Merge: combine per-category mappings with conflict detection."""
import json
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple

log = logging.getLogger(__name__)


def load_filtered_mapping(mapping_json: Path) -> Dict[str, str]:
    """Load a filtered_mapping.json and return {old_key: canonical_key}."""
    with open(mapping_json, encoding="utf-8") as f:
        payload = json.load(f)

    if isinstance(payload, dict):
        return payload
    if isinstance(payload, list):
        result = {}
        for pair in payload:
            result[pair["old"]] = pair["canonical"]
        return result
    raise ValueError(f"Unexpected mapping format in {mapping_json}")


def merge_category_mappings(
    category_mappings: List[Tuple[str, Dict[str, str]]]
) -> Tuple[Dict[str, str], List[Dict]]:
    """Merge multiple category-identified mappings into one combined mapping.

    Args:
        category_mappings: List of (category_name, {old_key: canonical_key}) tuples.

    Returns:
        (combined_mapping, conflicts_list)
        conflicts_list is empty if no conflicts. Each conflict entry:
        {"old_key": ..., "existing": {"category": ..., "target": ...},
         "conflicting": {"category": ..., "target": ...}}
    """
    combined: Dict[str, str] = {}
    sources: Dict[str, str] = {}  # old_key -> category that first set it
    conflicts: List[Dict] = []

    for cat, mapping in category_mappings:
        for old_key, new_val in mapping.items():
            if old_key in combined:
                if combined[old_key] == new_val:
                    continue  # same target — benign overlap
                conflicts.append({
                    "old_key": old_key,
                    "existing": {"category": sources[old_key],
                                 "target": combined[old_key]},
                    "conflicting": {"category": cat, "target": new_val},
                })
            else:
                combined[old_key] = new_val
                sources[old_key] = cat

    return combined, conflicts


def discover_category_mappings(
    c1_bulk_dir: Path,
    bbox_policy: str,
    out_version: str,
    categories: List[str],
) -> List[Tuple[str, Dict[str, str]]]:
    """Discover and load filtered_mapping.json for each category.

    Returns list of (category_name, mapping_dict).
    Skips categories where filtered_mapping.json is missing or unreadable.
    """
    results: List[Tuple[str, Dict[str, str]]] = []
    for cat in categories:
        mapping_path = (
            c1_bulk_dir
            / f"{cat}_{bbox_policy}_{out_version}"
            / "01_cert"
            / "filtered_mapping.json"
        )
        if not mapping_path.exists():
            log.warning("Mapping not found for category %s: %s", cat, mapping_path)
            continue
        try:
            mapping = load_filtered_mapping(mapping_path)
            if mapping:
                results.append((cat, mapping))
            else:
                log.info("Empty mapping for category %s, skipping", cat)
        except Exception as e:
            log.error("Failed to load mapping for %s: %s", cat, e)
    return results


def gate_check_phase1(
    c1_bulk_dir: Path,
    bbox_policy: str,
    out_version: str,
    categories: List[str],
) -> Tuple[bool, List[str]]:
    """Verify all categories have completed Phase 1 with status=ok.

    Returns (all_passed, failed_categories).
    """
    failed: List[str] = []
    for cat in categories:
        done_file = (
            c1_bulk_dir
            / f"{cat}_{bbox_policy}_{out_version}"
            / "phase1_done.json"
        )
        if not done_file.exists():
            failed.append(f"{cat}: missing phase1_done.json")
            continue
        try:
            payload = json.loads(done_file.read_text(encoding="utf-8"))
        except Exception as e:
            failed.append(f"{cat}: unreadable phase1_done.json ({e})")
            continue
        status = payload.get("status", "")
        audit_passed = payload.get("audit_passed", False)
        if status != "ok":
            failed.append(f"{cat}: status={status}")
        elif not audit_passed:
            failed.append(f"{cat}: audit not passed")
    return len(failed) == 0, failed
