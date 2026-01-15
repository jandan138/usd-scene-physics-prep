#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Batch convert normalized asset USDs to GLB within a subset package.

Input subset layout (normalized):
  <subset_root>/
    GRScenes_assets/<category>/<uid>/usd/<uid>.usd

Output layout (normalized, optional):
  <subset_root>/
    GRScenes_assets/<category>/<uid>/glb/<uid>.glb

Converter invocation (as provided):
  <ConvertAsset>/scripts/isaac_python.sh <ConvertAsset>/main.py \
    usd-to-glb <input_usd> --out <output_glb>

This script intentionally does NOT use pxr.Usd; it simply drives ConvertAsset.
"""

from __future__ import annotations

import argparse
import concurrent.futures as cf
import json
import os
import shlex
import subprocess
import time
from dataclasses import asdict, dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Set


def _norm_abs(p: str) -> str:
    return os.path.abspath(os.path.expanduser(p))


def _dedup_keep_order(items: Sequence[str]) -> List[str]:
    seen: Set[str] = set()
    out: List[str] = []
    for x in items:
        if x in seen:
            continue
        seen.add(x)
        out.append(x)
    return out


def _read_uid_file(path: str) -> List[str]:
    uids: List[str] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            uids.append(s)
    return _dedup_keep_order(uids)


@dataclass(frozen=True)
class AssetJob:
    category: str
    uid: str
    uid_dir: str
    input_usd: str
    output_glb: str


def _iter_asset_jobs(subset_root: str) -> Iterable[AssetJob]:
    assets_root = os.path.join(subset_root, "GRScenes_assets")
    if not os.path.isdir(assets_root):
        return

    for category in sorted(os.listdir(assets_root)):
        cat_dir = os.path.join(assets_root, category)
        if not os.path.isdir(cat_dir):
            continue

        for uid in sorted(os.listdir(cat_dir)):
            uid_dir = os.path.join(cat_dir, uid)
            if not os.path.isdir(uid_dir):
                continue

            input_usd = os.path.join(uid_dir, "usd", f"{uid}.usd")
            if not os.path.isfile(input_usd):
                continue

            output_glb = os.path.join(uid_dir, "glb", f"{uid}.glb")
            yield AssetJob(
                category=category,
                uid=uid,
                uid_dir=uid_dir,
                input_usd=input_usd,
                output_glb=output_glb,
            )


def _is_up_to_date(input_path: str, output_path: str) -> bool:
    if not os.path.exists(output_path):
        return False
    try:
        return os.path.getmtime(output_path) >= os.path.getmtime(input_path)
    except OSError:
        return False


def _cmd_str(cmd: Sequence[str]) -> str:
    return " ".join(shlex.quote(c) for c in cmd)


def _run_one(
    job: AssetJob,
    *,
    isaac_python_sh: str,
    convert_main_py: str,
    dry_run: bool,
    force: bool,
    extra_args: Sequence[str],
) -> Dict[str, object]:
    start = time.time()

    if not os.path.isfile(job.input_usd):
        return {
            **asdict(job),
            "status": "missing_input",
            "returncode": None,
            "seconds": 0.0,
        }

    if (not force) and _is_up_to_date(job.input_usd, job.output_glb):
        return {
            **asdict(job),
            "status": "skipped_existing",
            "returncode": 0,
            "seconds": 0.0,
        }

    cmd = [
        isaac_python_sh,
        convert_main_py,
        "usd-to-glb",
        job.input_usd,
        "--out",
        job.output_glb,
    ]
    if extra_args:
        cmd.extend(list(extra_args))

    if dry_run:
        return {
            **asdict(job),
            "status": "dry_run",
            "returncode": 0,
            "cmd": cmd,
            "cmd_str": _cmd_str(cmd),
            "seconds": 0.0,
        }

    os.makedirs(os.path.dirname(job.output_glb), exist_ok=True)

    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    ok = (p.returncode == 0) and os.path.isfile(job.output_glb)
    status = "ok" if ok else "failed"

    # Avoid huge JSON: keep last 8KB of converter output.
    out = p.stdout or ""
    tail = out[-8000:] if len(out) > 8000 else out

    return {
        **asdict(job),
        "status": status,
        "returncode": p.returncode,
        "cmd": cmd,
        "cmd_str": _cmd_str(cmd),
        "stdout_tail": tail,
        "seconds": round(time.time() - start, 3),
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--subset-root",
        required=True,
        help="Subset root that contains GRScenes_assets/<category>/<uid>/usd/<uid>.usd",
    )
    ap.add_argument(
        "--convertasset-root",
        default="/cpfs/shared/simulation/zhuzihou/dev/ConvertAsset",
        help="ConvertAsset repo root (contains scripts/isaac_python.sh and main.py)",
    )
    ap.add_argument("--uid", action="append", default=[], help="Only convert these UIDs (repeatable)")
    ap.add_argument("--uid-file", default=None, help="UID list file (one UID per line, supports # comments)")
    ap.add_argument("--jobs", type=int, default=1, help="Parallel jobs (default: 1)")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--force", action="store_true", help="Re-generate even if output is up-to-date")
    ap.add_argument(
        "--quiet",
        action="store_true",
        help="Only print final summary (progress still recorded in --report)",
    )
    ap.add_argument(
        "--extra-args",
        default=None,
        help="Extra args to append to ConvertAsset (string, e.g. \"--foo 1 --bar\")",
    )
    ap.add_argument("--report", default=None, help="Write JSON report path")

    args = ap.parse_args(list(argv) if argv is not None else None)

    subset_root = _norm_abs(args.subset_root)
    ca_root = _norm_abs(args.convertasset_root)

    isaac_python_sh = os.path.join(ca_root, "scripts", "isaac_python.sh")
    convert_main_py = os.path.join(ca_root, "main.py")

    if not os.path.isdir(subset_root):
        print("ERROR: subset-root not found:", subset_root)
        return 2
    if not os.path.isdir(os.path.join(subset_root, "GRScenes_assets")):
        print("ERROR: GRScenes_assets not found under:", subset_root)
        return 2

    if not os.path.isfile(isaac_python_sh):
        print("ERROR: ConvertAsset isaac_python.sh not found:", isaac_python_sh)
        return 2
    if not os.path.isfile(convert_main_py):
        print("ERROR: ConvertAsset main.py not found:", convert_main_py)
        return 2

    uids = list(args.uid or [])
    if args.uid_file:
        uids.extend(_read_uid_file(args.uid_file))
    uids = _dedup_keep_order([u for u in uids if u])
    want_uids = set(uids) if uids else None

    jobs_all = list(_iter_asset_jobs(subset_root))
    if want_uids is not None:
        jobs_all = [j for j in jobs_all if j.uid in want_uids]

    if not jobs_all:
        print("No convertible assets found.")
        return 3

    extra_args: List[str] = []
    if args.extra_args:
        extra_args = shlex.split(args.extra_args)

    max_workers = max(1, int(args.jobs))

    print(
        f"Found {len(jobs_all)} assets. jobs={max_workers} dry_run={bool(args.dry_run)} force={bool(args.force)}",
        flush=True,
    )

    results: List[Dict[str, object]] = []
    counts = {"ok": 0, "failed": 0, "skipped_existing": 0, "missing_input": 0, "dry_run": 0}

    total = len(jobs_all)
    done = 0

    with cf.ThreadPoolExecutor(max_workers=max_workers) as ex:
        futs = [
            ex.submit(
                _run_one,
                j,
                isaac_python_sh=isaac_python_sh,
                convert_main_py=convert_main_py,
                dry_run=bool(args.dry_run),
                force=bool(args.force),
                extra_args=extra_args,
            )
            for j in jobs_all
        ]

        for fut in cf.as_completed(futs):
            r = fut.result()
            results.append(r)
            st = str(r.get("status"))
            if st in counts:
                counts[st] += 1
            done += 1
            if not args.quiet:
                uid = r.get("uid")
                cat = r.get("category")
                secs = r.get("seconds")
                out_glb = r.get("output_glb")
                print(f"[{done:>3}/{total}] {cat}/{uid}  {st}  {secs}s  -> {out_glb}", flush=True)

    results.sort(key=lambda r: (str(r.get("category")), str(r.get("uid"))))

    summary = {
        "subset_root": subset_root,
        "convertasset_root": ca_root,
        "isaac_python_sh": isaac_python_sh,
        "convert_main_py": convert_main_py,
        "uids_filter": uids,
        "jobs": max_workers,
        "dry_run": bool(args.dry_run),
        "force": bool(args.force),
        "extra_args": extra_args,
        "counts": {"total": len(results), **counts},
        "results": results,
    }

    print("=== GLB convert summary ===")
    print(json.dumps(summary["counts"], ensure_ascii=False, indent=2))

    if args.report:
        report_path = _norm_abs(args.report)
        os.makedirs(os.path.dirname(report_path) or ".", exist_ok=True)
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        print("Wrote report:", report_path)

    if counts["failed"] > 0:
        return 4
    if counts["missing_input"] > 0:
        return 5
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
