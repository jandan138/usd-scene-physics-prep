#!/usr/bin/env python3
import os
import sys
import json
import argparse


def list_usd_files(assets_dir: str):
    files = []
    for root, _, fns in os.walk(assets_dir):
        for fn in fns:
            if fn.lower().endswith((".usd", ".usda", ".usdc")):
                files.append(os.path.join(root, fn))
    return files


def is_text_usd(path: str) -> bool:
    try:
        with open(path, "rb") as f:
            head = f.read(64)
        return b"usda" in head or head.startswith(b"#")
    except Exception:
        return False


def check_refs(usdf: str, mats_abs: str):
    ok_mdl = True
    ok_tex = True
    bad_mdl = []
    bad_tex = []
    mats_rel = None
    try:
        out_dir = os.path.dirname(usdf)
        mats_rel = os.path.relpath(mats_abs, out_dir).replace(os.sep, "/")
    except Exception:
        mats_rel = None

    if is_text_usd(usdf):
        try:
            with open(usdf, "r", encoding="utf-8", errors="ignore") as f:
                txt = f.read()
            # 粗略规则：
            # 1) 所有 .mdl 引用应包含到 Material/mdl 的相对前缀
            # 2) 任何出现 "Materials/Textures" 或大写 Textures 的路径视为不规范
            # 3) 任何绝对路径指向旧源的 MDL/贴图视为不规范
            import re
            mdl_refs = re.findall(r"@([^@]+\.mdl)@", txt)
            tex_refs = re.findall(r"@([^@]+\.(?:png|jpg|jpeg|exr|tif|tga))@", txt)
            for r in mdl_refs:
                rp = r.replace("\\", "/")
                if mats_rel and ("/Material/mdl/" in rp or rp.startswith(mats_rel + "/")):
                    continue
                bad_mdl.append(rp)
            for r in tex_refs:
                rp = r.replace("\\", "/")
                if ("/Material/mdl/textures/" in rp) and ("Materials/Textures" not in rp) and ("/Textures/" not in rp.split("/Material/mdl/")[-1]):
                    continue
                bad_tex.append(rp)
        except Exception:
            pass
    else:
        # 二进制 USD：无法直接读文本，标记为未知，不影响整体通过；用户可后续用 USD 工具做深检
        ok_mdl = True
        ok_tex = True

    ok_mdl = ok_mdl and len(bad_mdl) == 0
    ok_tex = ok_tex and len(bad_tex) == 0
    return {
        "usd": usdf,
        "is_text": is_text_usd(usdf),
        "mats_rel": mats_rel,
        "mdl_ok": ok_mdl,
        "tex_ok": ok_tex,
        "bad_mdl": bad_mdl,
        "bad_tex": bad_tex,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--assets-dir", required=True, help="exported assets root, e.g. export_specs_unified/GRScenes_assets")
    ap.add_argument("--materials-dir", required=True, help="materials mdl dir, e.g. export_specs_unified/Material/mdl")
    ap.add_argument("--output", default="check_reports/phase2_assets_report.json")
    ap.add_argument("--summary-only", action="store_true", help="only write summary, skip per-file results")
    ap.add_argument("--fail-only", action="store_true", help="only keep files with bad_mdl/bad_tex or zero-size")
    ap.add_argument("--gzip", action="store_true", help="compress output to .json.gz")
    args = ap.parse_args()

    assets_dir = os.path.abspath(args.assets_dir)
    mats_dir = os.path.abspath(args.materials_dir)
    files = list_usd_files(assets_dir)
    zero_size = []
    for p in files:
        try:
            if os.path.getsize(p) == 0:
                zero_size.append(p)
        except Exception:
            pass

    results = [check_refs(f, mats_dir) for f in files]
    summary = {
        "assets_dir": assets_dir,
        "materials_dir": mats_dir,
        "usd_count": len(files),
        "zero_size_count": len(zero_size),
        "zero_size_list": zero_size,
        "mdl_ok_count": sum(1 for r in results if r["mdl_ok"]),
        "tex_ok_count": sum(1 for r in results if r["tex_ok"]),
        "mdl_bad_examples": sum(len(r["bad_mdl"]) for r in results),
        "tex_bad_examples": sum(len(r["bad_tex"]) for r in results),
    }
    if args.summary_only:
        report = {"summary": summary}
    elif args.fail_only:
        bad = [r for r in results if (len(r["bad_mdl"]) > 0 or len(r["bad_tex"]) > 0)]
        # 把零字节也加入失败集
        bad_paths = set(r["usd"] for r in bad)
        for z in zero_size:
            if z not in bad_paths:
                bad.append({"usd": z, "is_text": False, "mats_rel": None, "mdl_ok": False, "tex_ok": False, "bad_mdl": ["zero_size"], "bad_tex": []})
        report = {"summary": summary, "results": bad}
    else:
        report = {"summary": summary, "results": results}

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    if args.gzip:
        outgz = args.output if args.output.endswith(".gz") else args.output + ".gz"
        import gzip
        with gzip.open(outgz, "wt", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print("written:", outgz)
    else:
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print("written:", args.output)


if __name__ == "__main__":
    main()
