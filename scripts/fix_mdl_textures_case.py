#!/usr/bin/env python3
import os, argparse, json, re

def list_mdl(root):
    out = []
    for d, _, fns in os.walk(root):
        for fn in fns:
            if fn.lower().endswith(".mdl"):
                out.append(os.path.join(d, fn))
    return out

def fix_text(s):
    p1 = re.compile(r"([\"'])(file:)?([^\"']*?Material/mdl/)Textures/([^\"']+\.(?:png|jpg|jpeg|exr|tif|tga|bmp|webp))(\1)")
    p2 = re.compile(r"([\"'])(file:)?([^\"']*?)Textures/([^\"']+\.(?:png|jpg|jpeg|exr|tif|tga|bmp|webp))(\1)")
    s2 = p1.sub(lambda m: m.group(1) + (m.group(2) or "") + m.group(3) + "textures/" + m.group(4) + m.group(5), s)
    s3 = p2.sub(lambda m: m.group(1) + (m.group(2) or "") + m.group(3) + "textures/" + m.group(4) + m.group(5), s2)
    return s3

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mdl-root", required=True)
    ap.add_argument("--output", default="check_reports/fix_mdl_textures_case_report.json")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    root = os.path.abspath(args.mdl_root)
    files = list_mdl(root)
    report = {"root": root, "total": len(files), "changed": [], "unchanged": []}
    for f in files:
        try:
            with open(f, "r", encoding="utf-8", errors="ignore") as fh:
                txt = fh.read()
        except Exception:
            report["unchanged"].append({"file": f, "reason": "read_failed"})
            continue
        new_txt = fix_text(txt)
        if new_txt != txt:
            if not args.dry_run:
                try:
                    with open(f + ".bak", "w", encoding="utf-8") as bk:
                        bk.write(txt)
                    with open(f, "w", encoding="utf-8") as fh:
                        fh.write(new_txt)
                except Exception:
                    report["unchanged"].append({"file": f, "reason": "write_failed"})
                    continue
            report["changed"].append(f)
        else:
            report["unchanged"].append({"file": f, "reason": "no_uppercase_found"})
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as out:
        json.dump(report, out, ensure_ascii=False, indent=2)
    print("written:", args.output)

if __name__ == "__main__":
    main()

