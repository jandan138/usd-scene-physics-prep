#!/usr/bin/env python3

import argparse

from pxr import Usd


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", help="USD stage path (e.g. layout.usd)")
    ap.add_argument("substring", help="Substring to match in prim path")
    ap.add_argument("--limit", type=int, default=200)
    args = ap.parse_args()

    stage = Usd.Stage.Open(args.stage)
    if not stage:
        print(f"ERROR: Failed to open stage: {args.stage}")
        return 2

    hits = []
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if args.substring in p:
            hits.append((p, prim.GetTypeName() or "(empty)"))

    print(f"hits: {len(hits)}")
    for i, (p, tn) in enumerate(hits[: args.limit]):
        print(f"- {p} ({tn})")
    if len(hits) > args.limit:
        print(f"... ({len(hits)} total, truncated)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
