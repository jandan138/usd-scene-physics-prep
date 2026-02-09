#!/usr/bin/env python3

import argparse

from pxr import Usd


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("layout", help="Path to layout.usd")
    ap.add_argument("substring", help="Substring to search in prim paths")
    ap.add_argument("--limit", type=int, default=50)
    args = ap.parse_args()

    stage = Usd.Stage.Open(args.layout)
    if not stage:
        print(f"ERROR: failed to open: {args.layout}")
        return 2

    hits = []
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if args.substring in p:
            hits.append(prim)

    print(f"hits: {len(hits)}")
    for prim in hits[: args.limit]:
        has_ref = prim.HasAuthoredReferences()
        has_payload = prim.HasAuthoredPayloads()
        print(f"{prim.GetPath()}  type={prim.GetTypeName() or '(empty)'}  ref={has_ref} payload={has_payload}")

    if len(hits) > args.limit:
        print(f"... truncated to {args.limit}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
