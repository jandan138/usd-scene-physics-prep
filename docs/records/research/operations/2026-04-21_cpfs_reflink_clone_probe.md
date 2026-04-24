---
title: CPFS Reflink Clone Probe Under /cpfs/user/zhuzihou
code_reference: []
created_at: 2026-04-21
updated_at: 2026-04-21
maintainer: OpenCode
status: active
doc_class: record
---

# CPFS Reflink Clone Probe Under /cpfs/user/zhuzihou

## Scope

Investigated whether `/cpfs/user/zhuzihou` supports a safe low-space promoted-clone strategy such as reflink/copy-on-write cloning, filesystem-native clone ioctls, or another practical clone path that avoids duplicating full dataset blocks.

The source dataset was not touched. Probes were limited to read-only inspection plus tiny temporary operations under `/cpfs/user/zhuzihou/tmp_downloads/`.

## Evidence Gathered

Environment and mount inspection:

```bash
stat -f -c '%T %a %s %S' /cpfs/user/zhuzihou
df -h /cpfs/user/zhuzihou
df -T /cpfs/user/zhuzihou
read /proc/mounts
cp --help
cp --version
```

Observed results:

- `/cpfs/user/zhuzihou` is mounted as `fuse.aliyun-alinas-efc`
- `cp` is GNU coreutils `8.32` and exposes `--reflink[=WHEN]`
- the mount is a FUSE-backed Alibaba CPFS/EFC client, not a local XFS/Btrfs/APFS-style filesystem mount

Relevant mount entry from `/proc/mounts`:

```text
bindroot-8c49f-deuIkFBy:cpfs-030wldnqm8evtpyhw1e-vpc-eyyd1d.cn-beijing.cpfs.aliyuncs.com:/ /cpfs/user/zhuzihou fuse.aliyun-alinas-efc rw,relatime,user_id=0,group_id=0,default_permissions,allow_other,max_read=1048576,dontcache=write,dircto 0 0
```

Writable-space probes under `/cpfs/user/zhuzihou/tmp_downloads`:

```bash
touch /cpfs/user/zhuzihou/tmp_downloads/<probe>/touch.test
dd if=/dev/zero of=/cpfs/user/zhuzihou/tmp_downloads/<probe>/4k.bin bs=4096 count=1 status=none
```

Observed results:

- empty file creation succeeds
- writing even a single 4 KiB block fails with `No space left on device`
- this indicates the path can still accept metadata updates while data-block allocation is currently exhausted or quota-blocked for this user/subtree

Direct reflink probe against an existing file on the same target mount:

```bash
cp --reflink=always \
  /cpfs/user/zhuzihou/tmp_downloads/Miniconda3-latest-Linux-x86_64.sh \
  /cpfs/user/zhuzihou/tmp_downloads/<probe>/clone.sh
```

Observed result:

```text
cp: failed to clone '.../clone.sh' from '.../Miniconda3-latest-Linux-x86_64.sh': Operation not supported
```

Hard-link contrast probe:

```bash
ln \
  /cpfs/user/zhuzihou/tmp_downloads/Miniconda3-latest-Linux-x86_64.sh \
  /cpfs/user/zhuzihou/tmp_downloads/<probe>/hardlink.sh
stat -c 'inode=%i links=%h size=%s blocks=%b path=%n' \
  /cpfs/user/zhuzihou/tmp_downloads/Miniconda3-latest-Linux-x86_64.sh \
  /cpfs/user/zhuzihou/tmp_downloads/<probe>/hardlink.sh
```

Observed result:

- `ln` succeeds
- both paths report the same inode and link count, proving that metadata-only aliasing works, but only as an unsafe hard link

## Interpretation

The key practical signal is the reflink probe on an existing file:

- `cp --reflink=always` reached the filesystem clone path
- the clone request failed with `Operation not supported`, not with `No space left on device`
- that is strong evidence that this mounted CPFS client does not expose the clone/reflink operation that GNU `cp` needs for safe CoW file cloning

Because the probe used a source and destination on the same mount and did not require cross-filesystem behavior, this is the most relevant real-world test for the requested promoted-clone workflow.

## Safety Assessment

### Reflink / CoW clone

- Not practically available from this environment on `/cpfs/user/zhuzihou`
- If it were available, it would be safe for later in-place edits because modified blocks would diverge per file
- The mount does not appear to support it

### Hard links

- Available
- Not safe for the promoted clone requirement
- Later in-place edits would modify the shared inode content seen by the source tree as well
- Deleting only the clone pathname would not mutate file contents, but content edits remain unsafe, so hard links are not acceptable as the promoted-clone substrate

### Plain copy / rsync copy

- Safe for later edits and deletions
- Not low-space
- Currently blocked in practice by `No space left on device` on this target path

## Conclusion

No safe low-space clone method was demonstrated on `/cpfs/user/zhuzihou` from this environment.

What is available:

- full copies, when enough writable quota/space exists
- hard links, which are unsafe for independent promoted-clone edits

What was not available:

- reflink / CoW clone via `cp --reflink=always`
- any other filesystem-native clone behavior visible through the mounted FUSE client

## Recommendation

Treat `/cpfs/user/zhuzihou` as lacking a safe reflink-style promoted-clone path from this environment.

Recommended next step:

1. Do not use hard links for the promoted clone.
2. Assume a true independent promoted clone requires real copied blocks.
3. Resolve the actual writable-space/quota blocker first, or provision a destination with verified reflink support on a different filesystem.
4. If Alibaba CPFS has a server-side snapshot/clone feature, verify it through platform-specific admin tooling or cloud APIs rather than Linux `cp`; no such client-visible feature was available here.

## Commands And Results Summary

```bash
stat -f -c '%T %a %s %S' /cpfs/user/zhuzihou
# -> fuseblk ...

df -T /cpfs/user/zhuzihou
# -> Type: fuse.aliyun-alinas-efc

cp --reflink=always SRC DEST
# -> Operation not supported

ln SRC DEST
# -> succeeds; same inode on both paths

dd if=/dev/zero of=.../4k.bin bs=4096 count=1 status=none
# -> No space left on device
```
