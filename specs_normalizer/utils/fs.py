import os
import shutil

def ensure_dir(p):
    os.makedirs(p, exist_ok=True)

def copy_file(src, dst):
    ensure_dir(os.path.dirname(dst))
    shutil.copy2(src, dst)

def copy_dir(src, dst):
    for root, dirs, files in os.walk(src):
        rel = os.path.relpath(root, src)
        target = os.path.join(dst, rel) if rel != "." else dst
        ensure_dir(target)
        for f in files:
            copy_file(os.path.join(root, f), os.path.join(target, f))

