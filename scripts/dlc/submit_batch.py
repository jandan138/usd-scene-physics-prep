import argparse
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional


def submit_jobs(
    chunk_total: int,
    task_name: str,
    data_sources: Optional[str] = None,
    command_args: Optional[str] = None,
    max_total: int = 100,
) -> None:
    repo_root = Path(__file__).resolve().parents[2]
    launch_script = repo_root / "scripts" / "dlc" / "launch_job.sh"
    if not launch_script.exists():
        print(f"ERROR: launch script not found: {launch_script}", file=sys.stderr)
        raise SystemExit(1)

    # 安全检查: 防止意外大批量提交 / Safety guard against accidental massive submissions
    if chunk_total > max_total:
        print(
            f"ERROR: chunk_total={chunk_total} exceeds --max-total={max_total}. "
            "Pass a larger --max-total to override.",
            file=sys.stderr,
        )
        raise SystemExit(2)

    successful: List[int] = []
    failed: List[int] = []

    for chunk_id in range(chunk_total):
        cmd: List[str] = [
            "bash",
            str(launch_script),
            task_name,
            str(chunk_id),
            str(chunk_total),
        ]
        # 参数4: data_sources — 必须始终占位, 否则 command_args 会被误解为 data_sources
        # Arg 4: data_sources — must always be present so command_args lands at $5, not $4
        DEFAULT_DATA_SOURCES = "d-mzps5b7joy2axmqpa8,d-d49o5g0h2818sw8j1g,d-8wz4emfs21s5ajs9oz"
        cmd.append(data_sources if data_sources else DEFAULT_DATA_SOURCES)
        # 参数5: command_args (可选, 覆盖 run_task.sh 的运行模式)
        # Arg 5: command_args (optional, overrides run_task.sh run mode)
        if command_args:
            cmd.append(command_args)

        print(f"Submitting chunk {chunk_id}/{chunk_total - 1}: {' '.join(cmd)}")

        # 最多重试3次, 指数退避 1s/2s/4s / Up to 3 retries with exponential backoff 1s/2s/4s
        max_retries = 3
        success = False
        for attempt in range(max_retries + 1):
            try:
                subprocess.run(cmd, check=True, cwd=str(repo_root))
                success = True
                break
            except subprocess.CalledProcessError as exc:
                if attempt < max_retries:
                    wait = 2 ** attempt  # 1, 2, 4 seconds
                    print(
                        f"  Attempt {attempt + 1} failed (exit {exc.returncode}), "
                        f"retrying in {wait}s...",
                        file=sys.stderr,
                    )
                    time.sleep(wait)
                else:
                    print(
                        f"  Chunk {chunk_id} failed after {max_retries + 1} attempts.",
                        file=sys.stderr,
                    )

        if success:
            successful.append(chunk_id)
        else:
            failed.append(chunk_id)

    # 汇总报告 / Summary report
    print(f"\n=== Submission summary ===")
    print(f"  Successful chunks ({len(successful)}): {successful}")
    print(f"  Failed chunks    ({len(failed)}): {failed}")
    if failed:
        raise SystemExit(1)


def main() -> None:
    parser = argparse.ArgumentParser(description="Submit batch DLC jobs")
    parser.add_argument("--total", type=int, required=True, help="Total chunk count")
    parser.add_argument("--name", type=str, default="physics_prep", help="Base task name")
    parser.add_argument("--data_sources", type=str, default=None, help="Comma-separated data source IDs")
    parser.add_argument("--command_args", type=str, default=None,
                        help="Custom run_task.sh args (e.g. 'process /path/to/assets')")
    parser.add_argument("--max-total", type=int, default=100,
                        help="Maximum allowed chunk count to prevent accidental massive submissions (default: 100)")
    args = parser.parse_args()

    if args.total <= 0:
        print("ERROR: --total must be a positive integer", file=sys.stderr)
        raise SystemExit(2)

    submit_jobs(args.total, args.name, args.data_sources, args.command_args, args.max_total)


if __name__ == "__main__":
    main()
