#!/usr/bin/env python3
"""Build the Zephyr ztests and run QEMU until it reports a test verdict."""

import argparse
import os
import selectors
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

SUCCESS = b"PROJECT EXECUTION SUCCESSFUL"
FAILURES = (b"PROJECT EXECUTION FAILED", b"FATAL ERROR")


def stop_group(proc):
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()


def run_tests(build_dir, timeout):
    with subprocess.Popen(["west", "build", "-d", build_dir, "-t", "run"],
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          start_new_session=True) as proc, selectors.DefaultSelector() as selector:
        ended = False
        try:
            selector.register(proc.stdout, selectors.EVENT_READ)
            deadline = time.monotonic() + timeout
            tail = b""
            while (remaining := deadline - time.monotonic()) > 0 and selector.select(remaining):
                chunk = os.read(proc.stdout.fileno(), 8192)
                if not chunk:
                    ended = proc.poll() is not None
                    sys.exit("QEMU output ended without a ztest success marker")
                sys.stdout.buffer.write(chunk)
                sys.stdout.buffer.flush()
                data = tail + chunk
                if any(marker in data for marker in FAILURES):
                    sys.exit("QEMU reported a test failure")
                if SUCCESS in data:
                    return
                tail = data[-(max(map(len, (*FAILURES, SUCCESS))) - 1):]
            sys.exit(f"QEMU timed out after {timeout} seconds")
        finally:
            if not ended:
                stop_group(proc)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", default="build/ztest_ci")
    parser.add_argument("--timeout", default=120, type=int)
    parser.add_argument("--pristine", action="store_true")
    parser.add_argument("--skip-build", action="store_true")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    sdk = os.environ.get("ZEPHYR_SDK_INSTALL_DIR")
    if sdk:
        sdk_path = Path(sdk)
        candidates = [
            sdk_path / "hosttools/usr/bin",
            *sorted((sdk_path / "hosttools").glob("sysroots/*-pokysdk-linux/usr/bin")),
        ]
        extra_paths = [str(p) for p in candidates if p.is_dir()]
        if extra_paths:
            os.environ["PATH"] = os.pathsep.join([*extra_paths, os.environ["PATH"]])
    for tool in ("west", "qemu-system-arm"):
        if not shutil.which(tool):
            sys.exit(f"Required executable missing from PATH: {tool}")
    if not args.skip_build:
        command = ["west", "build", "-b", "qemu_cortex_m3", "-d", args.build_dir]
        if args.pristine:
            command.append("--pristine=always")
        subprocess.run([*command, "tests/ztest"], check=True)
    run_tests(args.build_dir, args.timeout)


if __name__ == "__main__":
    main()
