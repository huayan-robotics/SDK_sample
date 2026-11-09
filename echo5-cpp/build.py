#!/usr/bin/env python3
"""Configure and build all C++ samples with CMake on Windows or Linux."""
from __future__ import annotations

import argparse
import subprocess
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Build Huayan C++ SDK samples")
    parser.add_argument(
        "--config",
        choices=("Debug", "Release", "RelWithDebInfo", "MinSizeRel"),
        default="Release",
        help="CMake build configuration (default: Release)",
    )
    parser.add_argument("--build-dir", default="build", help="Build directory")
    parser.add_argument("--generator", help="Optional CMake generator")
    parser.add_argument("--clean", action="store_true", help="Clean before build")
    args = parser.parse_args()

    root = Path(__file__).resolve().parent
    build_dir = root / args.build_dir
    configure = [
        "cmake", "-S", str(root), "-B", str(build_dir),
        f"-DCMAKE_BUILD_TYPE={args.config}",
    ]
    if args.generator:
        configure.extend(["-G", args.generator])

    subprocess.run(configure, check=True)
    # Visual Studio/MSBuild may race while writing tracking logs when the
    # workspace is hosted on a shared folder. Build one target at a time so
    # exported samples remain reliable in that supported setup.
    build = [
        "cmake", "--build", str(build_dir),
        "--config", args.config,
        "--parallel", "1",
    ]
    if args.clean:
        build.append("--clean-first")
    subprocess.run(build, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
