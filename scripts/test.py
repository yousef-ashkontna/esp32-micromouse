#!/usr/bin/env python3
"""Compile and run the host regression suites."""

import os
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which(os.environ.get("CXX", "g++"))
    if compiler is None:
        raise SystemExit("C++ compiler not found. Install g++ or set CXX to a compiler path.")
    output = root / ".pio" / "host-tests"
    output.mkdir(parents=True, exist_ok=True)
    suffix = ".exe" if os.name == "nt" else ""
    for suite in ("navigation", "firmware"):
        executable = output / (suite + "_test" + suffix)
        command = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", "-pedantic", "-Iinclude"]
        if suite == "firmware":
            command.append("-Itest/host/mocks")
        command += [f"test/host/{suite}_test.cpp", "-o", str(executable)]
        subprocess.run(command, cwd=root, check=True)
        subprocess.run([str(executable)], cwd=root, check=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except subprocess.CalledProcessError as error:
        sys.exit(error.returncode)
