"""Build and run tests/host_test.cpp with the zig C++ compiler (pip install ziglang)."""

from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
BUILD = HERE / "build"


def main() -> int:
    BUILD.mkdir(exist_ok=True)
    exe = BUILD / ("host_test.exe" if sys.platform == "win32" else "host_test")
    cmd = [
        sys.executable,
        "-m",
        "ziglang",
        "c++",
        "-std=c++17",
        "-O1",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-Wno-nullability-completeness",
        "-fsanitize=undefined",
        "-fno-sanitize-recover=all",
        str(HERE / "host_test.cpp"),
        "-o",
        str(exe),
    ]
    subprocess.run(cmd, check=True)
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
