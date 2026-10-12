"""Build and run tests/ultra_host_test.cpp with the zig C/C++ compiler (pip install ziglang).

With the QuinLED dig2analog-ultra-5ch firmware next to this repo (or ULTRA_FIRMWARE=<firmware dir>), the
board's light engine is compiled in and the chain test runs too; the protocol header copy is checked
against the firmware's master.
"""

import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / "build"
ZIG = [sys.executable, "-m", "ziglang"]


def main() -> int:
    BUILD.mkdir(exist_ok=True)
    exe = BUILD / ("ultra_host_test.exe" if sys.platform == "win32" else "ultra_host_test")
    fw = Path(os.environ.get("ULTRA_FIRMWARE", ROOT.parent / "dig2analog-ultra-5ch" / "firmware"))
    core = fw / "src" / "core"
    cmd = ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-nullability-completeness",
                 "-fsanitize=undefined", "-fno-sanitize-recover=all"]
    objs = []
    if (core / "ultra.c").exists():
        master = (core / "ultra_protocol.h").read_bytes().replace(b"\r\n", b"\n")
        copy = (ROOT / "components" / "dig2analog_ultra" / "ultra_protocol.h").read_bytes().replace(b"\r\n", b"\n")
        if master != copy:
            print("FAIL: components/dig2analog_ultra/ultra_protocol.h differs from the firmware's master copy")
            return 1
        for src in ("ultra.c", "ultra_tables.c", "config.c", "presets.c"):
            obj = BUILD / (src[:-2] + ".o")
            subprocess.run(ZIG + ["cc", "-std=c99", "-O1", "-Wall", "-fsanitize=undefined",
                                  "-fno-sanitize-recover=all", "-c", str(core / src), "-o", str(obj)], check=True)
            objs.append(str(obj))
        cmd += ["-DWITH_ENGINE", f"-I{core}"]
        print(f"board engine: {core}")
    cmd += [str(HERE / "ultra_host_test.cpp"), *objs, "-o", str(exe)]
    subprocess.run(cmd, check=True)
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
