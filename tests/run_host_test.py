"""Build and run tests/host_test.cpp with the zig C++ compiler (pip install ziglang).

With the QuinLED dig2analog-ultra-5ch firmware next to this repo (../quinled-ultra-firmware, the hardware project's
../dig2analog-ultra-5ch/firmware, or ULTRA_FIRMWARE=<firmware dir>), the board's config parser is compiled in and
ws2915.send_config's frames are checked against it.
"""

import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / "build"
ZIG = [sys.executable, "-m", "ziglang"]


def firmware_dir() -> Path:
    env = os.environ.get("ULTRA_FIRMWARE")
    if env:
        return Path(env)
    for cand in (ROOT.parent / "dig2analog-ultra-5ch" / "firmware", ROOT.parent / "quinled-ultra-firmware"):
        if (cand / "src" / "core" / "config.c").exists():
            return cand
    return ROOT.parent / "quinled-ultra-firmware"


def main() -> int:
    BUILD.mkdir(exist_ok=True)
    exe = BUILD / ("host_test.exe" if sys.platform == "win32" else "host_test")
    cmd = ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-nullability-completeness",
                 "-fsanitize=undefined", "-fno-sanitize-recover=all"]
    objs = []
    core = firmware_dir() / "src" / "core"
    if (core / "config.c").exists():
        for src in ("config.c", "presets.c"):
            obj = BUILD / ("fw_" + src[:-2] + ".o")
            subprocess.run(ZIG + ["cc", "-std=c99", "-O1", "-Wall", "-fsanitize=undefined",
                                  "-fno-sanitize-recover=all", "-c", str(core / src), "-o", str(obj)], check=True)
            objs.append(str(obj))
        cmd += ["-DWITH_FIRMWARE", f"-I{core}"]
        print(f"board firmware: {core}")
    cmd += [str(HERE / "host_test.cpp"), *objs, "-o", str(exe)]
    subprocess.run(cmd, check=True)
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
