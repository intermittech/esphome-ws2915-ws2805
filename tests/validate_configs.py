"""Schema tests: run `esphome config` on small configs and check pass/fail and messages.

Usage: .venv/Scripts/python tests/validate_configs.py
"""

from pathlib import Path
import subprocess
import sys
import textwrap

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
WORK = HERE / "build" / "cfg"
ESPHOME = [sys.executable, "-m", "esphome"]

BASE = """\
esphome:
  name: t
esp32:
  board: {board}
  framework:
    type: esp-idf
external_components:
  - source:
      type: local
      path: {components}
    components: [ws2915]
"""

HUB16 = """\
ws2915:
  id: d2a
  pin: GPIO16
  num_chips: 1
  gain: 31
"""

OUT = """\
output:
  - platform: ws2915
    id: o1
    channel: {channel}
    chip: {chip}
"""

# (name, yaml body, expect_ok, substring expected in output or None, board)
CASES = [
    ("minimal ws2915", HUB16, True, None),
    ("ws2915 without gain", "ws2915:\n  pin: GPIO16\n  num_chips: 1\n", False, "gain is required"),
    (
        "ws2805 with gain",
        "ws2915:\n  chip_type: ws2805\n  pin: GPIO16\n  num_chips: 1\n  gain: 31\n",
        False,
        "no gain header",
    ),
    (
        "ws2805 with header_format",
        "ws2915:\n  chip_type: ws2805\n  pin: GPIO16\n  num_chips: 1\n  header_format: 32bit\n",
        False,
        "no gain header",
    ),
    ("channel 5", HUB16 + OUT.format(channel=5, chip=0), True, None),
    ("channel ch3", HUB16 + OUT.format(channel="ch3", chip=0), True, None),
    ("channel w2", HUB16 + OUT.format(channel="w2", chip=0), True, None),
    ("channel white1", HUB16 + OUT.format(channel="white1", chip=0), True, None),
    ("channel 0", HUB16 + OUT.format(channel=0, chip=0), False, "must be at least 1"),
    ("channel 6", HUB16 + OUT.format(channel=6, chip=0), False, "must be at most 5"),
    ("channel bogus", HUB16 + OUT.format(channel="yellow", chip=0), False, "unknown channel"),
    ("chip out of range", HUB16 + OUT.format(channel=1, chip=1), False, "out of range"),
    (
        "gain map with ch keys",
        "ws2915:\n  pin: GPIO16\n  num_chips: 1\n  gain: {ch1: 31, ch2: 31, ch3: 31, w1: 20, white2: 0}\n",
        True,
        "gain 0 on white2",
    ),
    (
        "gain map missing channel",
        "ws2915:\n  pin: GPIO16\n  num_chips: 1\n  gain: {red: 31, green: 31, blue: 31, white1: 31}\n",
        False,
        "missing channel",
    ),
    ("gain 32", HUB16.replace("gain: 31", "gain: 32"), False, "at most 31"),
    ("reset 250us", HUB16 + "  reset_time: 250us\n", False, "> 280us"),
    (
        "ws2915 T1H 550 ns in spec",
        HUB16 + "  bit1_high: 550ns\n  bit1_low: 750ns\n",
        True,
        "!outside",
    ),
    (
        "ws2805 T1H 550 ns out of spec",
        "ws2915:\n  chip_type: ws2805\n  pin: GPIO16\n  num_chips: 1\n  bit1_high: 550ns\n  bit1_low: 750ns\n",
        True,
        "outside the ws2805 datasheet window",
    ),
    ("rate 200Hz", HUB16 + "  transition_refresh_rate: 200Hz\n", True, None),
    ("rate 200fps", HUB16 + "  transition_refresh_rate: 200fps\n", True, None),
    ("rate 5ms", HUB16 + "  transition_refresh_rate: 5ms\n", True, None),
    ("rate 2000Hz", HUB16 + "  transition_refresh_rate: 2000Hz\n", False, "1-1000 Hz"),
    (
        "rate slower than frame",
        HUB16.replace("num_chips: 1", "num_chips: 100") + "  transition_refresh_rate: 500Hz\n",
        True,
        "longer than",
    ),
    ("refresh never", HUB16 + "  refresh_interval: never\n", True, None),
    ("refresh 10ms", HUB16 + "  refresh_interval: 10ms\n", False, None),
    (
        "power limit without cap",
        HUB16 + "  power_limit:\n    channel_current: 3A\n",
        False,
        "needs max_chip_current",
    ),
    (
        "power limit chip cap",
        HUB16 + "  power_limit:\n    channel_current: 3A\n    max_chip_current: 5A\n",
        True,
        None,
    ),
    (
        "power limit max_power w/o voltage",
        HUB16 + "  power_limit:\n    channel_current: 3A\n    max_power: 150W\n",
        False,
        "needs supply_voltage",
    ),
    (
        "power limit max_power",
        HUB16
        + "  power_limit:\n    channel_current: {r: 3A, g: 3A, b: 3A, w1: 5A, w2: 5A}\n"
        "    max_power: 150W\n    supply_voltage: 24V\n",
        True,
        None,
    ),
    (
        "power limit current + power",
        HUB16
        + "  power_limit:\n    channel_current: 3A\n    max_current: 6A\n"
        "    max_power: 150W\n    supply_voltage: 24V\n",
        False,
        "not both",
    ),
    ("use_dma on esp32", HUB16 + "  use_dma: true\n", False, None),
]


def run_case(idx, name, body, expect_ok, needle, board="esp32dev"):
    WORK.mkdir(parents=True, exist_ok=True)
    path = WORK / f"case{idx:02d}.yaml"
    path.write_text(
        BASE.format(board=board, components=(ROOT / "components").as_posix()) + body,
        encoding="utf-8",
    )
    proc = subprocess.run(
        ESPHOME + ["config", str(path)], capture_output=True, text=True, encoding="utf-8", errors="replace"
    )
    out = proc.stdout + proc.stderr
    ok = proc.returncode == 0
    problems = []
    if ok != expect_ok:
        problems.append(f"expected {'pass' if expect_ok else 'fail'}, got rc={proc.returncode}")
    if needle:
        if needle.startswith("!"):
            if needle[1:] in out:
                problems.append(f"unexpected '{needle[1:]}' in output")
        elif needle not in out:
            problems.append(f"'{needle}' not in output")
    status = "ok  " if not problems else "FAIL"
    print(f"{status} {name}" + ("" if not problems else f": {'; '.join(problems)}"))
    if problems:
        print(textwrap.indent(out[-1500:], "     | "))
    return not problems


def main() -> int:
    results = [run_case(i, *case) for i, case in enumerate(CASES)]
    results.append(
        run_case(
            len(CASES),
            "esp32-c2 rejected (no RMT)",
            HUB16,
            False,
            "RMT",
            board="esp32-c2-devkitm-1",
        )
    )
    failed = results.count(False)
    print(f"\n{len(results) - failed}/{len(results)} schema cases passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
