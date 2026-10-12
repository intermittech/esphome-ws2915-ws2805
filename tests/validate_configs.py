"""Schema tests: run `esphome config` on small configs and check pass/fail and messages.

Usage: .venv/Scripts/python tests/validate_configs.py [name filter ...]
       (with filters, only the cases whose name contains one of them run)
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
    components: [ws2915, dig2analog_ultra, dig2analog_ultra_strip]
"""

HUB16 = """\
ws2915:
  id: d2a
  pin: GPIO16
  num_chips: 1
  gain: 31
"""

PSU = """\
power_supply:
  - id: psu
    pin: GPIO13
    enable_time: {enable}
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
    ("refresh default ws2915 = never", HUB16, True, "refresh_interval: never"),
    (
        "refresh default ws2805 = 1s",
        "ws2915:\n  chip_type: ws2805\n  pin: GPIO16\n  num_chips: 1\n",
        True,
        "refresh_interval: 1s",
    ),
    (
        "refresh explicit on ws2805",
        "ws2915:\n  chip_type: ws2805\n  pin: GPIO16\n  num_chips: 1\n  refresh_interval: never\n",
        True,
        "refresh_interval: never",
    ),
    ("refresh 10ms", HUB16 + "  refresh_interval: 10ms\n", False, None),
    (
        "power limit without limit",
        HUB16 + "  power_limit:\n    channel_current: 3A\n",
        False,
        "needs at least one of",
    ),
    (
        "channel limits: default + overrides",
        HUB16
        + "  power_limit:\n    channel_current: {default: 2A, ch4: 4A, ch5: 4A}\n"
        "    max_channel_current: {default: 3A, ch4: 5A, ch5: 5A}\n",
        True,
        "!limited at",
    ),
    (
        "channel limit single value",
        HUB16 + "  power_limit:\n    channel_current: 2A\n    max_channel_current: 3A\n",
        True,
        None,
    ),
    (
        "channel limit on some channels only",
        HUB16 + "  power_limit:\n    channel_current: 6A\n    max_channel_current: {w1: 5A, w2: 5A}\n",
        True,
        "CH4 (white1) draws 6.00 A at 100 % but is limited to 5.00 A",
    ),
    (
        "channel limit bad channel",
        HUB16 + "  power_limit:\n    channel_current: 2A\n    max_channel_current: {ch6: 3A}\n",
        False,
        "unknown channel 'ch6'",
    ),
    (
        "channel_current partial without default",
        HUB16 + "  power_limit:\n    channel_current: {red: 3A}\n    max_chip_current: 5A\n",
        False,
        "or give a default",
    ),
    (
        "gain map with default",
        "ws2915:\n  pin: GPIO16\n  num_chips: 1\n  gain: {default: 31, w2: 20}\n",
        True,
        None,
    ),
    (
        "power limit chip limit",
        HUB16 + "  power_limit:\n    channel_current: 3A\n    max_chip_current: 5A\n",
        True,
        None,
    ),
    (
        "power limit line current",
        HUB16
        + "  power_limit:\n    channel_current: {r: 3A, g: 3A, b: 3A, w1: 5A, w2: 5A}\n"
        "    max_current: 6.25A\n",
        True,
        None,
    ),
    (
        "max_power is rejected (amps only)",
        HUB16 + "  power_limit:\n    channel_current: 3A\n    max_power: 150W\n    supply_voltage: 24V\n",
        False,
        "all limits are in amps",
    ),
    (
        "supply_voltage alone is rejected",
        HUB16 + "  power_limit:\n    channel_current: 3A\n    max_current: 6A\n    supply_voltage: 24V\n",
        False,
        "supply_voltage is not supported",
    ),
    ("use_dma on esp32", HUB16 + "  use_dma: true\n", False, None),
    ("power_on_delay alone", HUB16 + "  power_on_delay: 1.5s\n", True, None),
    ("power_on_delay 60s", HUB16 + "  power_on_delay: 60s\n", False, None),
    (
        "hub power_supply non-blocking",
        PSU.format(enable="0ms") + HUB16 + "  power_supply: psu\n  power_on_delay: 1.5s\n",
        True,
        "!blocks the main loop",
    ),
    (
        "hub power_supply blocking enable_time",
        PSU.format(enable="1500ms") + HUB16 + "  power_supply: psu\n",
        True,
        "enable_time 1500 ms, which blocks the main loop",
    ),
    ("hub power_supply unknown id", HUB16 + "  power_supply: nope\n", False, "nope"),
    ("power_on_fade is gone", HUB16 + "  power_on_delay: 1.5s\n  power_on_fade: 2s\n", False, "power_on_fade"),
    (
        "light on a held line",
        PSU.format(enable="0ms")
        + HUB16
        + "  power_on_delay: 1.5s\n"
        + OUT.format(channel=1, chip=0)
        + "    power_supply: psu\n"
        "light:\n  - platform: monochromatic\n    name: L\n    output: o1\n"
        "    default_transition_length: 2s\n",
        True,
        None,
    ),
    (
        "power_supply on hub (default) and output",
        PSU.format(enable="0ms")
        + HUB16
        + "  power_supply: psu\n"
        + OUT.format(channel=1, chip=0)
        + "    power_supply: psu\n",
        True,
        None,
    ),
    (
        "power_supply on outputs, delay on hub",
        PSU.format(enable="0ms")
        + HUB16
        + "  power_on_delay: 1.5s\n"
        + OUT.format(channel=1, chip=0)
        + "    power_supply: psu\n",
        True,
        "!blocks the main loop",
    ),
    (
        "two outputs on two supplies",
        "power_supply:\n  - id: psu_a\n    pin: GPIO13\n    enable_time: 0ms\n"
        "  - id: psu_b\n    pin: GPIO14\n    enable_time: 0ms\n"
        + HUB16.replace("num_chips: 1", "num_chips: 2")
        + "  power_on_delay: 1.5s\n"
        "output:\n"
        "  - {platform: ws2915, id: a1, chip: 0, channel: 1, power_supply: psu_a}\n"
        "  - {platform: ws2915, id: b1, chip: 1, channel: 1, power_supply: psu_b}\n",
        True,
        None,
    ),
    (
        "output supply with blocking enable_time",
        PSU.format(enable="1500ms")
        + HUB16
        + OUT.format(channel=1, chip=0)
        + "    power_supply: psu\n",
        True,
        "enable_time 1500 ms, which blocks the main loop",
    ),
]


ULTRA = """\
dig2analog_ultra:
  id: u
  pin: GPIO16
  num_boards: 2
"""

ULTRA_RGBWW = """\
light:
  - platform: dig2analog_ultra
    name: rgbww
    board: 0
    red: 1
    green: 2
    blue: 3
    warm_white: 4
    cold_white: 5
    cold_white_color_temperature: 6500 K
    warm_white_color_temperature: 2700 K
    effects:
      - ultra_pulse:
      - ultra_color_loop:
      - ultra_strobe:
      - ultra_sunrise:
          duration: 10min
"""

ULTRA_CCT = """\
light:
  - platform: dig2analog_ultra
    name: cct
    board: 0
    warm_white: 1
    cold_white: 2
    cold_white_color_temperature: 6500 K
    warm_white_color_temperature: 2700 K
"""

ULTRA_MONO = """\
  - platform: dig2analog_ultra
    name: {name}
    board: {board}
    channel: {channel}
"""

CASES += [
    ("ultra minimal hub", ULTRA, True, None),
    # officially supported: up to 50 boards per data pin (research/chain-50-boards.md); the schema allows 64
    (
        "ultra 50 boards, lights on boards 0 and 49",
        ULTRA.replace("num_boards: 2", "num_boards: 50")
        + "light:\n"
        + ULTRA_MONO.format(name="first", board=0, channel=1)
        + ULTRA_MONO.format(name="last", board=49, channel=5),
        True,
        None,
    ),
    (
        "ultra 50 boards, a light on board 50",
        ULTRA.replace("num_boards: 2", "num_boards: 50") + "light:\n" + ULTRA_MONO.format(name="x", board=50, channel=1),
        False,
        "boards are 0-based",
    ),
    ("ultra num_boards 65", ULTRA.replace("num_boards: 2", "num_boards: 65"), False, "64"),
    ("ultra rgbww light with board effects", ULTRA + ULTRA_RGBWW, True, None),
    (
        "ultra monochromatic lights on board 1",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=1, channel=1) + ULTRA_MONO.format(name="b", board=1, channel="ch2"),
        True,
        None,
    ),
    (
        "ultra board out of range",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=2, channel=1),
        False,
        "board 2 out of range",
    ),
    (
        "ultra channel used twice",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1) + ULTRA_MONO.format(name="b", board=0, channel=1),
        False,
        "is used by both",
    ),
    (
        "ultra incomplete rgb",
        ULTRA + "light:\n  - platform: dig2analog_ultra\n    name: x\n    red: 1\n    green: 2\n",
        False,
        "give the channels of one light type",
    ),
    (
        "ultra constant_brightness on rgb",
        ULTRA + "light:\n  - platform: dig2analog_ultra\n    name: x\n    red: 1\n    green: 2\n    blue: 3\n"
        "    constant_brightness: true\n",
        False,
        "constant_brightness needs cold_white and warm_white",
    ),
    (
        "ultra color loop on a monochromatic light",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1) + "    effects:\n      - ultra_color_loop:\n",
        False,
        "needs an RGB light",
    ),
    (
        "ultra effect on a ws2915 light",
        HUB16 + OUT.format(channel=1, chip=0) + ULTRA
        + "light:\n  - platform: monochromatic\n    name: m\n    output: o1\n    effects:\n      - ultra_pulse:\n"
        + ULTRA_MONO.format(name="a", board=0, channel=1),
        False,
        "only runs on dig2analog_ultra lights",
    ),
    ("ultra pwm_frequency 150 kHz", ULTRA + "  pwm_frequency: 150kHz\n", False, "pwm_frequency must be"),
    ("ultra pwm_frequency 33 kHz", ULTRA + "  pwm_frequency: 33kHz\n", True, None),
    (
        "ultra settings",
        ULTRA + "  dithering: off_at_low_levels\n  dither_threshold: 2%\n  dim_floor: clamp\n  min_pulse: 150ns\n"
        "  signal_loss: fade_off\n  signal_loss_timeout: 5s\n  feedback_channel: all\n  stagger: false\n",
        True,
        None,
    ),
    ("ultra min_pulse 20 ns", ULTRA + "  min_pulse: 20ns\n", False, "min_pulse must be"),
    ("ultra stagger adaptive", ULTRA + "  stagger: adaptive\n", True, None),
    ("ultra stagger adaptive_spread", ULTRA + "  stagger: adaptive_spread\n", True, None),
    ("ultra stagger spread", ULTRA + "  stagger: spread\n", True, None),
    ("ultra stagger fixed_spread", ULTRA + "  stagger: fixed_spread\n", True, None),
    ("ultra stagger true", ULTRA + "  stagger: true\n", True, None),
    ("ultra stagger bogus", ULTRA + "  stagger: sideways\n", False, "Unknown value"),
    (
        "ultra power_limit without limits",
        ULTRA + "  power_limit:\n    channel_current: 2A\n",
        False,
        "power_limit needs",
    ),
    (
        "ultra power_limit",
        ULTRA + "  power_limit:\n    channel_current: {default: 2A, ch4: 4A, ch5: 4A}\n    max_channel_current: 5A\n"
        "    max_board_current: 8A\n",
        True,
        None,
    ),
    (
        "ultra power_limit measured board current only",
        ULTRA + "  power_limit:\n    max_board_current: 6A\n",
        True,
        None,
    ),
    (
        "ultra max_channel_current without channel_current",
        ULTRA + "  power_limit:\n    max_channel_current: 3A\n",
        False,
        "max_channel_current needs channel_current",
    ),
    (
        "ultra calibrate_current lambda",
        ULTRA + "button:\n  - platform: template\n    name: c\n    on_press:\n"
        "      - lambda: 'id(u).calibrate_current(0);'\n",
        True,
        None,
    ),
    (
        "ultra calibrate_current action",
        ULTRA + "button:\n  - platform: template\n    name: c\n    on_press:\n"
        "      - dig2analog_ultra.calibrate_current:\n          id: u\n"
        "      - dig2analog_ultra.calibrate_current:\n          id: u\n          boards: [1]\n          level: 50%\n"
        "      - dig2analog_ultra.cancel_calibration: u\n",
        True,
        None,
    ),
    (
        "ultra calibrate_current sequential (removed: always one board at a time)",
        ULTRA + "button:\n  - platform: template\n    name: c\n    on_press:\n"
        "      - dig2analog_ultra.calibrate_current:\n          id: u\n          sequential: true\n",
        False,
        "[sequential] is an invalid option",
    ),
    (
        "ultra calibrate_current level 3 %",
        ULTRA + "button:\n  - platform: template\n    name: c\n    on_press:\n"
        "      - dig2analog_ultra.calibrate_current:\n          id: u\n          level: 3%\n",
        False,
        "level must be all",
    ),
    (
        "ultra current_calibration schedule",
        ULTRA + "  current_calibration:\n    on_boot: true\n    interval: 7d\n    only_when_off: true\n"
        "    boards: [0, 1]\n",
        True,
        None,
    ),
    (
        "ultra current_calibration schedule with a power supply",
        PSU.format(enable="0ms") + ULTRA + "  power_supply: psu\n  current_calibration:\n    interval: 24h\n",
        True,
        None,
    ),
    (
        "ultra current_calibration every minute",
        ULTRA + "  current_calibration:\n    interval: 1min\n",
        False,
        "value must be at least",
    ),
    (
        "ultra current_calibration board out of range",
        ULTRA + "  current_calibration:\n    on_boot: true\n    boards: [2]\n",
        False,
        "current_calibration: board 2 out of range",
    ),
    (
        "ultra supply with blocking enable_time",
        PSU.format(enable="1500ms") + ULTRA + "  power_supply: psu\n",
        True,
        "enable_time 1500 ms, which blocks the main loop",
    ),
    ("ultra boot_protocol ultra", ULTRA + "  boot_protocol: ultra\n", True, None),
    ("ultra boot_protocol WS2805", ULTRA + "  boot_protocol: WS2805\n", True, None),
    ("ultra boot_protocol bogus", ULTRA + "  boot_protocol: dmx\n", False, "Unknown value"),
    (
        "ultra switch_boards_to_ultra action",
        ULTRA + "button:\n  - platform: template\n    name: s\n    on_press:\n"
        "      - dig2analog_ultra.switch_boards_to_ultra:\n          id: u\n"
        "      - dig2analog_ultra.switch_boards_to_ultra:\n          id: u\n          boot_protocol: ultra\n"
        "      - dig2analog_ultra.switch_boards_to_ultra:\n          id: u\n          boot_protocol: !lambda 'return 0;'\n",
        True,
        None,
    ),
    (
        "ultra switch_boards_to_ultra to WS2805",
        ULTRA + "button:\n  - platform: template\n    name: s\n    on_press:\n"
        "      - dig2analog_ultra.switch_boards_to_ultra:\n          id: u\n          boot_protocol: ws2805\n",
        False,
        "boot_protocol must be auto or ultra",
    ),
    (
        "ws2915 send_config action",
        HUB16
        + "button:\n  - platform: template\n    name: b\n    on_press:\n"
        "      - ws2915.send_config:\n          command: protocol\n          value: 3\n"
        "      - ws2915.send_config:\n          id: d2a\n          chip: 0\n          command: 0x0B\n"
        "          value: !lambda 'return 2;'\n",
        True,
        None,
    ),
    (
        "ws2915 send_config unknown command",
        HUB16 + "button:\n  - platform: template\n    name: b\n    on_press:\n"
        "      - ws2915.send_config:\n          command: brightness\n          value: 3\n",
        False,
        "unknown command 'brightness'",
    ),
    (
        "ws2915 send_config value 256",
        HUB16 + "button:\n  - platform: template\n    name: b\n    on_press:\n"
        "      - ws2915.send_config:\n          command: protocol\n          value: 256\n",
        False,
        "at most 255",
    ),
    (
        "ultra power_on_scene with restore",
        ULTRA + ULTRA_RGBWW + "    restore_mode: RESTORE_DEFAULT_OFF\n    power_on_scene: true\n",
        True,
        None,
    ),
    (
        "ultra power_on_scene ALWAYS_ON initial_state",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1)
        + "    restore_mode: ALWAYS_ON\n    initial_state:\n      brightness: 40%\n    power_on_scene: true\n",
        True,
        None,
    ),
    (
        "ultra power_on_scene with ALWAYS_OFF",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1) + "    power_on_scene: true\n",
        False,
        "power_on_scene shows the state the light comes back in",
    ),
    (
        "ws2915 send_config current limit / calibrate",
        HUB16 + "button:\n  - platform: template\n    name: b\n    on_press:\n"
        "      - ws2915.send_config:\n          command: current_limit\n          value: 120\n"
        "      - ws2915.send_config:\n          command: current_calibrate\n          value: 0\n",
        True,
        None,
    ),
    (
        "ws2915 send_config scene",
        HUB16 + "button:\n  - platform: template\n    name: b\n    on_press:\n"
        "      - ws2915.send_config:\n          command: scene\n          value: 1\n",
        True,
        None,
    ),
    ("ultra dim_to_warm true on cct", ULTRA + ULTRA_CCT + "    dim_to_warm: true\n", True, None),
    (
        "ultra dim_to_warm warm end on cct",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      warm_color_temperature: 2200 K\n",
        True,
        None,
    ),
    (
        "ultra dim_to_warm red_assist on rgbww",
        ULTRA + ULTRA_RGBWW + "    dim_to_warm:\n      warm_color_temperature: 1800 K\n      red_assist: 25%\n",
        True,
        None,
    ),
    (
        "ultra dim_to_warm red_assist on cct",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      warm_color_temperature: 1800 K\n      red_assist: 25%\n",
        False,
        "red_assist needs an RGBCCT light",
    ),
    (
        "ultra dim_to_warm red_assist above the WW LED",
        ULTRA + ULTRA_RGBWW + "    dim_to_warm:\n      warm_color_temperature: 3000 K\n      red_assist: 25%\n",
        False,
        "red_assist only works below the warm white LED",
    ),
    (
        "ultra dim_to_warm on a monochromatic light",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1) + "    dim_to_warm: true\n",
        False,
        "dim_to_warm needs cold_white and warm_white",
    ),
    (
        "ultra dim_to_warm points (Kelvin and mireds)",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      points:\n"
        "      - brightness: 100%\n        color_temperature: 4000 K\n"
        "      - brightness: 20%\n        color_temperature: 3000K\n"
        "      - brightness: 1%\n        color_temperature: 370 mireds\n",
        True,
        None,
    ),
    (
        "ultra dim_to_warm points with red assist",
        ULTRA + ULTRA_RGBWW + "    dim_to_warm:\n      red_assist: 30%\n      points:\n"
        "      - brightness: 100%\n        color_temperature: 2700 K\n"
        "      - brightness: 20%\n        color_temperature: 2200 K\n"
        "      - brightness: 1%\n        color_temperature: 1800 K\n",
        True,
        None,
    ),
    (
        "ultra dim_to_warm points warmer than WW without red_assist",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      points:\n"
        "      - brightness: 10%\n        color_temperature: 2200 K\n",
        False,
        "warmer than the warm white LED needs red_assist",
    ),
    (
        "ultra dim_to_warm points same brightness",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      points:\n"
        "      - brightness: 20%\n        color_temperature: 3000 K\n"
        "      - brightness: 20%\n        color_temperature: 2800 K\n",
        False,
        "two points with the same brightness",
    ),
    (
        "ultra dim_to_warm five points",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      points:\n"
        + "".join(f"      - brightness: {b}%\n        color_temperature: 3000 K\n" for b in (90, 70, 50, 30, 10)),
        False,
        "at most 4",
    ),
    (
        "ultra dim_to_warm point colder than CW",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      points:\n"
        "      - brightness: 50%\n        color_temperature: 8000 K\n",
        False,
        "colder than the cold white LED",
    ),
    (
        "ultra dim_to_warm point at 0 %",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      points:\n"
        "      - brightness: 0%\n        color_temperature: 3000 K\n",
        False,
        "brightness must be above 0 %",
    ),
    (
        "ultra dim_to_warm points and warm_color_temperature",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      warm_color_temperature: 2700 K\n      points:\n"
        "      - brightness: 20%\n        color_temperature: 3000 K\n",
        False,
        "points or warm_color_temperature, not both",
    ),
    (
        "ultra dim_to_warm warm end colder than CW",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      warm_color_temperature: 7000 K\n",
        False,
        "must be warmer than cold_white_color_temperature",
    ),
    (
        "ultra per-channel min_power",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1)
        + "    min_power: {default: 0.1%, ch1: 0.5%}\n    zero_means_zero: true\n",
        True,
        None,
    ),
]

# dig2analog_ultra_strip, effect_glide, RMT memory
STRIP_PIXELS = """\
light:
  - platform: dig2analog_ultra
    id: d1
    name: d1
    board: 0
    channel: 1
  - platform: dig2analog_ultra
    id: d2
    name: d2
    board: 0
    channel: 2
"""

STRIP = """\
  - platform: dig2analog_ultra_strip
    id: s
    name: strip
    lights: [{lights}]
    effects:
      - addressable_scan:
      - strobe:
"""

PARTITION = """\
  - platform: partition
    name: p
{gamma}    segments:
      - id: s
        from: 0
        to: 1
"""


def lines(n, symbols=None):
    out = "dig2analog_ultra:\n"
    for i in range(n):
        out += f"  - id: u{i}\n    pin: GPIO{16 + i}\n    num_boards: 1\n"
        if symbols:
            out += f"    rmt_symbols: {symbols}\n"
    return out


CASES += [
    ("ultra strip", ULTRA + STRIP_PIXELS + STRIP.format(lights="d1, d2"), True, None),
    ("ultra strip effect_glide false", ULTRA + STRIP_PIXELS + STRIP.format(lights="d1, d2") + "    effect_glide: false\n",
     True, None),
    ("ultra strip effect_glide bogus", ULTRA + STRIP_PIXELS + STRIP.format(lights="d1, d2") + "    effect_glide: maybe\n",
     False, "boolean"),
    (
        "ultra strip with a ws2915 light",
        HUB16 + OUT.format(channel=1, chip=0) + ULTRA + STRIP_PIXELS
        + "  - platform: monochromatic\n    id: m\n    name: m\n    output: o1\n" + STRIP.format(lights="d1, m"),
        False,
        "'m' is not a dig2analog_ultra light",
    ),
    ("ultra strip light twice", ULTRA + STRIP_PIXELS + STRIP.format(lights="d1, d1"), False, "'d1' is listed twice"),
    (
        "ultra strip gamma_correct",
        ULTRA + STRIP_PIXELS + STRIP.format(lights="d1, d2") + "    gamma_correct: 2.8\n",
        False,
        "leave gamma_correct at 1.0",
    ),
    (
        "ultra partition over a strip without gamma_correct 1.0",
        ULTRA + STRIP_PIXELS + STRIP.format(lights="d1, d2") + PARTITION.format(gamma=""),
        False,
        "set gamma_correct: 1.0 on the partition",
    ),
    (
        "ultra partition over a strip",
        ULTRA + STRIP_PIXELS + STRIP.format(lights="d1, d2") + PARTITION.format(gamma="    gamma_correct: 1.0\n"),
        True,
        None,
    ),
    (
        "ultra light effect_glide",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1)
        + "    effect_glide: false\n    effects:\n      - flicker:\n      - strobe:\n      - ultra_breathe:\n",
        True,
        None,
    ),
    (
        "ultra white-only light",
        ULTRA + "light:\n  - platform: dig2analog_ultra\n    name: w\n    board: 0\n    white: 4\n",
        True,
        None,
    ),
    ("ultra three lines at rmt_symbols 192", lines(3), False, "need 9 x 64"),
    ("ultra three lines at rmt_symbols 128", lines(3, 128), True, None),
    ("ultra two lines on an esp32-s3 at 192", lines(2), False, "use_dma: true", "esp32-s3-devkitc-1"),
    ("ultra two lines on an esp32-s3 at 96", lines(2, 96), True, None, "esp32-s3-devkitc-1"),
]

# board firmware update entity (update.py)
UPD_NET = "wifi:\n  ssid: x\n  password: xxxxxxxx\nhttp_request:\n"
UPD = "update:\n  - platform: dig2analog_ultra\n    id: fw\n    name: fw\n    dig2analog_ultra_id: u\n"
CASES += [
    ("ultra update minimal", ULTRA + UPD_NET + UPD, True, None),
    (
        "ultra update all options",
        ULTRA + UPD_NET + UPD
        + "    source: quinled/some-firmware\n    channel: beta\n    public_keys: test\n    board_revision: 2\n"
        "    passes: 2\n    update_interval: 1h\n    channel_select:\n      name: fw channel\n"
        "    status:\n      name: fw status\n",
        True,
        None,
    ),
    (
        "ultra update own server without signature",
        ULTRA + UPD_NET + UPD + "    source: http://nas.local/fw/{channel}.json\n    verify_signature: false\n",
        True,
        None,
    ),
    (
        "ultra update unsigned from github",
        ULTRA + UPD_NET + UPD + "    verify_signature: false\n",
        False,
        "only with your own source",
    ),
    ("ultra update bad source", ULTRA + UPD_NET + UPD + "    source: not a repo\n", False, "OWNER/REPO on GitHub"),
    ("ultra update channel bogus", ULTRA + UPD_NET + UPD + "    channel: nightly\n", False, "Unknown value"),
    ("ultra update two per line", ULTRA + UPD_NET + UPD + UPD.replace("id: fw", "id: fw2").replace("name: fw", "name: fw2").replace("update:\n", ""), False, "one per line"),
    ("ultra update without http_request", ULTRA + "wifi:\n  ssid: x\n  password: xxxxxxxx\n" + UPD, False, "http_request"),
    (
        "ultra update actions",
        ULTRA + UPD_NET + UPD
        + "button:\n  - platform: template\n    name: b\n    on_press:\n"
        "      - update.perform:\n          id: fw\n          force_update: true\n"
        "      - dig2analog_ultra.abort_update: fw\n"
        "      - dig2analog_ultra.recover:\n          id: fw\n"
        "      - dig2analog_ultra.recover:\n          id: fw\n          lead_in: 45s\n          power_cycle: false\n",
        True,
        None,
    ),
    (
        "ultra update recover lead_in too long",
        ULTRA + UPD_NET + UPD
        + "button:\n  - platform: template\n    name: b\n    on_press:\n"
        "      - dig2analog_ultra.recover:\n          id: fw\n          lead_in: 1h\n",
        False,
        "value must be at most 10min",
    ),
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
    filters = [f.lower() for f in sys.argv[1:]]
    cases = list(enumerate(CASES))
    cases.append((len(CASES), ("esp32-c2 rejected (no RMT)", HUB16, False, "RMT", "esp32-c2-devkitm-1")))
    if filters:
        cases = [(i, c) for i, c in cases if any(f in c[0].lower() for f in filters)]
    results = [run_case(i, *case) for i, case in cases]
    failed = results.count(False)
    print(f"\n{len(results) - failed}/{len(results)} schema cases passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
