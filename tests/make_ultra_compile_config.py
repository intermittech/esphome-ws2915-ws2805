"""Writes tests/build/ultra/ultra-example.yaml for a compile test: the Ultra example with the local components and
a few test-only additions (boot protocol, a ws2915 line with a send_config button, the switch_boards_to_ultra
button, the board firmware update with a power supply on the line), so one compile covers both components' new code. Also tests/build/ultra/ultra-strip.yaml: the
dig2analog_ultra_strip example with a partition, effect_glide modes and the downlight notification pattern.

Usage: .venv/Scripts/python tests/make_ultra_compile_config.py
then (PowerShell, ESP-IDF refuses Git Bash):
  & .\\.venv\\Scripts\\esphome.exe compile tests\\build\\ultra\\ultra-example.yaml *> tests\\build\\ultra\\compile.log
"""

from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = HERE / "build" / "ultra"

# the wiki's Home Assistant example (Ultra-Mode.md, "camera mode, PWM frequency and stagger")
HA_GLOBALS = """globals:
  - id: pwm_hz                              # the PWM frequency select's value (camera mode goes back to it)
    type: float
    initial_value: "25000.0f"
"""

HA_SWITCH = """  - platform: template
    id: camera_mode
    name: Camera mode
    icon: mdi:camera
    entity_category: config
    optimistic: true
    restore_mode: RESTORE_DEFAULT_OFF       # after a reboot of the ESP the last choice is sent again
    turn_on_action:
      - lambda: |-
          id(ultra).set_pwm_frequency(-1, 50000.0f);   // preset 50 kHz: banding <= 0.24 % at 1/120 s
          id(ultra).set_dithering(-1, 1);              // off at low levels ...
          id(ultra).set_dither_threshold(-1, 0.03f);   // ... below 3 %: no dither ripple there
    turn_off_action:
      - lambda: |-
          id(ultra).set_pwm_frequency(-1, id(pwm_hz)); // back to the PWM frequency select
          id(ultra).set_dithering(-1, 0);              // and the values in dig2analog_ultra:
          id(ultra).set_dither_threshold(-1, 0.01f);
"""

HA_SELECT = """  - platform: template
    name: PWM frequency
    icon: mdi:pulse
    entity_category: config
    options: ["2.197 kHz", "3.107 kHz", "4.395 kHz", "6.215 kHz", "8.789 kHz", "12.43 kHz", "17.58 kHz", "20 kHz",
              "25 kHz", "30 kHz", "35.16 kHz", "40 kHz", "50 kHz", "70.31 kHz", "100 kHz"]   # the board presets
    initial_option: "25 kHz"                # as pwm_frequency in dig2analog_ultra:
    optimistic: true
    restore_value: true
    on_value:                               # also runs at boot with the restored option
      - lambda: |-
          id(pwm_hz) = atof(x.c_str()) * 1000.0f;      // "12.43 kHz" -> 12430 Hz: that board preset
          if (!id(camera_mode).state)
            id(ultra).set_pwm_frequency(-1, id(pwm_hz));
  - platform: template
    name: Stagger
    icon: mdi:sine-wave
    entity_category: config
    options: ["off", "fixed", "fixed_spread", "adaptive", "adaptive_spread"]   # index = stagger mode 0-4
    initial_option: "adaptive_spread"
    optimistic: true
    restore_value: true
    on_value:                               # also runs at boot with the restored option
      - lambda: id(ultra).set_stagger(-1, i);
"""

EXTRA = """
# ---- compile-test additions (not part of the example)
""" + HA_GLOBALS + """
switch:
""" + HA_SWITCH + """  - platform: template
    name: Dim to warm
    optimistic: true
    restore_mode: RESTORE_DEFAULT_ON
    turn_on_action:
      - lambda: id(rgbcct_out).set_dim_to_warm(true);
    turn_off_action:
      - lambda: id(rgbcct_out).set_dim_to_warm(false);

ws2915:
  id: ws_line
  chip_type: ws2805
  pin: GPIO17
  num_chips: 2

# board firmware update (update.py), with a supply on the line (power hold / power cycle)
http_request:

power_supply:
  - id: led_psu
    pin: GPIO13
    enable_time: 0ms
    keep_on_time: 10s

update:
  - platform: dig2analog_ultra
    id: board_fw
    name: Board firmware
    dig2analog_ultra_id: ultra
    channel: beta
    public_keys: test
    channel_select:
      name: Board firmware channel
    status:
      name: Board firmware status

button:
  - platform: template
    name: Boards on WS2805 line to Ultra
    on_press:
      - ws2915.send_config:
          id: ws_line
          command: protocol
          value: 3
      - ws2915.send_config:
          id: ws_line
          chip: 1
          command: pwm_preset
          value: !lambda "return 8;"
      - ws2915.send_config:
          id: ws_line
          command: current_limit
          value: 120
  - platform: template
    name: Calibrate board 1 at 25 %
    on_press:
      - dig2analog_ultra.calibrate_current:
          id: ultra
          boards: [1]
          level: 25%
      - lambda: |-
          if (id(ultra).calibration_running())
            ESP_LOGI("test", "board %u, %u of %u", id(ultra).calibration_board(),
                     (unsigned) id(ultra).calibration_step(), (unsigned) id(ultra).calibration_count());
          id(ultra).calibrate_current(0);
  - platform: template
    name: Cancel calibration
    on_press:
      - dig2analog_ultra.cancel_calibration: ultra
  # the wiki's example (Ultra-Mode.md, "Boards forced to WS2805 / WS2915")
  - platform: template
    name: Switch boards to Ultra
    icon: mdi:swap-horizontal
    entity_category: config
    on_press:
      - dig2analog_ultra.switch_boards_to_ultra:
          id: ultra
          boot_protocol: auto   # what the boards store afterwards: auto (the default) or ultra
  # board firmware update (update.py): install again, abort, recover
  - platform: template
    name: Board firmware install again
    on_press:
      - update.perform:
          id: board_fw
          force_update: true
  - platform: template
    name: Board firmware abort
    on_press:
      - dig2analog_ultra.abort_update: board_fw
  - platform: template
    name: Board firmware recover
    on_press:
      - dig2analog_ultra.recover:
          id: board_fw
          lead_in: 45s
"""


STRIP_EXTRA = """
  # ---- compile-test additions: a strip across both lines, effect_glide modes, a brightness-only notification strip
  - platform: dig2analog_ultra_strip
    name: Everything
    lights: [front_1, front_2, front_3, side_rgbcct, spot_b, spot_a, side_rgb]
    effect_glide: true
    default_transition_length: 2s
    effects:
      - addressable_fireworks:
      - addressable_flicker:
  - platform: dig2analog_ultra_strip
    id: spots_notify
    name: Spots notification
    internal: true
    lights: [spot_a, spot_b]
    default_transition_length: 400ms
    effects:
      - addressable_lambda:
          name: Chase
          update_interval: 150ms
          lambda: |-
            static int pos = 0;
            if (initial_run)
              pos = 0;
            it.all() = Color::BLACK;
            it[pos % it.size()] = Color::WHITE;
            pos++;

script:
  - id: notify_chase
    mode: restart
    parameters:
      runs: int
    then:
      - light.turn_on:
          id: spots_notify
          brightness: 100%
          effect: Chase
      - delay: !lambda "return runs * 300;"
      - light.turn_off: spots_notify

button:
  - platform: template
    name: Test notification
    on_press:
      - script.execute:
          id: notify_chase
          runs: 3
"""


def strip_config() -> str:
    """tests/build/ultra/ultra-strip.yaml: the strip + partition example, a strip across both lines, effect_glide modes,
    ESPHome effects gliding on a single light, and the downlight notification pattern."""
    src = (ROOT / "examples" / "quinled-dig2analog-ultra-strip-partition.yaml").read_text(encoding="utf-8")
    src = src.replace(
        "  - source: github://intermittech/esphome-ws2915-ws2805@main\n",
        "  - source:\n      type: local\n      path: ../../../components\n",
    )
    assert "path: ../../../components" in src
    # host-side effects on a single light (they glide on the board), a strobe that stays hard, a board effect
    old = "    id: spot_a\n    name: Path spot A\n    board: 0\n    channel: 4\n"
    assert src.count(old) == 1
    src = src.replace(old, old + "    effect_glide: auto\n    effects:\n      - flicker:\n      - strobe:\n"
                      "      - ultra_breathe:\n")
    old = "    lights: [side_rgb, spot_a, spot_b, side_rgbcct]\n"
    assert src.count(old) == 1
    src = src.replace(old, old + "    effect_glide: false\n")
    old = "      - addressable_rainbow:\n          width: 3\n"
    assert src.count(old) == 1
    src = src.replace(old, old + "      - strobe:\n      - flicker:\n")
    # the strips before the partition, the rest at the end
    head, part = src.split("\n  # both lines as one strip", 1)
    return head + STRIP_EXTRA.split("\nscript:\n", 1)[0] + "\n  # both lines as one strip" + part.rstrip() + "\n" \
        + "\nscript:\n" + STRIP_EXTRA.split("\nscript:\n", 1)[1]


def main() -> None:
    src = (ROOT / "examples" / "quinled-dig2analog-ultra-rgbcct.yaml").read_text(encoding="utf-8")
    src = src.replace(
        "  - source: github://intermittech/esphome-ws2915-ws2805@main\n    components: [dig2analog_ultra]",
        "  - source:\n      type: local\n      path: ../../../components\n    components: [dig2analog_ultra, ws2915]",
    )
    src = src.replace("  # boot_protocol: ultra", "  boot_protocol: ultra  #")
    # a supply on the line: the firmware update's power hold / power cycle compile too
    src = src.replace("dig2analog_ultra:\n  id: ultra\n", "dig2analog_ultra:\n  id: ultra\n  power_supply: led_psu\n", 1)
    assert "  power_supply: led_psu\n" in src
    src = src.replace("    # power_on_scene: true", "    power_on_scene: true  #")
    # the hub's own calibration schedule (off in the example: calibration is a manual step)
    src = src.replace("  power_supply: led_psu\n", "  power_supply: led_psu\n  current_calibration:\n    interval: 7d\n"
                      "    only_when_off: true\n", 1)
    assert "  current_calibration:\n    interval: 7d\n" in src
    src = src.replace("    # dim_to_warm:", "    dim_to_warm:  #")
    src = src.replace("    id: rgbcct\n", "    id: rgbcct\n    output_id: rgbcct_out\n", 1)
    assert "output_id: rgbcct_out" in src
    src = src.replace("    #   warm_color_temperature: 2000 K", "      warm_color_temperature: 2000 K")
    src = src.replace("    #   red_assist: 25%", "      red_assist: 25%  #")
    assert "path: ../../../components" in src and "  boot_protocol: ultra  #" in src
    assert "    power_on_scene: true  #" in src
    # the wiki's PWM frequency select replaces the example's own (same name)
    src, n = re.subn(r"  - platform: template\n    name: PWM frequency\n(?:    .*\n|      .*\n)*?    id: pwm_select\n", "", src)
    assert n == 1, "example PWM select"
    src = src.replace("\nselect:\n", "\nselect:\n" + HA_SELECT, 1)
    # dim to warm set points on the example's CCT light
    old = "    cold_white_color_temperature: 5000 K\n    warm_white_color_temperature: 2700 K\n"
    assert src.count(old) == 1
    src = src.replace(old, old + "    dim_to_warm:\n      points:\n        - brightness: 100%\n"
                      "          color_temperature: 4500 K\n        - brightness: 20%\n          color_temperature: 3000K\n"
                      "        - brightness: 1%\n          color_temperature: 370 mireds\n")
    assert "set_stagger(-1, i)" in src
    # the example's own button: list gets the test buttons appended
    src = src.replace("\nbutton:\n", "\nbutton:\n" + EXTRA.split("button:\n", 1)[1], 1)
    src += EXTRA.split("\nbutton:\n", 1)[0] + "\n"
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "ultra-example.yaml").write_text(src, encoding="utf-8")
    (OUT / "ultra-strip.yaml").write_text(strip_config(), encoding="utf-8")
    secrets = OUT / "secrets.yaml"
    if not secrets.exists():
        secrets.write_text('wifi_ssid: "test"\nwifi_password: "testtest"\n', encoding="utf-8")
    print(f"written {OUT / 'ultra-example.yaml'}, {OUT / 'ultra-strip.yaml'}")


if __name__ == "__main__":
    main()
