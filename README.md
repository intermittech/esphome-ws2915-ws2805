# esphome-ws2915-ws2805

ESPHome external component for the WorldSemi **WS2915** (16-bit, gain header) and **WS2805** (8-bit)
5-channel one-wire PWM drivers. Written for the **dig2analog-5ch** board (one chip per board, boards
chained Data-Out → Data-In), but it works with any WS2915/WS2805 line.

Each chip channel is a normal ESPHome `FloatOutput`, so the stock light platforms (`rgbww`, `cwww`,
`rgbw`, `rgb`, `monochromatic`, …) sit on top. Gamma, colour temperature, interlock, transitions,
effects, restore modes and Home Assistant integration all work the same as with `ledc`.
Transport is the ESP-IDF RMT TX driver with a bit-level encoder (ESP-IDF ≥ 5.3, ESPHome 2026.x).

> This is a separate component from the addressable WS2805 *strip* component (`ws2805`). This one drives
> WS2915/WS2805 chips as 5 independent PWM channels, for example on a driver board. The two can be used
> side by side.

## Install

Add this to your ESPHome YAML:

```yaml
external_components:
  - source: github://intermittech/esphome-ws2915-ws2805@main
    components: [ws2915]
```

- **Pin a version:** replace `@main` with a tag or commit hash.
- **Get the latest:** ESPHome re-downloads at most once a day. Add `refresh: always` next to `components:` to always fetch the latest.

## Minimal config

```yaml
ws2915:
  id: d2a
  chip_type: ws2915             # ws2915 (16-bit) | ws2805 (8-bit)
  pin: GPIO16
  num_chips: 1                  # boards / chips on this line
  gain: 31                      # ws2915 only, see below

output:
  - { platform: ws2915, id: out_red,   channel: 1 }
  - { platform: ws2915, id: out_green, channel: 2 }
  - { platform: ws2915, id: out_blue,  channel: 3 }
  - { platform: ws2915, id: out_cold,  channel: 4 }
  - { platform: ws2915, id: out_warm,  channel: 5 }

light:
  - platform: rgbww
    name: RGBCCT
    red: out_red
    green: out_green
    blue: out_blue
    cold_white: out_cold
    warm_white: out_warm
    cold_white_color_temperature: 6500 K
    warm_white_color_temperature: 2700 K
    color_interlock: true
```

## Where each feature lives

| Feature | Where | Option |
|---|---|---|
| GPIO pin | `ws2915` | `pin` (`inverted: true` for an odd number of inverters on the board) |
| Boards in the chain | `ws2915` | `num_chips`; per output `chip: 0..num_chips-1` (0 = first board) |
| Colour interlock (RGB ↔ CW/WW) | `rgbww` light | `color_interlock: true`. HA then shows separate RGB and colour-temperature modes. |
| CW + WW constant total | `cwww` / `rgbww` light | `constant_brightness: true` |
| White colour temperatures | `cwww` / `rgbww` light | `cold_white_color_temperature`, `warm_white_color_temperature`, in `K` or `mireds` |
| Gamma | any light | `gamma_correct` (default 2.8). ESPHome's 256-point 16-bit table is interpolated, so the WS2915 still gets 16-bit steps. |
| Transitions, effects, restore | any light | `default_transition_length`, `effects`, `restore_mode`, … |
| Refresh at rest | `ws2915` | `refresh_interval` |
| Refresh during transitions/effects | `ws2915` | `transition_refresh_rate` |
| Lowest / highest level | output | `min_power`, `max_power`, `zero_means_zero` (stock) |
| Switch a PSU on/off | output | `power_supply` (stock) |
| Power / current limit | `ws2915` | `power_limit` |

## `ws2915:` options

| Option | Default | |
|---|---|---|
| `id` | – | |
| `chip_type` | `ws2915` | `ws2915` or `ws2805` |
| `pin` | **required** | ESP output pin |
| `num_chips` | **required** | 1–1024 |
| `gain` | **required for ws2915** | 0–31 for all channels, or a map `{red: 31, green: 31, blue: 31, white1: 31, white2: 31}` (keys may also be `r/g/b/w1/w2` or `ch1..ch5`). Rejected for `ws2805`. |
| `header_format` | `32bit` | ws2915 only: `32bit` (datasheet V1.3/V1.5, 7 check bits `0`) or `26bit` (V1.1, 1 check bit `1`) |
| `refresh_interval` | `never` | At rest: re-send the unchanged frame this often (≥ 20 ms). This recovers from a frame corrupted by ESD/EMI. |
| `transition_refresh_rate` | one frame per main loop (16 ms, ~60 fps) | While values change: `200Hz`, `200fps` or a frame interval like `5ms` (1–1000 Hz). Faster than the main loop switches ESPHome's main loop to high-frequency mode while values change, plus 250 ms. |
| `power_limit` | – | see below |
| `bit0_high` / `bit0_low` | 340 / 960 ns | Valid for both chips; out-of-datasheet values give a warning. |
| `bit1_high` / `bit1_low` | 700 / 650 ns | WS2915 T1H 520–1000 ns, WS2805 580–1000 ns |
| `reset_time` | 300 µs | > 280 µs, ≤ 800 µs |
| `rmt_symbols` | 192 (ESP32/S2/S3/P4), 96 (C3/C5/C6/H2) | RMT memory. One WS2915 frame (113 symbols) fits completely on ESP32, so Wi-Fi interrupt latency cannot disturb it. |
| `use_dma` | – | ESP32-S3/P4 only |

### Output (`platform: ws2915`)

| Option | |
|---|---|
| `channel` | `1`–`5` (board CH1–CH5), `ch1`–`ch5`, `red/green/blue/white1/white2` or `r/g/b/w1/w2`. Wire order is R, G, B, W1, W2. |
| `chip` | 0-based position on the line (default 0) |
| `ws2915_id` | hub id if there is more than one line |
| stock | `min_power`, `max_power`, `zero_means_zero`, `inverted`, `power_supply` |

### WS2915 gain

The 5-bit gain per channel (R/G/B/W1/W2) is in the header, and every chip on the line receives the same header. Gain sets the chip's **sink current**: code 0 = 0 mA (output never sinks), 31 ≈ 30 mA (RGB) / 61 mA (W).
- **dig2analog-5ch:** each output only pulls a 3.3 kΩ pull-up (~1.6 mA) low to switch a MOSFET. Use `gain: 31`; anything below ~2 would not pull the gate low.
- **LEDs driven directly:** gain is the LED current.

### Power limiter

```yaml
  power_limit:
    channel_current:     # LED current per channel at 100 % duty, per chip
      red: 3A
      green: 3A
      blue: 3A
      white1: 5A
      white2: 5A         # or one value for all: channel_current: 3A
    max_chip_current: 5A # per chip/board, e.g. below its fuse; only that board is dimmed
    max_current: 10A     # whole line, e.g. the PSU; everything is dimmed uniformly
    # or max_power: 240W + supply_voltage: 24V instead of max_current
```

- **Estimate:** current = Σ channel_current × duty. This is a model, not a measurement. It is computed after gamma, so it follows real duty.
- **Scaling:** when a cap is hit, the frame is scaled at send time. The light's target values stay untouched, so the light comes back to full when the load drops, and colour ratios are kept.
- **Lambdas:**
  - `id(d2a).get_current()`: estimated current after limiting, in A.
  - `id(d2a).get_requested_current()`: current the lights ask for, in A.
  - `id(d2a).is_limiting()`

### Runtime API (lambdas)

| Call | |
|---|---|
| `set_gain(ch, 0..31)`, `get_gain(ch)` | chain-wide WS2915 gain, `ch` 0–4 |
| `set_header_format(ws2915::HEADER_32BIT / HEADER_26BIT)` | for bench tests |
| `set_raw(chip, ch, value)` | raw level (0–65535 / 0–255), bypasses the light until its next update |
| `refresh()` | re-send now |
| `get_current()`, `get_requested_current()`, `is_limiting()`, `get_num_chips()` | |

## Frame time

Each frame is `[32-bit header (WS2915)] [80 bits (WS2915) / 40 bits (WS2805) per chip] [reset 300 µs]`. Times below use the 1.35 µs worst-case bit:

| Chips | WS2915 | WS2805 |
|---|---|---|
| 1 | 451 µs (2200 fps) | 354 µs |
| 10 | 1.42 ms (700 fps) | 0.84 ms |
| 50 | 5.7 ms (174 fps) | 3.0 ms |
| 100 | 11.1 ms (90 fps) | 5.7 ms |

`esphome config` warns when `transition_refresh_rate` is faster than the frame. The WS2915 scans at 2 kHz (V1.5), so refresh rates above ~500 Hz don't add anything.

## Examples (ESP32 classic, ESP-IDF)

| File | |
|---|---|
| [`dig2analog-5ch-16b-rgbcct.yaml`](examples/dig2analog-5ch-16b-rgbcct.yaml) | WS2915, one RGBCCT light with interlock, refresh, power limit, current sensor |
| [`dig2analog-5ch-16b-5x-white.yaml`](examples/dig2analog-5ch-16b-5x-white.yaml) | WS2915, five single-colour lights |
| [`dig2analog-5ch-8b-rgbcct.yaml`](examples/dig2analog-5ch-8b-rgbcct.yaml) | WS2805, one RGBCCT light (temperatures in mireds) |
| [`dig2analog-5ch-8b-3boards-cct.yaml`](examples/dig2analog-5ch-8b-3boards-cct.yaml) | WS2805, three boards, 2 × CCT + 1 white per board, per-board and PSU limits |
| [`dig2analog-5ch-16b-bench.yaml`](examples/dig2analog-5ch-16b-bench.yaml) | WS2915 bench: raw levels, gain and header format adjustable at runtime |

- **Secrets:** copy `examples/secrets.example.yaml` to `examples/secrets.yaml` first.
- **Source:** the examples fetch the component from GitHub. To work on the component itself, switch to the commented local `source:` block.

On 8-bit (WS2805), gamma 2.8 maps the lowest ~10 % of the brightness slider to 0. Use `min_power` on the outputs or a lower `gamma_correct` if that matters.

## Bench plan (open WS2915 points)

Datasheet V1.1 and V1.5 disagree, and nothing here has been seen on real hardware yet. Use the bench example and a scope on the data line and a MOSFET gate.

1. **Header format.** Set `32bit` (default, V1.5 and WorldSemi's configurator) and `26bit` (V1.1). The right one makes the levels follow the raw numbers; the wrong one shifts all data by 6 bits.
2. **Gain mapping.** Check that gain 0 turns an output off (V1.5) and that low gain codes still switch the MOSFET (≥ 2 expected with 3.3 kΩ).
3. **PWM scheme.** Is 2 kHz a single pulse, or dithered? What is the lowest code that visibly lights the strip? That code becomes the suggested `min_power`.
4. **Re-latch.** With `refresh_interval: 1s`, the strip must show no flicker or PWM phase jump on each frame. If it does, use `never` or a longer interval.
5. **Timing margin.** Check that the defaults pass after the board's double inverter, U5 74AHCT1G14 → U3B LVC2G14.

## Tests

```bash
.venv/Scripts/python tests/run_host_test.py      # frame/encoder/limiter, zig C++ (pip install ziglang)
.venv/Scripts/python tests/validate_configs.py   # 32 schema cases via `esphome config`
.venv/Scripts/esphome compile examples/dig2analog-5ch-16b-rgbcct.yaml
```

- **Host test:**
  - Header layout against WorldSemi's formula `R<<27|G<<22|B<<17|W1<<12|W2<<7` (and V1.1 26-bit).
  - The full bit stream through random RMT chunk sizes (1–200 symbols, up to 1024 chips), decoded back.
  - Wire packing.
  - The limiter on 500 random chains.

## Status / limitations

- **Tested:** ESP32 classic (`esp32dev`, ESP-IDF 5.5) compiles. Host and schema tests pass.
- **Not yet verified on hardware:** waveform, header format, PWM behaviour (see bench plan).
- **ESP32-P4:** should work (RMT, `use_dma` allowed) but has not been compiled here yet.
- **ESP32-S31:** needs ESPHome support for the chip first. The code only uses the generic IDF RMT API.
- **ESP32-C2 / C61:** rejected (no RMT).
- **Addressable light framework:** not used on purpose. ESPHome's addressable lights are 3–4 × 8-bit per pixel, so they would throw away the 16 bits and the fifth channel.

## License

GPL-3.0, the same as the ESPHome C++ core this component compiles into. See [LICENSE](LICENSE).
