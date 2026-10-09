# esphome-ws2915-ws2805

ESPHome external component for the WorldSemi **WS2915** (16-bit) and **WS2805** (8-bit) 5-channel PWM
driver chips. Every channel becomes a normal ESPHome output, so the stock lights (`rgbww`, `cwww`,
`monochromatic`, …) work on top with gamma, colour temperature, colour interlock, transitions and effects.

Made for the **QuinLED dig2analog** boards, and works with any WS2915/WS2805 hardware.

| Board | Chip | `chip_type` | |
|---|---|---|---|
| [QuinLED dig2analog+](https://quinled.info/quinled-dig2analog-plus/) | WS2805 | `ws2805` | Available now |
| QuinLED dig2analog-5ch (-8b / -16b) | WS2805 / WS2915 | `ws2805` / `ws2915` | In development; will supersede the QuinLED dig2analog+ |

## Install

```yaml
external_components:
  - source: github://intermittech/esphome-ws2915-ws2805@main
    components: [ws2915]
```

## Quick start

Two chained QuinLED boards: an RGBCCT light on the first, and two single-colour lights on the second.

```yaml
ws2915:
  id: d2a
  chip_type: ws2805            # QuinLED dig2analog+ or QuinLED dig2analog-5ch-8b
  # chip_type: ws2915          # QuinLED dig2analog-5ch-16b, together with:
  # gain: 31
  pin: GPIO16                  # to the first board's Data-In
  num_chips: 2                 # boards chained on this line

output:
  # Board 1 (chip: 0, the default)
  - { platform: ws2915, id: out_red,   channel: 1 }
  - { platform: ws2915, id: out_green, channel: 2 }
  - { platform: ws2915, id: out_blue,  channel: 3 }
  - { platform: ws2915, id: out_warm,  channel: 4 }   # W1 = warm white
  - { platform: ws2915, id: out_cold,  channel: 5 }   # W2 = cold white
  # Board 2
  - { platform: ws2915, id: b2_ch1, chip: 1, channel: 1 }
  - { platform: ws2915, id: b2_ch2, chip: 1, channel: 2 }

light:
  - platform: rgbww
    name: RGBCCT
    red: out_red
    green: out_green
    blue: out_blue
    warm_white: out_warm
    cold_white: out_cold
    warm_white_color_temperature: 2700 K
    cold_white_color_temperature: 6500 K
    color_interlock: true
  - platform: monochromatic
    name: Shelf
    output: b2_ch1
  - platform: monochromatic
    name: Cabinet
    output: b2_ch2
```

## Features

- **16-bit dimming** on the WS2915, with ESPHome's gamma keeping the full resolution.
- **Chains of up to 1024 boards** on one data line, each with five channels.
- **Refresh rates:** a re-send at rest, and a higher frame rate during fades and effects.
- **[Power limiter](https://github.com/intermittech/esphome-ws2915-ws2805/wiki/Power-Limiter):** limits per channel, per board (fuse) and per power supply.
- **[Power supplies](https://github.com/intermittech/esphome-ws2915-ws2805/wiki/Power-Supply):** switched without blocking, with a power-on delay for boards that need time to start.

## Documentation

See the **[wiki](https://github.com/intermittech/esphome-ws2915-ws2805/wiki)**:

| | |
|---|---|
| [QuinLED boards](https://github.com/intermittech/esphome-ws2915-ws2805/wiki/QuinLED-Boards) | Settings and examples for the QuinLED dig2analog+ and QuinLED dig2analog-5ch |
| [Configuration](https://github.com/intermittech/esphome-ws2915-ws2805/wiki/Configuration) | All options and the lambda API |
| [Lights and colour](https://github.com/intermittech/esphome-ws2915-ws2805/wiki/Lights-and-Colour) | Interlock, colour temperature, gamma, white order, the WS2805 low end |
| [Refresh and timing](https://github.com/intermittech/esphome-ws2915-ws2805/wiki/Refresh-and-Timing) | Refresh rates, frame times, bit timing |
| [Bench plan](https://github.com/intermittech/esphome-ws2915-ws2805/wiki/Bench-Plan) | What still needs checking on hardware |

Ready-made configurations for the QuinLED dig2analog-5ch are in [`examples/`](examples). The `-8b-` ones
also fit the QuinLED dig2analog+.

> Not the addressable `ws2805` LED-strip component: this one drives the chips as five independent PWM
> channels. The two can be used side by side.

## Status

Compiles and passes its tests for the ESP32 classic (ESP-IDF). The WS2915 still needs checking on real
hardware; see the [bench plan](https://github.com/intermittech/esphome-ws2915-ws2805/wiki/Bench-Plan).

## License

GPL-3.0, the same as the ESPHome C++ core this component compiles into. See [LICENSE](LICENSE).
