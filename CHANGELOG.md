# Changelog

A summary of the changes to the `ws2915` ESPHome component (WS2915 / WS2805), by date. This repository holds the
current state only.

## 2026-11-02

- Docs: in WS mode a QuinLED Dig2Analog-Plus-5ch data pin takes up to 10 boards in a row (longer chains: Ultra mode);
  see the QuinLED boards page.
- Docs: clearer wording.

## 2026-11-01

- Docs and examples use the QuinLED board names: QuinLED Dig2Analog-5ch-8b / -16b, QuinLED Dig2Analog-Plus-5ch (in WS
  mode), QuinLED dig2analog+.
- Example `quinled-dig2analog-5ch-16b-raw-levels.yaml`: raw 16-bit levels, gain and header format at runtime, to find
  the lowest level that lights a strip or the header format of a WS2915.
- `ws2915.send_config`: the command list and values follow the QuinLED Dig2Analog-Plus-5ch firmware 0.7.2
  (`signal_loss` 3 = the stored signal-loss levels; commands 20-22 by number).

## 2026-10-31

- **Chips count from 1**, like `num_chips`: `chip: 1` (the default) is the first board on the line. `chip:` on an
  output, `ws2915.send_config` and the lambdas `set_raw` / `send_config` take 1 .. `num_chips`; `chip: 0` is refused.
  `send_config` without `chip:` goes to every chip.

## 2026-10-30

- Example `quinled-dig2analog-5ch-16b-web-test.yaml`: one slider per channel on the ESP32's own web page.

## 2026-10-12 / 13

- `ws2915.send_config`: board settings (config frames) for boards with their own microcontroller in WS2805 / WS2915
  mode.
- The Ultra-mode components live in [quinled-ultra-esphome](https://github.com/intermittech/quinled-ultra-esphome).

## 2026-10-09

- `refresh_interval`: default `1s` on the WS2805 (a re-send at rest shows no flicker), `never` on the WS2915.
- `power_limit`: in amps only (`channel_current`, `max_channel_current`, `max_chip_current`, `max_current`).
- Docs: a short README; the details are in the wiki.

## 2026-10-08

- First release: the `ws2915:` hub and the `ws2915` output platform for WS2915 (16-bit) and WS2805 (8-bit) chips,
  five channels per chip, up to 1024 chips on a line, on the ESP32's RMT.
- Refresh at rest and during transitions, the WS2915 gain and header format, bit timing options.
- Power limiter per channel, per chip and per line; power supplies switched without blocking, a power-on delay and
  the fade-in replay.
- Examples for the QuinLED Dig2Analog-5ch; host and schema tests.
