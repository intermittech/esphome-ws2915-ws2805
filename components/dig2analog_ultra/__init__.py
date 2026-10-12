"""QuinLED dig2analog boards in Ultra mode: the boards run transitions, gamma and effects themselves.

The hub sends commands on one data line (WS2915-style one-wire pulses, ESP-IDF RMT). Lights are the
`dig2analog_ultra` light platform (light.py). Board settings are host-managed: the values here are sent
to every board at boot and repeated, and never stored in the boards' flash (except `boot_protocol`).
"""

import logging

from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import esp32, esp32_rmt, power_supply
from esphome.components.esp32 import include_builtin_idf_component
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.const import (
    CONF_BLUE,
    CONF_ENABLE_TIME,
    CONF_GREEN,
    CONF_ID,
    CONF_INVERTED,
    CONF_KEEP_ON_TIME,
    CONF_NUMBER,
    CONF_PIN,
    CONF_POWER_SUPPLY,
    CONF_RED,
    CONF_RMT_SYMBOLS,
    CONF_USE_DMA,
)
from esphome.core import CORE
from esphome.types import ConfigType

_LOGGER = logging.getLogger(__name__)

DEPENDENCIES = ["esp32"]
MULTI_CONF = True

dig2analog_ultra_ns = cg.esphome_ns.namespace("dig2analog_ultra")
UltraHub = dig2analog_ultra_ns.class_("UltraHub", cg.Component)
SwitchBoardsToUltraAction = dig2analog_ultra_ns.class_("SwitchBoardsToUltraAction", automation.Action)
CalibrateCurrentAction = dig2analog_ultra_ns.class_("CalibrateCurrentAction", automation.Action)
CancelCalibrationAction = dig2analog_ultra_ns.class_("CancelCalibrationAction", automation.Action)

CONF_DIG2ANALOG_ULTRA_ID = "dig2analog_ultra_id"
CONF_NUM_BOARDS = "num_boards"
CONF_BIT0_HIGH = "bit0_high"
CONF_BIT0_LOW = "bit0_low"
CONF_BIT1_HIGH = "bit1_high"
CONF_BIT1_LOW = "bit1_low"
CONF_RESET_TIME = "reset_time"
CONF_REFRESH_INTERVAL = "refresh_interval"
CONF_START_DELAY = "start_delay"
CONF_REPEATS = "repeats"
CONF_POWER_ON_DELAY = "power_on_delay"
CONF_PWM_FREQUENCY = "pwm_frequency"
CONF_DITHERING = "dithering"
CONF_DITHER_THRESHOLD = "dither_threshold"
CONF_DIM_FLOOR = "dim_floor"
CONF_MIN_PULSE = "min_pulse"
CONF_STAGGER = "stagger"
CONF_SIGNAL_LOSS = "signal_loss"
CONF_SIGNAL_LOSS_TIMEOUT = "signal_loss_timeout"
CONF_FEEDBACK_CHANNEL = "feedback_channel"
CONF_POWER_LIMIT = "power_limit"
CONF_CHANNEL_CURRENT = "channel_current"
CONF_MAX_CHANNEL_CURRENT = "max_channel_current"
CONF_MAX_BOARD_CURRENT = "max_board_current"
CONF_WHITE1 = "white1"
CONF_WHITE2 = "white2"
CONF_DEFAULT = "default"
CONF_BOOT_PROTOCOL = "boot_protocol"
CONF_CURRENT_CALIBRATION = "current_calibration"
CONF_ON_BOOT = "on_boot"
CONF_INTERVAL = "interval"
CONF_ONLY_WHEN_OFF = "only_when_off"
CONF_BOARDS = "boards"
CONF_LEVEL = "level"

# light platform keys that take a channel, and their roles (ULTRA_ROLE_*)
ROLE_KEYS = ["channel", "red", "green", "blue", "white", "cold_white", "warm_white"]
ROLES = {"channel": 1, "red": 2, "green": 3, "blue": 4, "white": 5, "cold_white": 6, "warm_white": 7}

# Board terminals CH1..CH5 = channel index 0..4 (data order R, G, B, W1, W2 in WS mode).
CHANNEL_KEYS = [CONF_RED, CONF_GREEN, CONF_BLUE, CONF_WHITE1, CONF_WHITE2]
CHANNEL_ALIASES = {key: idx for idx, key in enumerate(CHANNEL_KEYS)}
CHANNEL_ALIASES.update({"r": 0, "g": 1, "b": 2, "w1": 3, "w2": 4})
CHANNEL_ALIASES.update({f"ch{i + 1}": i for i in range(5)})

# ULTRA_CFG_* ids (ultra_protocol.h)
CFG_PWM_PRESET = 1
CFG_PWM_PERIOD = 2
CFG_DITHER = 3
CFG_DITHER_THRESH = 4
CFG_DIM_FLOOR = 5
CFG_MIN_PULSE = 6
CFG_STAGGER = 7
CFG_LOSS = 8
CFG_LOSS_TIMEOUT = 9
CFG_FEEDBACK = 10
CFG_CH_CURRENT = 11
CFG_CH_MAX = 16
CFG_BOARD_MAX = 21
CFG_BOOT_PROTOCOL = 22  # persisted on the board

DITHERING = {"on": 0, "off_at_low_levels": 1, "off": 2}
DIM_FLOOR = {"skip": 0, "clamp": 1, "off": 2}
SIGNAL_LOSS = {"hold": 0, "off": 1, "fade_off": 2}
FEEDBACK = {"red": 0, "all": 1, "off": 2}
# each mode followed by its spread-spectrum variant; "spread" = adaptive_spread
STAGGER = {"off": 0, "fixed": 1, "fixed_spread": 2, "adaptive": 3, "adaptive_spread": 4, "spread": 4}
BOOT_PROTOCOL = {"auto": 0, "ws2915": 1, "ws2805": 2, "ultra": 3}


def _validate_stagger(value):
    """off / fixed / fixed_spread / adaptive / adaptive_spread (alias spread); true = fixed, false = off."""
    if isinstance(value, bool):
        return "fixed" if value else "off"
    return cv.one_of(*STAGGER, lower=True)(value)


def light_roles(config):
    """(channel 0..4, role) of a dig2analog_ultra light. A light of only `white:` is monochromatic: its channel follows
    the brightness (ESPHome gives a monochromatic light no white value)."""
    keys = [k for k in ROLE_KEYS if k in config]
    return [(config[k], ROLES["channel"] if keys == ["white"] else ROLES[k]) for k in keys]


# effect_glide (lights and strips): how the steps of ESPHome's own effects reach the board. UltraLight EffectGlide.
EFFECT_GLIDE_MODES = {False: 0, True: 1, "auto": 2}
# ESPHome effects whose steps stay hard (commands, no glide) with effect_glide: auto
HARD_EFFECTS = {"strobe"}


def validate_effect_glide(value):
    """auto (small steps glide, jumps up at once, down with a short tail; strobe hard), true (every step glides over
    the step interval) or false (hard steps)."""
    if isinstance(value, str) and value.strip().lower() == "auto":
        return "auto"
    return cv.boolean(value)


def validate_channel(value):
    """1..5 (board terminals CH1..CH5), ch1..ch5, red/green/blue/white1/white2 or r/g/b/w1/w2 -> 0..4."""
    if isinstance(value, int) and not isinstance(value, bool):
        return cv.int_range(min=1, max=5)(value) - 1
    key = cv.string_strict(value).lower()
    if key.isdigit():
        return cv.int_range(min=1, max=5)(int(key)) - 1
    if key not in CHANNEL_ALIASES:
        raise cv.Invalid(f"unknown channel '{value}' (use 1-5, {', '.join(CHANNEL_ALIASES)})")
    return CHANNEL_ALIASES[key]


def _channel_map(value_validator, what, require_all=True):
    """One value for every channel, or a mapping channel -> value (with an optional `default:`)."""

    def validator(value):
        if not isinstance(value, dict):
            v = value_validator(value)
            return {k: v for k in CHANNEL_KEYS}
        out = {}
        default = None
        for raw_key, raw_val in value.items():
            if str(raw_key).lower() == CONF_DEFAULT:
                default = value_validator(raw_val)
                continue
            try:
                name = CHANNEL_KEYS[validate_channel(raw_key)]
            except cv.Invalid as err:
                raise cv.Invalid(str(err), [raw_key]) from err
            if name in out:
                raise cv.Invalid(f"channel '{name}' given twice", [raw_key])
            out[name] = value_validator(raw_val)
        for k in CHANNEL_KEYS:
            if k not in out and default is not None:
                out[k] = default
        missing = [k for k in CHANNEL_KEYS if k not in out]
        if missing and require_all:
            raise cv.Invalid(f"{what}: missing channel(s) {', '.join(missing)} (or give a default:)")
        return {k: out[k] for k in CHANNEL_KEYS if k in out}

    return validator


_PWM_MIN_HZ = 144e6 / 65535
_PWM_MAX_HZ = 100000.0


def _validate_pwm_frequency(value):
    hz = cv.frequency(value)
    if not _PWM_MIN_HZ * 0.999 <= hz <= _PWM_MAX_HZ * 1.001:
        raise cv.Invalid(f"pwm_frequency must be {_PWM_MIN_HZ:.1f} Hz - 100 kHz")
    return hz


def _validate_reset(value):
    value = cv.positive_time_period_nanoseconds(value)
    us = value.total_nanoseconds / 1000
    if us <= 280:
        raise cv.Invalid("reset_time must be > 280us (WS28xx reset minimum)")
    if us > 800:
        raise cv.Invalid("reset_time above 800us does not fit one RMT symbol")
    return value


def _validate_min_pulse(value):
    value = cv.positive_time_period_nanoseconds(value)
    ns = value.total_nanoseconds
    if not 50 <= ns <= 1000:
        raise cv.Invalid("min_pulse must be 50ns - 1us")
    return value


def _validate_dither_threshold(value):
    v = cv.percentage(value)
    if not 0.001 <= v <= 0.255:
        raise cv.Invalid("dither_threshold must be 0.1% - 25.5%")
    return v


POWER_LIMIT_SCHEMA = cv.Schema(
    {
        # LED current of each channel at 100 % (same on every board of this line). Needed for max_channel_current
        # and for boards without current sense; boards with it measure the board current (max_board_current).
        cv.Optional(CONF_CHANNEL_CURRENT): _channel_map(cv.current, "channel_current"),
        # Limit per channel (e.g. its terminal / MOSFET rating); only that channel is held back.
        cv.Optional(CONF_MAX_CHANNEL_CURRENT): _channel_map(
            cv.All(cv.current, cv.Range(min=0.01)), "max_channel_current", require_all=False
        ),
        # Limit per board (e.g. below its fuse); all its channels are dimmed together. Boards with the input
        # current sense hold their measured current to it (without channel_current); the others estimate it.
        cv.Optional(CONF_MAX_BOARD_CURRENT): cv.All(cv.current, cv.Range(min=0.01, max=65.0)),
    }
)


def _validate_power_limit(value):
    value = POWER_LIMIT_SCHEMA(value)
    if CONF_MAX_CHANNEL_CURRENT not in value and CONF_MAX_BOARD_CURRENT not in value:
        raise cv.Invalid("power_limit needs max_channel_current and/or max_board_current")
    if CONF_MAX_CHANNEL_CURRENT in value and CONF_CHANNEL_CURRENT not in value:
        raise cv.Invalid("max_channel_current needs channel_current")
    return value


def _validate_cal_level(value):
    """all (the standard test: each channel at 25 / 50 / 75 / 100 %) or one duty 5-100 % -> 0 / 5..100."""
    if isinstance(value, str) and value.strip().lower() == "all":
        return 0
    pct = round(cv.percentage(value) * 100)
    if not 5 <= pct <= 100:
        raise cv.Invalid("level must be all, or 5% - 100%")
    return pct


# Boards with the input current sense: when the hub runs the calibration by itself. Normally it is a manual step (the
# action below), after installing or changing what is connected. A run darkens the pin and tests one board after the
# other (~1.1 s each, visible), so nothing runs unless configured.
CURRENT_CALIBRATION_SCHEMA = cv.Schema(
    {
        # once, 10 s after the ESP boots (with only_when_off: once the lights are off)
        cv.Optional(CONF_ON_BOOT, default=False): cv.boolean,
        # periodically (from boot on); a run that falls due while a light is on waits for them to go off
        cv.Optional(CONF_INTERVAL): cv.All(
            cv.positive_time_period_milliseconds,
            cv.Range(min=cv.TimePeriod(minutes=10), max=cv.TimePeriod(days=30)),
        ),
        # automatic runs only while every light on this line is off (with a power_supply it is switched on for the
        # run). false: also while they are on (the pin goes dark for the run, then fades back)
        cv.Optional(CONF_ONLY_WHEN_OFF, default=True): cv.boolean,
        # which boards (0-based) to test, default all (the whole pin goes dark either way)
        cv.Optional(CONF_BOARDS): cv.ensure_list(cv.int_range(min=0, max=63)),
    }
)

CONFIG_SCHEMA = cv.All(
    esp32.only_on_variant(
        unsupported=list(esp32_rmt.VARIANTS_NO_RMT),
        msg_prefix="dig2analog_ultra (RMT)",
    ),
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(UltraHub),
            cv.Required(CONF_PIN): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_NUM_BOARDS): cv.int_range(min=1, max=64),
            cv.Optional(CONF_BIT0_HIGH, default="340ns"): cv.positive_time_period_nanoseconds,
            cv.Optional(CONF_BIT0_LOW, default="960ns"): cv.positive_time_period_nanoseconds,
            cv.Optional(CONF_BIT1_HIGH, default="700ns"): cv.positive_time_period_nanoseconds,
            cv.Optional(CONF_BIT1_LOW, default="650ns"): cv.positive_time_period_nanoseconds,
            cv.Optional(CONF_RESET_TIME, default="300us"): _validate_reset,
            # At rest a frame this often: keeps the boards' clocks in step and repeats the current state.
            cv.Optional(CONF_REFRESH_INTERVAL, default="200ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(milliseconds=20), max=cv.TimePeriod(seconds=10)),
            ),
            # Commands take effect this long after they are sent, so all copies and boards start together.
            cv.Optional(CONF_START_DELAY, default="30ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(max=cv.TimePeriod(milliseconds=1000)),
            ),
            # Copies of every command (a corrupted frame does not lose it).
            cv.Optional(CONF_REPEATS, default=3): cv.int_range(min=1, max=8),
            # Board settings (sent to every board; runtime setters change them per board)
            cv.Optional(CONF_PWM_FREQUENCY, default="25kHz"): _validate_pwm_frequency,
            cv.Optional(CONF_DITHERING, default="on"): cv.enum(DITHERING, lower=True),
            cv.Optional(CONF_DITHER_THRESHOLD, default="1.0%"): _validate_dither_threshold,
            cv.Optional(CONF_DIM_FLOOR, default="skip"): cv.enum(DIM_FLOOR, lower=True),
            cv.Optional(CONF_MIN_PULSE, default="200ns"): _validate_min_pulse,
            # off, fixed (channels a third of a period apart), fixed_spread, adaptive (phases follow the colour mix
            # for the least LED_+ ripple current; uses power_limit's channel_current as weights when given),
            # adaptive_spread (alias spread); *_spread adds spread-spectrum PWM (every period +-6 %, conducted EMC)
            cv.Optional(CONF_STAGGER, default="adaptive_spread"): _validate_stagger,
            cv.Optional(CONF_SIGNAL_LOSS, default="hold"): cv.enum(SIGNAL_LOSS, lower=True),
            cv.Optional(CONF_SIGNAL_LOSS_TIMEOUT, default="3s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(milliseconds=500), max=cv.TimePeriod(seconds=60)),
            ),
            cv.Optional(CONF_FEEDBACK_CHANNEL, default="red"): cv.enum(FEEDBACK, lower=True),
            cv.Optional(CONF_POWER_LIMIT): _validate_power_limit,
            cv.Optional(CONF_CURRENT_CALIBRATION): CURRENT_CALIBRATION_SCHEMA,
            # Protocol mode the boards start in after power-up; stored on the boards (the only setting that is),
            # written only when it differs. Not given: the boards keep what they have (default auto).
            cv.Optional(CONF_BOOT_PROTOCOL): cv.enum(BOOT_PROTOCOL, lower=True),
            # Supply of the boards / LEDs: requested while a light is on; the boards' state is sent again
            # power_on_delay after it switched on (fades that ran in the dark are replayed).
            cv.Optional(CONF_POWER_SUPPLY): cv.use_id(power_supply.PowerSupply),
            cv.Optional(CONF_POWER_ON_DELAY, default="100ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(max=cv.TimePeriod(seconds=30)),
            ),
            cv.SplitDefault(
                CONF_RMT_SYMBOLS,
                esp32=192,
                esp32_c3=96,
                esp32_c5=96,
                esp32_c6=96,
                esp32_h2=96,
                esp32_p4=192,
                esp32_s2=192,
                esp32_s3=192,
            ): cv.int_range(min=2),
            cv.Optional(CONF_USE_DMA): cv.All(
                esp32.only_on_variant(supported=[esp32.VARIANT_ESP32P4, esp32.VARIANT_ESP32S3]),
                cv.boolean,
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
)


def _lights_of(full_config, hub_id):
    return [
        light
        for light in full_config.get("light", []) or []
        if light.get("platform") == "dig2analog_ultra" and light.get(CONF_DIG2ANALOG_ULTRA_ID) == hub_id
    ]


# RMT memory for transmitting per variant (ESP-IDF soc_caps): TX channels x symbols per memory block. A channel
# without DMA takes rmt_symbols / block size blocks, so the default 192 leaves room for 2 lines on an ESP32, 1 on an
# ESP32-S3 / C3 / C6.
RMT_TX_MEMORY = {
    esp32.VARIANT_ESP32: (8, 64),
    esp32.VARIANT_ESP32S2: (4, 64),
    esp32.VARIANT_ESP32S3: (4, 48),
    esp32.VARIANT_ESP32C3: (2, 48),
    esp32.VARIANT_ESP32C5: (2, 48),
    esp32.VARIANT_ESP32C6: (2, 48),
    esp32.VARIANT_ESP32H2: (2, 48),
    esp32.VARIANT_ESP32P4: (4, 48),
}


def _validate_rmt_memory(full):
    """The data lines (and other RMT transmitters) must fit the RMT memory, or the later ones fail at boot."""
    variant = esp32.get_esp32_variant()
    if variant not in RMT_TX_MEMORY:
        return
    channels, block = RMT_TX_MEMORY[variant]
    hubs = full.get("dig2analog_ultra", []) or []
    hubs = hubs if isinstance(hubs, list) else [hubs]
    users = [(f"dig2analog_ultra '{h[CONF_ID]}'", h) for h in hubs]
    users += [
        (f"light '{c.get('name', c[CONF_ID])}'", c)
        for c in full.get("light", []) or []
        if c.get("platform") == "esp32_rmt_led_strip"
    ]
    users += [(f"remote_transmitter '{c[CONF_ID]}'", c) for c in full.get("remote_transmitter", []) or []]
    need = 0
    for _, c in users:
        symbols = c.get(CONF_RMT_SYMBOLS, block)
        need += 1 if c.get(CONF_USE_DMA) else -(-symbols // block)
    if need > channels:
        names = ", ".join(name for name, _ in users)
        raise cv.Invalid(
            f"the RMT of the {variant} has {channels} x {block} symbols for transmitting; {names} need "
            f"{need} x {block}. Give the data lines a smaller rmt_symbols (e.g. {block} or {2 * block}; each line "
            f"takes rmt_symbols / {block} blocks)"
            + (" or use_dma: true" if variant in (esp32.VARIANT_ESP32S3, esp32.VARIANT_ESP32P4) else "")
        )


def _final_validate(configs):
    full = fv.full_config.get()
    _validate_rmt_memory(full)
    for conf in configs if isinstance(configs, list) else [configs]:
        used = {}
        count = {}
        for light in _lights_of(full, conf[CONF_ID]):
            board = light["board"]
            if board >= conf[CONF_NUM_BOARDS]:
                raise cv.Invalid(
                    f"light '{light.get('name', light[CONF_ID])}': board {board} out of range, "
                    f"'{conf[CONF_ID]}' has num_boards: {conf[CONF_NUM_BOARDS]} (boards are 0-based)"
                )
            count[board] = count.get(board, 0) + 1
            for ch in (light[k] for k in ROLE_KEYS if k in light):
                key = (board, ch)
                if key in used:
                    raise cv.Invalid(
                        f"board {board} CH{ch + 1} is used by both '{used[key]}' and "
                        f"'{light.get('name', light[CONF_ID])}'"
                    )
                used[key] = light.get("name", str(light[CONF_ID]))
        for b in conf.get(CONF_CURRENT_CALIBRATION, {}).get(CONF_BOARDS, []):
            if b >= conf[CONF_NUM_BOARDS]:
                raise cv.Invalid(
                    f"current_calibration: board {b} out of range, '{conf[CONF_ID]}' has num_boards: "
                    f"{conf[CONF_NUM_BOARDS]} (boards are 0-based)"
                )
        if CONF_POWER_SUPPLY in conf:
            for psu in full.get("power_supply", []) or []:
                if psu[CONF_ID] == conf[CONF_POWER_SUPPLY] and psu[CONF_ENABLE_TIME].total_milliseconds > 50:
                    _LOGGER.warning(
                        "dig2analog_ultra: power_supply '%s' has enable_time %d ms, which blocks the main loop "
                        "on every switch-on; set enable_time: 0ms and use power_on_delay instead",
                        conf[CONF_POWER_SUPPLY],
                        psu[CONF_ENABLE_TIME].total_milliseconds,
                    )
    # ultra_* effects only run on dig2analog_ultra lights
    for light in full.get("light", []) or []:
        if light.get("platform") == "dig2analog_ultra":
            continue
        for eff in light.get("effects", []) or []:
            key = next(iter(eff))
            if key.startswith("ultra_"):
                raise cv.Invalid(
                    f"light '{light.get('name', light[CONF_ID])}': effect {key} only runs on dig2analog_ultra lights"
                )
    return configs


FINAL_VALIDATE_SCHEMA = _final_validate


def _keep_on_time_ms(full_config, psu_id) -> int:
    for conf in full_config.get("power_supply", []) or []:
        if conf[CONF_ID] == psu_id:
            return int(conf[CONF_KEEP_ON_TIME].total_milliseconds)
    return 0


async def to_code(config: ConfigType) -> None:
    include_builtin_idf_component("esp_driver_rmt")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_pin(config[CONF_PIN][CONF_NUMBER]))
    if config[CONF_PIN][CONF_INVERTED]:
        cg.add(var.set_inverted(True))
    cg.add(var.set_num_boards(config[CONF_NUM_BOARDS]))
    cg.add(var.set_rmt_symbols(config[CONF_RMT_SYMBOLS]))
    if CONF_USE_DMA in config:
        cg.add(var.set_use_dma(config[CONF_USE_DMA]))
    cg.add(
        var.set_timing(
            int(config[CONF_BIT0_HIGH].total_nanoseconds),
            int(config[CONF_BIT0_LOW].total_nanoseconds),
            int(config[CONF_BIT1_HIGH].total_nanoseconds),
            int(config[CONF_BIT1_LOW].total_nanoseconds),
            int(config[CONF_RESET_TIME].total_nanoseconds),
        )
    )
    cg.add(var.set_refresh_interval(config[CONF_REFRESH_INTERVAL].total_milliseconds))
    cg.add(var.set_start_delay(config[CONF_START_DELAY].total_milliseconds))
    cg.add(var.set_repeats(config[CONF_REPEATS]))

    # board settings: all boards (-1), sent at boot
    cg.add(var.set_pwm_frequency(-1, config[CONF_PWM_FREQUENCY]))
    cg.add(var.set_board_config(-1, CFG_DITHER, DITHERING[config[CONF_DITHERING]]))
    cg.add(var.set_board_config(-1, CFG_DITHER_THRESH, round(config[CONF_DITHER_THRESHOLD] * 1000)))
    cg.add(var.set_board_config(-1, CFG_DIM_FLOOR, DIM_FLOOR[config[CONF_DIM_FLOOR]]))
    cg.add(var.set_board_config(-1, CFG_MIN_PULSE, round(config[CONF_MIN_PULSE].total_nanoseconds / 10)))
    cg.add(var.set_board_config(-1, CFG_STAGGER, STAGGER[config[CONF_STAGGER]]))
    cg.add(var.set_board_config(-1, CFG_LOSS, SIGNAL_LOSS[config[CONF_SIGNAL_LOSS]]))
    cg.add(var.set_board_config(-1, CFG_LOSS_TIMEOUT, round(config[CONF_SIGNAL_LOSS_TIMEOUT].total_milliseconds / 100)))
    cg.add(var.set_board_config(-1, CFG_FEEDBACK, FEEDBACK[config[CONF_FEEDBACK_CHANNEL]]))
    plim = config.get(CONF_POWER_LIMIT, {})
    for idx, key in enumerate(CHANNEL_KEYS):
        amps = plim.get(CONF_CHANNEL_CURRENT, {}).get(key, 0.0)
        cg.add(var.set_board_config(-1, CFG_CH_CURRENT + idx, round(amps * 1000)))
        limit = plim.get(CONF_MAX_CHANNEL_CURRENT, {}).get(key, 0.0)
        cg.add(var.set_board_config(-1, CFG_CH_MAX + idx, round(limit * 1000)))
    cg.add(var.set_board_config(-1, CFG_BOARD_MAX, round(plim.get(CONF_MAX_BOARD_CURRENT, 0.0) * 1000)))
    if CONF_BOOT_PROTOCOL in config:
        cg.add(var.set_board_config(-1, CFG_BOOT_PROTOCOL, BOOT_PROTOCOL[config[CONF_BOOT_PROTOCOL]]))

    if CONF_POWER_SUPPLY in config:
        psu = await cg.get_variable(config[CONF_POWER_SUPPLY])
        cg.add(var.set_power_supply(psu, _keep_on_time_ms(CORE.config, config[CONF_POWER_SUPPLY])))
        cg.add(var.set_power_on_delay(config[CONF_POWER_ON_DELAY].total_milliseconds))
    if CONF_CURRENT_CALIBRATION in config:
        cal = config[CONF_CURRENT_CALIBRATION]
        cg.add(
            var.set_calibration_schedule(
                cal[CONF_ON_BOOT],
                int(cal[CONF_INTERVAL].total_milliseconds) if CONF_INTERVAL in cal else 0,
                cal[CONF_ONLY_WHEN_OFF],
                cal.get(CONF_BOARDS, []),
            )
        )


# ------------------------------------------------------------------------------- switch WS-forced boards to Ultra
SWITCH_BOOT_PROTOCOL = {"auto": 0, "ultra": 3}


def _validate_switch_boot_protocol(value):
    """auto or ultra: what the boards store as their boot protocol after the switch."""
    key = cv.string_strict(value).lower()
    if key not in SWITCH_BOOT_PROTOCOL:
        raise cv.Invalid(f"boot_protocol must be auto or ultra, not '{value}'")
    return SWITCH_BOOT_PROTOCOL[key]


@automation.register_action(
    "dig2analog_ultra.switch_boards_to_ultra",
    SwitchBoardsToUltraAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(UltraHub),
            cv.Optional(CONF_BOOT_PROTOCOL, default="auto"): cv.templatable(_validate_switch_boot_protocol),
        }
    ),
    synchronous=True,
)
async def switch_boards_to_ultra_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_boot_protocol(await cg.templatable(config[CONF_BOOT_PROTOCOL], args, cg.uint8)))
    return var


# ------------------------------------------------------------------------------- current calibration on demand
# The pin goes dark, its boards are tested one after the other in chain order (~1.1 s each), then the pin fades back.
# Other pins keep running.
@automation.register_action(
    "dig2analog_ultra.calibrate_current",
    CalibrateCurrentAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(UltraHub),
            # which boards (0-based) to test, default all on this pin (the whole pin goes dark either way)
            cv.Optional(CONF_BOARDS): cv.ensure_list(cv.int_range(min=0, max=63)),
            # all = the standard test (each channel at 25 / 50 / 75 / 100 %); or one level, e.g. 25%
            cv.Optional(CONF_LEVEL, default="all"): _validate_cal_level,
        }
    ),
    synchronous=True,
)
async def calibrate_current_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_boards(config.get(CONF_BOARDS, [])))
    cg.add(var.set_level(config[CONF_LEVEL]))
    return var


@automation.register_action(
    "dig2analog_ultra.cancel_calibration",
    CancelCalibrationAction,
    automation.maybe_simple_id({cv.GenerateID(): cv.use_id(UltraHub)}),
    synchronous=True,
)
async def cancel_calibration_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
