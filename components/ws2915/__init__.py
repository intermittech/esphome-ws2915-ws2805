"""WorldSemi WS2915 (16-bit) / WS2805 (8-bit) 5-channel one-wire PWM driver hub.

Each chip is exposed as five FloatOutputs (see output.py), so the stock rgbww / rgbct /
cwww / monochromatic lights work on top of it. Transport is the ESP-IDF RMT TX driver
with a bit-level simple encoder (ESP-IDF >= 5.3).
"""

import logging
import re

from esphome import pins
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
    CONF_MAX_CURRENT,
    CONF_MAX_POWER,
    CONF_NUM_CHIPS,
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
AUTO_LOAD = ["output"]
MULTI_CONF = True

ws2915_ns = cg.esphome_ns.namespace("ws2915")
WS2915Component = ws2915_ns.class_("WS2915Component", cg.Component)

ChipType = ws2915_ns.enum("ChipType")
CHIP_TYPES = {
    "ws2915": ChipType.CHIP_WS2915,  # 16-bit, gain header
    "ws2805": ChipType.CHIP_WS2805,  # 8-bit, no header; pin-compatible except pin 8
}

HeaderFormat = ws2915_ns.enum("HeaderFormat")
HEADER_FORMATS = {
    # Datasheet V1.3 (CN, 2025-06) / V1.5 (EN, 2026-07) and WorldSemi's configurator:
    # R5 G5 B5 W1-5 W2-5 + 7 check bits "0" = 32 bits.
    "32bit": HeaderFormat.HEADER_32BIT,
    # Datasheet V1.1 (2024-12): R5 G5 B5 W1-5 W2-5 + 1 check bit "1" = 26 bits.
    "26bit": HeaderFormat.HEADER_26BIT,
}

CONF_CHIP_TYPE = "chip_type"
CONF_HEADER_FORMAT = "header_format"
CONF_GAIN = "gain"
CONF_WHITE1 = "white1"
CONF_WHITE2 = "white2"
CONF_BIT0_HIGH = "bit0_high"
CONF_BIT0_LOW = "bit0_low"
CONF_BIT1_HIGH = "bit1_high"
CONF_BIT1_LOW = "bit1_low"
CONF_RESET_TIME = "reset_time"
CONF_REFRESH_INTERVAL = "refresh_interval"
CONF_TRANSITION_REFRESH_RATE = "transition_refresh_rate"
CONF_POWER_LIMIT = "power_limit"
CONF_CHANNEL_CURRENT = "channel_current"
CONF_MAX_CHIP_CURRENT = "max_chip_current"
CONF_MAX_CHANNEL_CURRENT = "max_channel_current"
CONF_SUPPLY_VOLTAGE = "supply_voltage"
CONF_POWER_ON_DELAY = "power_on_delay"
CONF_WS2915_ID = "ws2915_id"

# Order = header/data order in the datasheets (R, G, B, W1, W2) = C++ channel index.
CHANNEL_KEYS = [CONF_RED, CONF_GREEN, CONF_BLUE, CONF_WHITE1, CONF_WHITE2]

# Accepted channel names -> index. Datasheet pin names and board terminal numbers too.
CHANNEL_ALIASES = {key: idx for idx, key in enumerate(CHANNEL_KEYS)}
CHANNEL_ALIASES.update({"r": 0, "g": 1, "b": 2, "w1": 3, "w2": 4})
CHANNEL_ALIASES.update({f"ch{i + 1}": i for i in range(5)})


def validate_channel(value):
    """Channel name/alias, or 1..5 like bp5758d / the board's CH1..CH5 terminals -> 0..4."""
    if isinstance(value, int) and not isinstance(value, bool):
        return cv.int_range(min=1, max=5)(value) - 1
    key = cv.string_strict(value).lower()
    if key.isdigit():
        return cv.int_range(min=1, max=5)(int(key)) - 1
    if key not in CHANNEL_ALIASES:
        raise cv.Invalid(
            f"unknown channel '{value}' (use 1-5, {', '.join(CHANNEL_ALIASES)})"
        )
    return CHANNEL_ALIASES[key]


power = cv.float_with_unit("power", "(W|w|watt|Watt|watts|Watts)?")


CONF_DEFAULT = "default"


def _channel_map(value_validator, what, require_all=True):
    """Per-channel mapping that accepts every channel alias, normalised to CHANNEL_KEYS.

    `default:` applies to every channel not listed. With require_all=False, channels
    left out (and no default) are omitted from the result.
    """

    def validator(value):
        if not isinstance(value, dict):
            raise cv.Invalid(f"{what} must be a value or a mapping of channel -> value")
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
            raise cv.Invalid(
                f"{what}: missing channel(s) {', '.join(missing)} (or give a default:)"
            )
        return {k: out[k] for k in CHANNEL_KEYS if k in out}

    return validator


_GAIN_VALUE = cv.int_range(min=0, max=31)
_GAIN_MAP = _channel_map(_GAIN_VALUE, "gain")


def validate_gain(value):
    """One int for all channels, or a per-channel mapping."""
    if isinstance(value, dict):
        return _GAIN_MAP(value)
    v = _GAIN_VALUE(value)
    return {k: v for k in CHANNEL_KEYS}


_CURRENT_MAP = _channel_map(cv.current, "channel_current")


def validate_channel_current(value):
    if isinstance(value, dict):
        return _CURRENT_MAP(value)
    v = cv.current(value)
    return {k: v for k in CHANNEL_KEYS}


_CHANNEL_CAP = cv.All(cv.current, cv.Range(min=0.01))
_CHANNEL_CAP_MAP = _channel_map(_CHANNEL_CAP, "max_channel_current", require_all=False)


def validate_max_channel_current(value):
    """One cap for every channel, or per channel (channels left out are not capped)."""
    if isinstance(value, dict):
        return _CHANNEL_CAP_MAP(value)
    v = _CHANNEL_CAP(value)
    return {k: v for k in CHANNEL_KEYS}


# Datasheet limits (ns). Out-of-spec values are allowed but warned about, so the bench
# can still explore margins. WS2915 V1.5 relaxed T1H to 520 ns; WS2805 V1.6 keeps 580 ns.
_SPEC = {
    "ws2915": {
        CONF_BIT0_HIGH: (220, 380),
        CONF_BIT0_LOW: (580, 1000),
        CONF_BIT1_HIGH: (520, 1000),
        CONF_BIT1_LOW: (580, 1000),
    },
    "ws2805": {
        CONF_BIT0_HIGH: (220, 380),
        CONF_BIT0_LOW: (580, 1000),
        CONF_BIT1_HIGH: (580, 1000),
        CONF_BIT1_LOW: (580, 1000),
    },
}
_MIN_BIT_PERIOD_NS = 1250


def _warn_timing(config: ConfigType) -> ConfigType:
    spec = _SPEC[config[CONF_CHIP_TYPE]]
    ns = {k: config[k].total_nanoseconds for k in spec}
    for key, (lo, hi) in spec.items():
        if not lo <= ns[key] <= hi:
            _LOGGER.warning(
                "ws2915: %s=%d ns is outside the %s datasheet window %d-%d ns",
                key,
                ns[key],
                config[CONF_CHIP_TYPE],
                lo,
                hi,
            )
    for hi_key, lo_key in (
        (CONF_BIT0_HIGH, CONF_BIT0_LOW),
        (CONF_BIT1_HIGH, CONF_BIT1_LOW),
    ):
        period = ns[hi_key] + ns[lo_key]
        if period < _MIN_BIT_PERIOD_NS:
            _LOGGER.warning(
                "ws2915: %s+%s=%d ns is below the datasheet minimum bit period of %d ns",
                hi_key,
                lo_key,
                period,
                _MIN_BIT_PERIOD_NS,
            )
    return config


def frame_time_us(config: ConfigType) -> float:
    """Worst-case frame duration: header + data bits at the slower bit period, plus reset."""
    hdr = 0
    if config[CONF_CHIP_TYPE] == "ws2915":
        hdr = 26 if config.get(CONF_HEADER_FORMAT) == "26bit" else 32
    bits = hdr + config[CONF_NUM_CHIPS] * (80 if config[CONF_CHIP_TYPE] == "ws2915" else 40)
    period = max(
        config[CONF_BIT0_HIGH].total_nanoseconds + config[CONF_BIT0_LOW].total_nanoseconds,
        config[CONF_BIT1_HIGH].total_nanoseconds + config[CONF_BIT1_LOW].total_nanoseconds,
    )
    return bits * period / 1000 + config[CONF_RESET_TIME].total_nanoseconds / 1000


def _warn_frame_rate(config: ConfigType) -> ConfigType:
    if (interval := config.get(CONF_TRANSITION_REFRESH_RATE)) is not None:
        frame = frame_time_us(config)
        if frame > interval:
            _LOGGER.warning(
                "ws2915: a frame for %d chip(s) takes %.0f us, longer than the %d us "
                "transition_refresh_rate interval; the line will run at ~%.0f fps",
                config[CONF_NUM_CHIPS],
                frame,
                interval,
                1e6 / frame,
            )
    return config


def _validate_chip_options(config: ConfigType) -> ConfigType:
    chip = config[CONF_CHIP_TYPE]
    if chip == "ws2915":
        if CONF_GAIN not in config:
            raise cv.Invalid(
                "gain is required for chip_type ws2915 (it sets the chain-wide gain header). "
                "With a pull-up/MOSFET front end use 31; for direct LED drive it sets the current.",
                [CONF_GAIN],
            )
        config.setdefault(CONF_HEADER_FORMAT, "32bit")
        zero = [k for k, v in config[CONF_GAIN].items() if v == 0]
        if zero and config[CONF_HEADER_FORMAT] == "32bit":
            _LOGGER.warning(
                "ws2915: gain 0 on %s: on V1.3+/V1.5 silicon code 0 = 0 mA, i.e. that output never sinks",
                ", ".join(zero),
            )
    else:
        for key in (CONF_GAIN, CONF_HEADER_FORMAT):
            if key in config:
                raise cv.Invalid(
                    f"the WS2805 has no gain header; remove {key}", [key]
                )
    return config


def _validate_reset(value):
    value = cv.positive_time_period_nanoseconds(value)
    us = value.total_nanoseconds / 1000
    if us <= 280:
        raise cv.Invalid("reset_time must be > 280us (datasheet RES minimum)")
    if us > 800:
        raise cv.Invalid("reset_time above 800us does not fit one RMT symbol")
    return value


_RATE_RE = re.compile(r"^\s*([0-9]*\.?[0-9]+)\s*(hz|fps)\s*$", re.IGNORECASE)


def _validate_transition_rate(value):
    """'200Hz' / '200fps', or the frame interval as a time ('5ms'). Returns microseconds."""
    if isinstance(value, str) and (m := _RATE_RE.match(value)):
        hz = float(m.group(1))
        if not 1 <= hz <= 1000:
            raise cv.Invalid("transition_refresh_rate must be 1-1000 Hz")
        return round(1e6 / hz)
    us = cv.positive_time_period_microseconds(value).total_microseconds
    if not 1000 <= us <= 1_000_000:
        raise cv.Invalid("transition_refresh_rate must be 1ms-1s (1000-1 Hz)")
    return int(us)


def _validate_power_limit(value):
    value = POWER_LIMIT_SCHEMA(value)
    caps = (CONF_MAX_CHANNEL_CURRENT, CONF_MAX_CHIP_CURRENT, CONF_MAX_CURRENT, CONF_MAX_POWER)
    if not any(k in value for k in caps):
        raise cv.Invalid(
            "power_limit needs at least one of max_channel_current, max_chip_current, "
            "max_current / max_power"
        )
    if CONF_MAX_CURRENT in value and CONF_MAX_POWER in value:
        raise cv.Invalid("power_limit: use max_current or max_power, not both")
    if CONF_MAX_POWER in value and CONF_SUPPLY_VOLTAGE not in value:
        raise cv.Invalid(
            "power_limit: max_power needs supply_voltage", [CONF_SUPPLY_VOLTAGE]
        )
    if CONF_MAX_POWER not in value and CONF_SUPPLY_VOLTAGE in value:
        raise cv.Invalid(
            "power_limit: supply_voltage is only used with max_power",
            [CONF_SUPPLY_VOLTAGE],
        )
    for key, cap in value.get(CONF_MAX_CHANNEL_CURRENT, {}).items():
        draw = value[CONF_CHANNEL_CURRENT][key]
        if draw > cap:
            _LOGGER.info(
                "ws2915: CH%d (%s) draws %.2f A at 100 %% but is capped at %.2f A, "
                "so it will not exceed %.0f %%",
                CHANNEL_KEYS.index(key) + 1,
                key,
                draw,
                cap,
                cap / draw * 100,
            )
    return value


POWER_LIMIT_SCHEMA = cv.Schema(
    {
        # LED current of each channel at 100 % duty, per chip (same for every chip on this line).
        cv.Required(CONF_CHANNEL_CURRENT): validate_channel_current,
        # Cap per channel, e.g. its MOSFET / terminal rating. Only that channel is held back,
        # so independent lights on the same chip are not affected.
        cv.Optional(CONF_MAX_CHANNEL_CURRENT): validate_max_channel_current,
        # Cap per chip / board, e.g. below its fuse. Only the overloaded chip is dimmed.
        cv.Optional(CONF_MAX_CHIP_CURRENT): cv.All(cv.current, cv.Range(min=0.01)),
        # Cap for the whole line, e.g. the supply. All chips are dimmed uniformly.
        cv.Optional(CONF_MAX_CURRENT): cv.All(cv.current, cv.Range(min=0.01)),
        cv.Optional(CONF_MAX_POWER): cv.All(power, cv.Range(min=0.1)),
        cv.Optional(CONF_SUPPLY_VOLTAGE): cv.All(cv.voltage, cv.Range(min=1.0)),
    }
)


CONFIG_SCHEMA = cv.All(
    esp32.only_on_variant(
        unsupported=list(esp32_rmt.VARIANTS_NO_RMT),
        msg_prefix="WS2915 (RMT)",
    ),
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(WS2915Component),
            cv.Optional(CONF_CHIP_TYPE, default="ws2915"): cv.one_of(
                *CHIP_TYPES, lower=True
            ),
            cv.Required(CONF_PIN): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_NUM_CHIPS): cv.int_range(min=1, max=1024),
            # WS2915 only (rejected for ws2805). header_format defaults to 32bit there.
            cv.Optional(CONF_HEADER_FORMAT): cv.one_of(*HEADER_FORMATS, lower=True),
            # WS2915 only, and required there: no default on purpose, since gain sets the
            # output current (direct drive) or must exceed the pull-up current (MOSFET stage).
            cv.Optional(CONF_GAIN): validate_gain,
            cv.Optional(
                CONF_BIT0_HIGH, default="340ns"
            ): cv.positive_time_period_nanoseconds,
            cv.Optional(
                CONF_BIT0_LOW, default="960ns"
            ): cv.positive_time_period_nanoseconds,
            cv.Optional(
                CONF_BIT1_HIGH, default="700ns"
            ): cv.positive_time_period_nanoseconds,
            cv.Optional(
                CONF_BIT1_LOW, default="650ns"
            ): cv.positive_time_period_nanoseconds,
            cv.Optional(CONF_RESET_TIME, default="300us"): _validate_reset,
            # Re-send the (unchanged) frame periodically, so a frame corrupted by ESD/EMI
            # does not stay latched until the next change. "never" = only on change.
            cv.Optional(CONF_REFRESH_INTERVAL, default="never"): cv.Any(
                cv.one_of("never", lower=True),
                cv.All(
                    cv.positive_time_period_milliseconds,
                    cv.Range(min=cv.TimePeriod(milliseconds=20)),
                ),
            ),
            # While values change (transitions, effects): frames per second, or the frame
            # interval. Unset = one frame per ESPHome main loop (16 ms, ~60 fps). Faster than
            # the main loop keeps the loop in high-frequency mode until the light settles.
            cv.Optional(CONF_TRANSITION_REFRESH_RATE): _validate_transition_rate,
            cv.Optional(CONF_POWER_LIMIT): _validate_power_limit,
            # Default supply for every channel whose output has no power_supply of its own.
            # Supplies (here or on outputs) are switched by this driver without blocking;
            # give them enable_time: 0ms and use power_on_delay.
            cv.Optional(CONF_POWER_SUPPLY): cv.use_id(power_supply.PowerSupply),
            # Time the boards need after power-on before they take data: after boot, and
            # after power_supply switches on. Frames are held meanwhile (non-blocking).
            cv.Optional(CONF_POWER_ON_DELAY): cv.All(
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
                esp32.only_on_variant(
                    supported=[esp32.VARIANT_ESP32P4, esp32.VARIANT_ESP32S3]
                ),
                cv.boolean,
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_chip_options,
    _warn_timing,
    _warn_frame_rate,
)


def _hub_outputs(full_config, hub_id):
    """Output configs (platform ws2915) that belong to hub `hub_id`."""
    return [
        out
        for out in full_config.get("output", []) or []
        if out.get("platform") == "ws2915" and out.get(CONF_WS2915_ID) == hub_id
    ]


def _power_supply_config(full_config, psu_id):
    for conf in full_config.get("power_supply", []) or []:
        if conf[CONF_ID] == psu_id:
            return conf
    return None


def keep_on_time_ms(full_config, psu_id) -> int:
    """keep_on_time of power_supply `psu_id` (0 if it cannot be found)."""
    conf = _power_supply_config(full_config, psu_id)
    return int(conf[CONF_KEEP_ON_TIME].total_milliseconds) if conf else 0


def _final_validate(configs):
    full = fv.full_config.get()
    for conf in configs if isinstance(configs, list) else [configs]:
        # Supplies used by this line: its default, and any set on its outputs.
        used = {}
        if CONF_POWER_SUPPLY in conf:
            used[str(conf[CONF_POWER_SUPPLY])] = conf[CONF_POWER_SUPPLY]
        for out in _hub_outputs(full, conf[CONF_ID]):
            if CONF_POWER_SUPPLY in out:
                used[str(out[CONF_POWER_SUPPLY])] = out[CONF_POWER_SUPPLY]
        for name, psu_id in used.items():
            psu = _power_supply_config(full, psu_id)
            if psu is None:
                continue
            enable_ms = psu[CONF_ENABLE_TIME].total_milliseconds
            if enable_ms > 50:
                _LOGGER.warning(
                    "ws2915: power_supply '%s' has enable_time %d ms, which blocks the main "
                    "loop on every switch-on; set enable_time: 0ms and use power_on_delay on "
                    "ws2915 '%s' instead",
                    name,
                    enable_ms,
                    conf[CONF_ID],
                )
    return configs


FINAL_VALIDATE_SCHEMA = _final_validate

_LIGHT_OUTPUT_KEYS = ("output", "red", "green", "blue", "white", "cold_white", "warm_white")


def _hub_lights(full_config, hub_id):
    """(light id, [output ids on this line]) for every light that uses this line."""
    ours = {o[CONF_ID] for o in _hub_outputs(full_config, hub_id)}
    found = []
    for light in full_config.get("light", []) or []:
        outs = [light[k] for k in _LIGHT_OUTPUT_KEYS if light.get(k) in ours]
        if outs:
            found.append((light[CONF_ID], outs))
    return found


async def to_code(config: ConfigType) -> None:
    # ESPHome excludes the IDF RMT driver by default to save compile time.
    include_builtin_idf_component("esp_driver_rmt")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_chip_type(CHIP_TYPES[config[CONF_CHIP_TYPE]]))
    cg.add(var.set_pin(config[CONF_PIN][CONF_NUMBER]))
    if config[CONF_PIN][CONF_INVERTED]:
        cg.add(var.set_inverted(True))
    cg.add(var.set_num_chips(config[CONF_NUM_CHIPS]))
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

    if CONF_HEADER_FORMAT in config:
        cg.add(var.set_header_format(HEADER_FORMATS[config[CONF_HEADER_FORMAT]]))
    if CONF_GAIN in config:
        for idx, key in enumerate(CHANNEL_KEYS):
            cg.add(var.set_gain(idx, config[CONF_GAIN][key]))

    refresh = config[CONF_REFRESH_INTERVAL]
    if refresh != "never":
        cg.add(var.set_refresh_interval(refresh.total_milliseconds))
    if CONF_TRANSITION_REFRESH_RATE in config:
        cg.add(
            var.set_transition_refresh_interval(config[CONF_TRANSITION_REFRESH_RATE])
        )

    if CONF_POWER_ON_DELAY in config:
        cg.add(var.set_power_on_delay(config[CONF_POWER_ON_DELAY].total_milliseconds))
    if CONF_POWER_SUPPLY in config:
        # Default for channels whose output has no power_supply of its own.
        psu = await cg.get_variable(config[CONF_POWER_SUPPLY])
        cg.add(
            var.set_default_power_supply(
                psu, keep_on_time_ms(CORE.config, config[CONF_POWER_SUPPLY])
            )
        )

    if CONF_POWER_LIMIT in config:
        plim = config[CONF_POWER_LIMIT]
        for idx, key in enumerate(CHANNEL_KEYS):
            cg.add(var.set_channel_current(idx, plim[CONF_CHANNEL_CURRENT][key]))
        for key, cap in plim.get(CONF_MAX_CHANNEL_CURRENT, {}).items():
            cg.add(var.set_max_channel_current(CHANNEL_KEYS.index(key), cap))
        if CONF_MAX_CHIP_CURRENT in plim:
            cg.add(var.set_max_chip_current(plim[CONF_MAX_CHIP_CURRENT]))
        if CONF_MAX_CURRENT in plim:
            cg.add(var.set_max_current(plim[CONF_MAX_CURRENT]))
        elif CONF_MAX_POWER in plim:
            cg.add(
                var.set_max_current(plim[CONF_MAX_POWER] / plim[CONF_SUPPLY_VOLTAGE])
            )

    # Lights on this line: after a power-on hold, a fade-in that ran in the dark is replayed
    # with the length the call used. Registered last; their variables exist by now.
    if CONF_POWER_ON_DELAY in config:
        for light_id, out_ids in _hub_lights(CORE.config, config[CONF_ID]):
            light_var = await cg.get_variable(light_id)
            outs = [await cg.get_variable(o) for o in out_ids]
            cg.add(var.add_light(light_var, outs))
