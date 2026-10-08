import esphome.codegen as cg
from esphome.components import output
import esphome.config_validation as cv
from esphome.const import CONF_CHANNEL, CONF_ID, CONF_NUM_CHIPS, CONF_POWER_SUPPLY
from esphome.core import CORE
import esphome.final_validate as fv
from esphome.types import ConfigType

from . import CONF_WS2915_ID, WS2915Component, keep_on_time_ms, validate_channel

DEPENDENCIES = ["ws2915"]

Channel = WS2915Component.class_("Channel", output.FloatOutput)

CONF_CHIP = "chip"

CONFIG_SCHEMA = output.FLOAT_OUTPUT_SCHEMA.extend(
    {
        cv.GenerateID(CONF_WS2915_ID): cv.use_id(WS2915Component),
        cv.Required(CONF_ID): cv.declare_id(Channel),
        cv.Optional(CONF_CHIP, default=0): cv.int_range(min=0, max=1023),
        # 1..5 (board terminals CH1..CH5), ch1..ch5, red/green/blue/white1/white2 or
        # r/g/b/w1/w2. Wire order is R, G, B, W1, W2 = 1..5.
        cv.Required(CONF_CHANNEL): validate_channel,
    }
)


def _final_validate(config: ConfigType) -> ConfigType:
    fconf = fv.full_config.get()
    try:
        hub_path = fconf.get_path_for_id(config[CONF_WS2915_ID])[:-1]
        hub = fconf.get_config_for_path(hub_path)
    except KeyError:
        return config  # ID validation reports this
    if config[CONF_CHIP] >= hub[CONF_NUM_CHIPS]:
        raise cv.Invalid(
            f"chip {config[CONF_CHIP]} out of range: '{config[CONF_WS2915_ID]}' "
            f"has num_chips: {hub[CONF_NUM_CHIPS]} (chips are 0-based)"
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    # power_supply is switched by the ws2915 hub (non-blocking, with power_on_delay), not by
    # the stock output code, which waits enable_time with a blocking delay().
    stock = {k: v for k, v in config.items() if k != CONF_POWER_SUPPLY}
    await output.register_output(var, stock)

    parent = await cg.get_variable(config[CONF_WS2915_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_chip(config[CONF_CHIP]))
    cg.add(var.set_channel(config[CONF_CHANNEL]))
    if CONF_POWER_SUPPLY in config:
        psu = await cg.get_variable(config[CONF_POWER_SUPPLY])
        cg.add(
            parent.add_channel_power_supply(
                config[CONF_CHIP],
                config[CONF_CHANNEL],
                psu,
                keep_on_time_ms(CORE.config, config[CONF_POWER_SUPPLY]),
            )
        )
