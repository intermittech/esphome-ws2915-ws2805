import esphome.codegen as cg
from esphome.components import output
import esphome.config_validation as cv
from esphome.const import CONF_CHANNEL, CONF_ID, CONF_NUM_CHIPS
import esphome.final_validate as fv
from esphome.types import ConfigType

from . import WS2915Component, validate_channel

DEPENDENCIES = ["ws2915"]

Channel = WS2915Component.class_("Channel", output.FloatOutput)

CONF_WS2915_ID = "ws2915_id"
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
    await output.register_output(var, config)

    parent = await cg.get_variable(config[CONF_WS2915_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_chip(config[CONF_CHIP]))
    cg.add(var.set_channel(config[CONF_CHANNEL]))
