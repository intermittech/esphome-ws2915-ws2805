"""dig2analog_ultra_strip light: a virtual addressable strip, one pixel per dig2analog_ultra light.

ESPHome's addressable effects (rainbow, color wipe, scan, twinkle, fireworks, flicker, ...) run on it unchanged. Each
changed pixel goes to its board as one command; the board applies the light's gamma and glides between the steps.
"""

import esphome.codegen as cg
from esphome.components import light
from esphome.components.dig2analog_ultra import (
    EFFECT_GLIDE_MODES,
    HARD_EFFECTS,
    dig2analog_ultra_ns,
    light_roles,
    validate_effect_glide,
)
import esphome.config_validation as cv
from esphome.const import CONF_EFFECTS, CONF_GAMMA_CORRECT, CONF_ID, CONF_OUTPUT_ID, CONF_TYPE_ID
from esphome.core import CORE
import esphome.final_validate as fv
from esphome.types import ConfigType

DEPENDENCIES = ["dig2analog_ultra"]

UltraStrip = dig2analog_ultra_ns.class_("UltraStrip", light.AddressableLight)

CONF_LIGHTS = "lights"
CONF_EFFECT_GLIDE = "effect_glide"
CONF_SEGMENTS = "segments"

WHITE_KEYS = ("white", "cold_white", "warm_white")


def _validate_gamma(value):
    value = cv.positive_float(value)
    if value != 1.0:
        raise cv.Invalid("leave gamma_correct at 1.0 here: each board applies its light's own gamma")
    return value


CONFIG_SCHEMA = light.ADDRESSABLE_LIGHT_SCHEMA.extend(
    {
        cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(UltraStrip),
        # pixel 0, 1, 2, ...: dig2analog_ultra lights, on any boards and data lines
        cv.Required(CONF_LIGHTS): cv.All(cv.ensure_list(cv.use_id(light.LightState)), cv.Length(min=1)),
        cv.Optional(CONF_EFFECT_GLIDE, default="auto"): validate_effect_glide,
        cv.Optional(CONF_GAMMA_CORRECT, default=1.0): _validate_gamma,
    }
).extend(cv.COMPONENT_SCHEMA)


def _ultra_lights(full_config) -> dict:
    return {
        str(conf[CONF_ID]): conf
        for conf in full_config.get("light", []) or []
        if conf.get("platform") == "dig2analog_ultra"
    }


def _final_validate(config: ConfigType) -> ConfigType:
    full = fv.full_config.get()
    ultra = _ultra_lights(full)
    seen = set()
    for i, light_id in enumerate(config[CONF_LIGHTS]):
        if str(light_id) not in ultra:
            raise cv.Invalid(f"'{light_id}' is not a dig2analog_ultra light", [CONF_LIGHTS, i])
        if str(light_id) in seen:
            raise cv.Invalid(f"'{light_id}' is listed twice", [CONF_LIGHTS, i])
        seen.add(str(light_id))
    # A partition over this strip writes its pixels with its own gamma: the boards apply gamma already.
    for part in full.get("light", []) or []:
        if part.get("platform") != "partition":
            continue
        uses = any(str(seg.get(CONF_ID)) == str(config[CONF_ID]) for seg in part.get(CONF_SEGMENTS, []))
        if uses and part.get(CONF_GAMMA_CORRECT, 2.8) != 1.0:
            raise cv.Invalid(
                f"partition '{part.get('name', part[CONF_ID])}' uses this strip: set gamma_correct: 1.0 on the "
                "partition (each board applies its light's own gamma)"
            )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


def _roles(conf: ConfigType) -> int:
    """Roles of CH1..CH5 (ULTRA_ROLE_*), 4 bits each, CH1 in the lowest nibble."""
    packed = 0
    for channel, role in light_roles(conf):
        packed |= role << (4 * channel)
    return packed


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    ultra = _ultra_lights(CORE.config)
    rgb = white = False
    for light_id in config[CONF_LIGHTS]:
        conf = ultra[str(light_id)]
        cg.add(var.add_pixel(await cg.get_variable(conf[CONF_OUTPUT_ID]), _roles(conf)))
        rgb |= "red" in conf
        white |= any(k in conf for k in WHITE_KEYS)
    # RGB / RGBW when some light has RGB, else brightness only (e.g. single-channel downlights)
    cg.add(var.set_colors(rgb, white))
    cg.add(var.set_glide_mode(EFFECT_GLIDE_MODES[config[CONF_EFFECT_GLIDE]]))
    for index, effect in enumerate(config.get(CONF_EFFECTS, []), start=1):
        key = next(k for k in effect if k != CONF_TYPE_ID)
        if key in HARD_EFFECTS:
            cg.add(var.add_hard_effect(index))
    await light.register_light(var, config)
    await cg.register_component(var, config)
