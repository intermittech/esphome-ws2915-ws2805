"""Board firmware updates for one dig2analog_ultra line (fw_update.h).

The entity reads the firmware repo's signed channel manifest from GitHub; Install downloads the image and sends it
to every board of the line in one session. The boards decide themselves (newer: install, same: skip, older: reject
unless forced) and blink the result. Design: the firmware repo's research/firmware-update.md §4.
"""

import re

from esphome import automation
import esphome.codegen as cg
from esphome.components import http_request, select, text_sensor, update
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.const import CONF_ID, CONF_SOURCE, DEVICE_CLASS_FIRMWARE, ENTITY_CATEGORY_CONFIG, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_DIG2ANALOG_ULTRA_ID, UltraHub, dig2analog_ultra_ns

DEPENDENCIES = ["dig2analog_ultra", "http_request"]

CONF_CHANNEL = "channel"
CONF_CHANNEL_SELECT = "channel_select"
CONF_STATUS = "status"
CONF_PUBLIC_KEYS = "public_keys"
CONF_VERIFY_SIGNATURE = "verify_signature"
CONF_BOARD_REVISION = "board_revision"
CONF_PASSES = "passes"
CONF_LEAD_IN = "lead_in"
CONF_POWER_CYCLE = "power_cycle"

# The QuinLED firmware repo on GitHub (its `channels` branch holds the signed channel manifests).
DEFAULT_REPO = "intermittech/quinled-ultra-firmware"

# FW_CH_* (fw_signature.h); the select's options are in this order
CHANNELS = {"stable": 1, "beta": 2, "alpha": 4}
# fw_keys.h: production (none until the firmware repo is set up) or the TEST keys (development, bench)
PUBLIC_KEYS = {
    "production": ("esphome::dig2analog_ultra::FW_PRODUCTION_KEYS", "esphome::dig2analog_ultra::FW_PRODUCTION_KEY_COUNT"),
    "test": (
        "esphome::dig2analog_ultra::FW_TEST_KEYS",
        "(sizeof(esphome::dig2analog_ultra::FW_TEST_KEYS) / sizeof(esphome::dig2analog_ultra::FW_TEST_KEYS[0]))",
    ),
}
HW_ID_D2A_ULTRA_5CH = 1  # UPD_HW_ID_D2A_ULTRA_5CH (ultra_update.h)

UltraFirmwareUpdate = dig2analog_ultra_ns.class_("UltraFirmwareUpdate", update.UpdateEntity, cg.PollingComponent)
FwChannelSelect = dig2analog_ultra_ns.class_("FwChannelSelect", select.Select, cg.Component)
AbortUpdateAction = dig2analog_ultra_ns.class_("AbortUpdateAction", automation.Action)
RecoverAction = dig2analog_ultra_ns.class_("RecoverAction", automation.Action)


def AUTO_LOAD(config=None):
    """json for the manifest; select / text_sensor only for the entities that use them (no config: all)."""
    loads = ["json"]
    if not config or CONF_CHANNEL_SELECT in config:
        loads.append("select")
    if not config or CONF_STATUS in config:
        loads.append("text_sensor")
    return loads


def _validate_source(value):
    """OWNER/REPO on GitHub (its `channels` branch), or an http(s) URL of a manifest ({channel} is replaced)."""
    value = cv.string_strict(value)
    if "://" in value:
        return cv.url(value)
    if not re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", value):
        raise cv.Invalid("source: OWNER/REPO on GitHub, or the http(s) URL of a manifest ({channel} is replaced)")
    return value


def _validate(config):
    if not config[CONF_VERIFY_SIGNATURE] and CONF_SOURCE not in config:
        raise cv.Invalid(
            f"{CONF_VERIFY_SIGNATURE}: false only with your own {CONF_SOURCE} (a LAN mirror or your own builds)"
        )
    return config


CONFIG_SCHEMA = cv.All(
    update.update_schema(UltraFirmwareUpdate, device_class=DEVICE_CLASS_FIRMWARE, entity_category=ENTITY_CATEGORY_CONFIG)
    .extend(
        {
            cv.GenerateID(CONF_DIG2ANALOG_ULTRA_ID): cv.use_id(UltraHub),
            cv.GenerateID(http_request.CONF_HTTP_REQUEST_ID): cv.use_id(http_request.HttpRequestComponent),
            cv.Optional(CONF_SOURCE): _validate_source,
            cv.Optional(CONF_CHANNEL, default="stable"): cv.enum(CHANNELS, lower=True),
            cv.Optional(CONF_PUBLIC_KEYS, default="production"): cv.one_of(*PUBLIC_KEYS, lower=True),
            cv.Optional(CONF_VERIFY_SIGNATURE, default=True): cv.boolean,
            # board revision of this line (bit of an image's hw_rev_mask): 1 = PB11 wired (3-timer build)
            cv.Optional(CONF_BOARD_REVISION, default=1): cv.int_range(min=0, max=15),
            # every page this often per session (a board that missed one gets it in the next pass)
            cv.Optional(CONF_PASSES, default=3): cv.int_range(min=1, max=8),
            # optional: switch the channel from Home Assistant (stored on the ESP32)
            cv.Optional(CONF_CHANNEL_SELECT): select.select_schema(
                FwChannelSelect, entity_category=ENTITY_CATEGORY_CONFIG, icon="mdi:source-branch"
            ).extend(cv.COMPONENT_SCHEMA),
            # optional: what the update does now, and afterwards the blink legend
            cv.Optional(CONF_STATUS): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:information-outline"
            ),
        }
    )
    .extend(cv.polling_component_schema("6h")),
    _validate,
)


def _final_validate(config):
    """One update entity per line: a session owns the line."""
    full = fv.full_config.get()
    hub = config[CONF_DIG2ANALOG_ULTRA_ID]
    same = [
        u
        for u in full.get("update", []) or []
        if u.get("platform") == "dig2analog_ultra" and u.get(CONF_DIG2ANALOG_ULTRA_ID) == hub
    ]
    if len(same) > 1:
        raise cv.Invalid(f"dig2analog_ultra '{hub}' has {len(same)} firmware update entities; one per line")
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    cg.add_define("USE_DIG2ANALOG_ULTRA_UPDATE")
    var = await update.new_update(config)
    await cg.register_component(var, config)
    hub = await cg.get_variable(config[CONF_DIG2ANALOG_ULTRA_ID])
    cg.add(var.set_hub(hub))
    cg.add(var.set_http(await cg.get_variable(config[http_request.CONF_HTTP_REQUEST_ID])))
    cg.add(var.set_source(config.get(CONF_SOURCE, DEFAULT_REPO)))
    cg.add(var.set_channel(config[CONF_CHANNEL]))
    keys, count = PUBLIC_KEYS[config[CONF_PUBLIC_KEYS]]
    cg.add(var.set_keys(cg.RawExpression(keys), cg.RawExpression(count)))
    cg.add(var.set_verify_signature(config[CONF_VERIFY_SIGNATURE]))
    cg.add(var.set_hardware(HW_ID_D2A_ULTRA_5CH, config[CONF_BOARD_REVISION]))
    cg.add(var.set_passes(config[CONF_PASSES]))
    cg.add(hub.set_ping_burst(True))  # freshly installed boards confirm themselves at a PING
    if sel_conf := config.get(CONF_CHANNEL_SELECT):
        sel = await select.new_select(sel_conf, options=list(CHANNELS))
        await cg.register_component(sel, sel_conf)
        cg.add(sel.set_update(var))
    if status_conf := config.get(CONF_STATUS):
        cg.add(var.set_status_sensor(await text_sensor.new_text_sensor(status_conf)))


# ------------------------------------------------------------------------------------------------ actions
@automation.register_action(
    "dig2analog_ultra.abort_update",
    AbortUpdateAction,
    automation.maybe_simple_id({cv.GenerateID(): cv.use_id(UltraFirmwareUpdate)}),
    synchronous=True,
)
async def abort_update_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "dig2analog_ultra.recover",
    RecoverAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(UltraFirmwareUpdate),
            # ANNOUNCE frames this long before the data: boards that no longer listen are caught while they boot
            cv.Optional(CONF_LEAD_IN, default="30s"): cv.templatable(
                cv.All(cv.positive_time_period_milliseconds, cv.Range(min=cv.TimePeriod(seconds=1), max=cv.TimePeriod(minutes=10)))
            ),
            # the hub's power_supply off and on during the lead-in (Diff-Power); without one: by hand
            cv.Optional(CONF_POWER_CYCLE, default=True): cv.templatable(cv.boolean),
        }
    ),
    synchronous=True,
)
async def recover_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_lead_in(await cg.templatable(config[CONF_LEAD_IN], args, cg.uint32)))
    cg.add(var.set_power_cycle(await cg.templatable(config[CONF_POWER_CYCLE], args, cg.bool_)))
    return var
