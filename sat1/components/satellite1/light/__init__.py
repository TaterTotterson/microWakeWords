import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import light
from esphome.const import CONF_OUTPUT_ID

from .. import (
    CONF_SATELLITE1,
    satellite1_ns,
    Satellite1, 
    Satellite1SPIService
)

DEPENDENCIES = ["satellite1"]
CODEOWNERS = ["@gnumpi"]

LedRing = satellite1_ns.class_("LEDRing", light.AddressableLight, Satellite1SPIService)

CONFIG_SCHEMA = light.ADDRESSABLE_LIGHT_SCHEMA.extend(
    {
        cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(LedRing),
        cv.GenerateID(CONF_SATELLITE1): cv.use_id(Satellite1),
    }
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await light.register_light(var, config)
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_SATELLITE1])