import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.const import (
    CONF_ID,
 )


from esphome.components.satellite1 import (
    CONF_SATELLITE1,
    satellite1_ns,
    Satellite1,
    Satellite1SPIService
)

DEPENDENCIES = ["spi"]
CODEOWNERS = ["@gnumpi"]




SPIErrorRate = satellite1_ns.class_("SPIErrorRate", cg.Component, Satellite1SPIService )


CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(SPIErrorRate),
        cv.GenerateID(CONF_SATELLITE1): cv.use_id(Satellite1),
    })
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_SATELLITE1])
    return var






