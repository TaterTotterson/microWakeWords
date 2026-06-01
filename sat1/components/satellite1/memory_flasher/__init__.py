import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

from esphome.components.memory_flasher import (
    FLASHER_CONFIG_SCHEMA,
    MemoryFlasher,
    register_memory_flasher
)

from .. import satellite1 as sat

CODEOWNERS = ["@gnumpi"]

DEPENDENCIES = ["satellite1", "memory_flasher"]

XMOSFlasher = sat.namespace.class_("XMOSFlasher", MemoryFlasher, sat.Satellite1SPIService, cg.Component )

CONF_AUTO_FLASH = "auto_flash"



CONFIG_SCHEMA = FLASHER_CONFIG_SCHEMA.extend(
    cv.Schema(
        {
          cv.GenerateID(): cv.declare_id(XMOSFlasher),
          cv.GenerateID(sat.CONF_SATELLITE1): cv.use_id(sat.Satellite1),
          cv.Optional(CONF_AUTO_FLASH, default=False): cv.boolean,
        }
    )
)
    


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await register_memory_flasher(var, config)
    await cg.register_parented(var, config[sat.CONF_SATELLITE1])
    cg.add(var.set_auto_flash(config[CONF_AUTO_FLASH]))
    return var
