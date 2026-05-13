import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation, pins

from esphome.components.spi import SPIDevice, register_spi_device, spi_device_schema

from esphome.const import CONF_ID, CONF_TRIGGER_ID

namespace = cg.esphome_ns.namespace("satellite1")
Satellite1 = namespace.class_("Satellite1", SPIDevice, cg.Component)
Satellite1SPIService = namespace.class_("Satellite1SPIService", cg.Parented.template(Satellite1))


XMOSHardwareResetAction = namespace.class_(
    "XMOSHardwareResetAction", automation.Action
)

XMOSConnectedStateTrigger = namespace.class_(
    "XMOSConnectedStateTrigger", automation.Trigger
)

FlashConnectedStateTrigger = namespace.class_(
    "FlashConnectedStateTrigger", automation.Trigger
)

XMOSNoResponseStateTrigger = namespace.class_(
    "XMOSNoResponseStateTrigger", automation.Trigger
)



CONF_SATELLITE1 = "satellite1"
CONF_XMOS_RST_PIN = "xmos_rst_pin"
CONF_ON_XMOS_NO_RESPONSE = "on_xmos_no_response"
CONF_ON_XMOS_CONNECTED = "on_xmos_connected"
CONF_ON_FLASH_CONNECTED = "on_flash_connected"

SAT1_CONFIG_SCHEMA = (
     cv.Schema({
        cv.GenerateID(): cv.declare_id(Satellite1),
        cv.Optional(CONF_XMOS_RST_PIN, default="GPIO12"): pins.gpio_output_pin_schema,

        cv.Optional(CONF_ON_XMOS_CONNECTED): automation.validate_automation({
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(XMOSConnectedStateTrigger),
        }),
        cv.Optional(CONF_ON_FLASH_CONNECTED): automation.validate_automation({
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(FlashConnectedStateTrigger),
        }),
        cv.Optional(CONF_ON_XMOS_NO_RESPONSE): automation.validate_automation({
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(XMOSNoResponseStateTrigger),
        }),

    }).extend(spi_device_schema(True, "1Hz"))
)


async def register_satellite1(config) :
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await register_spi_device(var, config)
    
    rst_pin = await cg.gpio_pin_expression(config[CONF_XMOS_RST_PIN])
    cg.add(var.set_xmos_rst_pin(rst_pin))
    
    for conf in config.get(CONF_ON_XMOS_CONNECTED, []):
         trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var )
         await automation.build_automation(trigger, [], conf)
    
    for conf in config.get(CONF_ON_FLASH_CONNECTED, []):
         trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var )
         await automation.build_automation(trigger, [], conf)
    
    for conf in config.get(CONF_ON_XMOS_NO_RESPONSE, []):
         trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var )
         await automation.build_automation(trigger, [], conf)
    
    return var


RESET_XMOS_ACTION_SCHEMA = automation.maybe_simple_id(
    {
        cv.GenerateID(CONF_SATELLITE1): cv.use_id(Satellite1)
    }
)
@automation.register_action(
    "satellite1.xmos_hardware_reset", 
    XMOSHardwareResetAction, 
    RESET_XMOS_ACTION_SCHEMA,
    synchronous=True,
)
async def erase_memory_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_SATELLITE1])
    return var

