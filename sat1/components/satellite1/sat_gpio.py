import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins

from esphome.const import (
    CONF_ID,
    CONF_MODE,
    CONF_PIN,
    CONF_PORT,
    CONF_OUTPUT,
    CONF_INPUT,
    CONF_INVERTED
)

from . import satellite1 as sat


Satellite1GPIOPin = sat.namespace.class_( 
    "Satellite1GPIOPin", 
    cg.GPIOPin, 
    sat.Satellite1SPIService, 
    cg.Parented.template(sat.Satellite1)
)

XMOSPort = sat.namespace.enum("XMOSPort", is_class=True)


XMOS_PORT = {
"INPUT_A" : XMOSPort.INPUT_A,
"INPUT_B" : XMOSPort.INPUT_B,
"OUTPUT_A" : XMOSPort.OUTPUT_A      
}

def _validate_pin_mode(value):
    if not (value[CONF_INPUT] or value[CONF_OUTPUT]):
        raise cv.Invalid("Mode must be either input or output")
    if value[CONF_INPUT] and value[CONF_OUTPUT]:
        raise cv.Invalid("Mode must be either input or output")
    return value


PIN_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.declare_id(Satellite1GPIOPin),
        cv.Required(sat.CONF_SATELLITE1): cv.use_id(sat.Satellite1),
        cv.Required(CONF_PORT): cv.enum(XMOS_PORT),
        cv.Required(CONF_PIN): cv.int_range(min=0, max=7),
        cv.Optional(CONF_MODE, default=CONF_OUTPUT): cv.All(
            {
                cv.Optional(CONF_INPUT, default=False): cv.boolean,
                cv.Optional(CONF_OUTPUT, default=False): cv.boolean,
            },
            _validate_pin_mode,
        ),
        cv.Optional(CONF_INVERTED, default=False): cv.boolean,
    }
)


@pins.PIN_SCHEMA_REGISTRY.register(sat.CONF_SATELLITE1, PIN_SCHEMA)
async def satellite1_pin_to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_parented(var, config[sat.CONF_SATELLITE1])

    cg.add(var.set_pin(config[CONF_PORT], config[CONF_PIN]))
    cg.add(var.set_inverted(config[CONF_INVERTED]))
    port_mode = {
        CONF_INPUT : config[CONF_PORT] in ["INPUT_A","INPUT_B"],
        CONF_OUTPUT: config[CONF_PORT] in ["OUTPUT_A"]
    }
    cg.add(var.set_flags(pins.gpio_flags_expr(port_mode)))

    return var
