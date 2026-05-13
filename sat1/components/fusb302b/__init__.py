from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c


from esphome.pins import internal_gpio_input_pin_number

from esphome.const import (
    CONF_ID,
    CONF_IRQ_PIN,
    CONF_ON_CONNECT,
    CONF_ON_DISCONNECT,
    CONF_ON_ERROR,
    CONF_TRIGGER_ID,
)

CODEOWNERS = ["@gnumpi"]
DEPENDENCIES = ["i2c"]

pd_ns = cg.esphome_ns.namespace("power_delivery")
PowerDelivery = pd_ns.class_("PowerDelivery", cg.Component)
FUSB302B = pd_ns.class_("FUSB302B", PowerDelivery, cg.Component, i2c.I2CDevice)


RequestVoltageAction = pd_ns.class_(
    "PowerDeliveryRequestVoltage",
    automation.Action,
    cg.Parented.template(PowerDelivery),
)

ConnectedTrigger = pd_ns.class_("ConnectedTrigger", automation.Trigger)
DisconnectedTrigger = pd_ns.class_("DisconnectedTrigger", automation.Trigger.template())
ErrorTrigger = pd_ns.class_("ErrorTrigger", automation.Trigger.template())

TransitionTrigger = pd_ns.class_("TransitionTrigger", automation.Trigger.template())
PowerReadyTrigger = pd_ns.class_("PowerReadyTrigger", automation.Trigger)

ValidContractTrigger = pd_ns.class_("ValidContractTrigger", automation.Trigger)
IsConnectedCondition = pd_ns.class_("IsConnectedCondition", automation.Condition)

CONF_REQUEST_VOLTAGE = "request_voltage"
CONF_ON_CONTRACT = "on_contract"
CONF_ON_PWR_RDY = "on_power_ready"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(FUSB302B),
            cv.Required(CONF_IRQ_PIN): internal_gpio_input_pin_number,
            cv.Required(CONF_REQUEST_VOLTAGE): cv.Range(
                min=5, max=20, max_included=True
            ),
            cv.Optional(CONF_ON_CONNECT): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(ConnectedTrigger),
                }
            ),
            cv.Optional(CONF_ON_DISCONNECT): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(DisconnectedTrigger),
                }
            ),
            cv.Optional(CONF_ON_ERROR): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(ErrorTrigger),
                }
            ),
            cv.Optional(CONF_ON_PWR_RDY): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(PowerReadyTrigger),
                }
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x22))
)

PD_ACTION_SCHEMA = automation.maybe_simple_id(
    {cv.GenerateID(): cv.use_id(PowerDelivery)}
)


@automation.register_action(
    "power_delivery.request_voltage",
    RequestVoltageAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(PowerDelivery),
            cv.Required(CONF_REQUEST_VOLTAGE): cv.templatable(
                cv.Range(min=5, max=20, max_included=True)
            ),
        },
        key=CONF_REQUEST_VOLTAGE,
    ),
    synchronous=True,
)
async def power_delivery_request_voltage_action(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    voltage = await cg.templatable(config[CONF_REQUEST_VOLTAGE], args, cg.int32)
    cg.add(var.set_voltage(voltage))
    return var


@automation.register_condition(
    "power_delivery.is_connected", IsConnectedCondition, PD_ACTION_SCHEMA
)
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    cg.add(var.set_request_voltage(config[CONF_REQUEST_VOLTAGE]))
    cg.add(var.set_irq_pin(config[CONF_IRQ_PIN]))
    for conf in config.get(CONF_ON_CONNECT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
    for conf in config.get(CONF_ON_DISCONNECT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
    for conf in config.get(CONF_ON_ERROR, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
    for conf in config.get(CONF_ON_PWR_RDY, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
