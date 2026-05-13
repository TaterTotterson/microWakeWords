import esphome.codegen as cg
from esphome.components import i2c
import esphome.config_validation as cv
from esphome import automation
from esphome.components.audio_dac import AudioDac, audio_dac_ns
from esphome.const import CONF_ID, CONF_MODE, CONF_CHANNEL

CODEOWNERS = ["@gnumpi"]
DEPENDENCIES = ["i2c"]

tas2780_ns = cg.esphome_ns.namespace("tas2780")
tas2780 = tas2780_ns.class_("TAS2780", AudioDac, cg.Component, i2c.I2CDevice)


RestAction = tas2780_ns.class_(
    "ResetAction", automation.Action, cg.Parented.template(tas2780)
)

ActivateAction = tas2780_ns.class_(
    "ActivateAction", automation.Action
)

UpdateConfigAction = tas2780_ns.class_(
    "UpdateConfigAction", automation.Action
)

DeactivateAction = tas2780_ns.class_(
    "DeactivateAction", automation.Action, cg.Parented.template(tas2780)
)

CONF_VOL_RNG_MIN = "vol_range_min"
CONF_VOL_RNG_MAX = "vol_range_max"
CONF_AMP_LEVEL_IDX = "amp_level_idx" 

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(tas2780),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x18))
)


TAS2780_ACTION_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(tas2780),
        cv.Optional(CONF_MODE, default=2) : cv.templatable(cv.int_range(min=0, max=3)),
    }
)

@automation.register_action("tas2780.deactivate", DeactivateAction, TAS2780_ACTION_SCHEMA, synchronous=True)
@automation.register_action("tas2780.reset", RestAction, TAS2780_ACTION_SCHEMA, synchronous=True)
async def tas2780_action(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

@automation.register_action("tas2780.activate", ActivateAction, TAS2780_ACTION_SCHEMA, synchronous=True)
async def tas2780_activate_action(config, action_id, template_arg, args):
    tas2780 = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, tas2780)
    mode_ = config.get(CONF_MODE)
    template_ = await cg.templatable(mode_, args, cg.uint8)
    cg.add(var.set_mode(template_))
    return var



TAS2780_UPDATE_CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(tas2780),
        cv.Optional(CONF_VOL_RNG_MIN, default=.3) : cv.templatable(cv.float_range(0.,1.)),
        cv.Optional(CONF_VOL_RNG_MAX, default=1.) : cv.templatable(cv.float_range(0.,1.)),
        cv.Optional(CONF_AMP_LEVEL_IDX) : cv.templatable(cv.int_range(0, 20)),
        cv.Optional(CONF_CHANNEL) : cv.templatable(cv.int_range(0,2))
    }
)


@automation.register_action("tas2780.update_config", UpdateConfigAction, TAS2780_UPDATE_CONFIG_SCHEMA, synchronous=True)
async def tas2780_action(config, action_id, template_arg, args):
    tas2780 = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, tas2780)
    vol_min_ = config.get(CONF_VOL_RNG_MIN)
    template_ = await cg.templatable(vol_min_, args, cg.float_)
    cg.add(var.set_vol_range_min(template_))
    vol_max_ = config.get(CONF_VOL_RNG_MAX)
    template_ = await cg.templatable(vol_max_, args, cg.float_)
    cg.add(var.set_vol_range_max(template_))
    if CONF_AMP_LEVEL_IDX in config:
        amp_level_idx = config.get(CONF_AMP_LEVEL_IDX)
        template_ = await cg.templatable(amp_level_idx, args, cg.uint8)
        cg.add(var.set_amp_level(template_))
    if CONF_CHANNEL in config:
        channel = config.get(CONF_CHANNEL)
        template_ = await cg.templatable(channel, args, cg.uint8)
        cg.add(var.set_channel(template_))
    
    return var



async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)