import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components.audio_dac import AudioDac, audio_dac_ns

from esphome.const import CONF_ID, CONF_TRIGGER_ID

from ..satellite1 import (
    Satellite1, 
    Satellite1SPIService,
    namespace as sat1_ns,
    CONF_SATELLITE1
)


CODEOWNERS = ["@gnumpi"]
DEPENDENCIES = ["satellite1"]

DACProxy = sat1_ns.class_("DACProxy", AudioDac, Satellite1SPIService, cg.Component)

ActivateAction = sat1_ns.class_(
    "ActivateAction", automation.Action, cg.Parented.template(DACProxy)
)


ActivateLineOutAction = sat1_ns.class_(
    "ActivateLineOutAction", automation.Action, cg.Parented.template(DACProxy)
)

ActivateSpeakerAction = sat1_ns.class_(
    "ActivateSpeakerAction", automation.Action, cg.Parented.template(DACProxy)
)

StateTrigger = sat1_ns.class_("StateTrigger", automation.Trigger)
SpeakerActivatedTrigger = sat1_ns.class_("SpeakerActivatedTrigger", automation.Trigger.template())
LineOutActivatedTrigger = sat1_ns.class_("LineOutActivatedTrigger", automation.Trigger.template())



CONF_SPEAKER_DAC = "speaker_dac"
CONF_LINE_OUT_DAC = "line_out_dac" 

CONF_ON_STATE_CHANGE = "on_state_change"
CONF_ON_SPEAKER_ACTIVATED = "on_speaker_activated"
CONF_ON_LINE_OUT_ACTIVATED = "on_line_out_activated"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DACProxy),
            cv.GenerateID(CONF_SATELLITE1): cv.use_id(Satellite1),
            cv.Optional(CONF_SPEAKER_DAC) : cv.use_id(AudioDac),
            cv.Optional(CONF_LINE_OUT_DAC) : cv.use_id(AudioDac),

            cv.Optional(CONF_ON_STATE_CHANGE): automation.validate_automation({
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(StateTrigger),
            }),
            cv.Optional(CONF_ON_SPEAKER_ACTIVATED): automation.validate_automation({
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(SpeakerActivatedTrigger),
            }),
            cv.Optional(CONF_ON_LINE_OUT_ACTIVATED): automation.validate_automation({
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(LineOutActivatedTrigger),
            }),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_SATELLITE1])
    if CONF_SPEAKER_DAC in config :
        speaker_dac = await cg.get_variable(config[CONF_SPEAKER_DAC])
        cg.add( var.set_speaker_dac(speaker_dac) )
    if CONF_LINE_OUT_DAC in config :
        lineout_dac = await cg.get_variable(config[CONF_LINE_OUT_DAC])
        cg.add( var.set_lineout_dac(lineout_dac) )
    
    for conf in config.get(CONF_ON_STATE_CHANGE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
    for conf in config.get(CONF_ON_SPEAKER_ACTIVATED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
    for conf in config.get(CONF_ON_LINE_OUT_ACTIVATED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)


DAC_ACTION_SCHEMA = automation.maybe_simple_id({cv.GenerateID(): cv.use_id(DACProxy)})

@automation.register_action("dac_proxy.activate", ActivateAction, DAC_ACTION_SCHEMA, synchronous=True)
@automation.register_action("dac_proxy.activate_line_out", ActivateLineOutAction, DAC_ACTION_SCHEMA, synchronous=True)
@automation.register_action("dac_proxy.activate_speaker", ActivateSpeakerAction, DAC_ACTION_SCHEMA, synchronous=True)
async def tas2780_action(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
