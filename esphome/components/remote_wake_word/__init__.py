from esphome import automation
from esphome.automation import register_action, register_condition
import esphome.codegen as cg
from esphome.components import esp32, microphone
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_MICROPHONE, CONF_TRIGGER_ID, CONF_URL

CODEOWNERS = ["@TaterTotterson"]
DEPENDENCIES = ["microphone", "network"]

DOMAIN = "remote_wake_word"

CONF_CHUNK_DURATION_MS = "chunk_duration_ms"
CONF_MAX_FAILURES = "max_failures"
CONF_HTTP_TIMEOUT_MS = "http_timeout_ms"
CONF_ON_ERROR = "on_error"
CONF_ON_WAKE_WORD_DETECTED = "on_wake_word_detected"
CONF_SOURCE_DEVICE = "source_device"
CONF_WAKE_WORD = "wake_word"

remote_wake_word_ns = cg.esphome_ns.namespace("remote_wake_word")
RemoteWakeWord = remote_wake_word_ns.class_("RemoteWakeWord", cg.Component)

StartAction = remote_wake_word_ns.class_("StartAction", automation.Action)
StopAction = remote_wake_word_ns.class_("StopAction", automation.Action)
IsRunningCondition = remote_wake_word_ns.class_("IsRunningCondition", automation.Condition)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RemoteWakeWord),
        cv.Required(CONF_MICROPHONE): microphone.microphone_source_schema(
            min_bits_per_sample=16,
            max_bits_per_sample=16,
            min_channels=1,
            max_channels=1,
        ),
        cv.Required(CONF_URL): cv.string,
        cv.Optional(CONF_WAKE_WORD, default=""): cv.string,
        cv.Optional(CONF_SOURCE_DEVICE, default=""): cv.string,
        cv.Optional(CONF_CHUNK_DURATION_MS, default=500): cv.int_range(min=100, max=2000),
        cv.Optional(CONF_MAX_FAILURES, default=3): cv.int_range(min=1, max=20),
        cv.Optional(CONF_HTTP_TIMEOUT_MS, default=3000): cv.int_range(min=250, max=10000),
        cv.Optional(CONF_ON_WAKE_WORD_DETECTED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    automation.Trigger.template(cg.std_string)
                ),
            }
        ),
        cv.Optional(CONF_ON_ERROR): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    automation.Trigger.template(cg.std_string)
                ),
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

FINAL_VALIDATE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_MICROPHONE): microphone.final_validate_microphone_source_schema(
            "remote_wake_word",
            sample_rate=16000,
        ),
    },
    extra=cv.ALLOW_EXTRA,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    mic_source = await microphone.microphone_source_to_code(config[CONF_MICROPHONE])
    cg.add(var.set_microphone_source(mic_source))
    cg.add(var.set_url(config[CONF_URL]))
    cg.add(var.set_wake_word(config[CONF_WAKE_WORD]))
    cg.add(var.set_source_device(config[CONF_SOURCE_DEVICE]))
    cg.add(var.set_chunk_duration_ms(config[CONF_CHUNK_DURATION_MS]))
    cg.add(var.set_max_failures(config[CONF_MAX_FAILURES]))
    cg.add(var.set_http_timeout_ms(config[CONF_HTTP_TIMEOUT_MS]))
    cg.add_define("USE_REMOTE_WAKE_WORD")
    esp32.add_idf_component(name="espressif/esp_websocket_client", ref="1.4.0")

    for conf in config.get(CONF_ON_WAKE_WORD_DETECTED, []):
        await automation.build_automation(
            var.get_wake_word_detected_trigger(),
            [(cg.std_string, "wake_word")],
            conf,
        )

    for conf in config.get(CONF_ON_ERROR, []):
        await automation.build_automation(
            var.get_error_trigger(),
            [(cg.std_string, "message")],
            conf,
        )


REMOTE_WAKE_WORD_ACTION_SCHEMA = cv.Schema({cv.GenerateID(): cv.use_id(RemoteWakeWord)})


@register_action("remote_wake_word.start", StartAction, REMOTE_WAKE_WORD_ACTION_SCHEMA, synchronous=True)
@register_action("remote_wake_word.stop", StopAction, REMOTE_WAKE_WORD_ACTION_SCHEMA, synchronous=True)
@register_condition("remote_wake_word.is_running", IsRunningCondition, REMOTE_WAKE_WORD_ACTION_SCHEMA)
async def remote_wake_word_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
