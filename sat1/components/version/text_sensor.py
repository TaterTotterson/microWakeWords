import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import (
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_NEW_BOX,
    CONF_HIDE_TIMESTAMP,
)

from .helpers import has_changed_files, get_current_commit_hash

version_ns = cg.esphome_ns.namespace("version")
VersionTextSensor = version_ns.class_(
    "VersionTextSensor", text_sensor.TextSensor, cg.Component
)

def validate_git(config):
    if has_changed_files():
        raise cv.Invalid("Please commit all local changes before building the firmware.")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        text_sensor.text_sensor_schema(
            icon=ICON_NEW_BOX,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        )
        .extend(
            {
                cv.GenerateID(): cv.declare_id(VersionTextSensor),
                cv.Optional(CONF_HIDE_TIMESTAMP, default=False): cv.boolean,
            }
        )
        .extend(cv.COMPONENT_SCHEMA)
    ),
   #validate_git
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    await cg.register_component(var, config)
    cg.add(var.set_hide_timestamp(config[CONF_HIDE_TIMESTAMP]))
    cg.add(var.set_git_commit(get_current_commit_hash() or "NoGit"))
