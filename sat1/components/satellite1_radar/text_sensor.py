import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_SATELLITE1_RADAR_ID, Satellite1Radar

DEPENDENCIES = ["satellite1_radar"]

CONFIG_SCHEMA = text_sensor.text_sensor_schema(
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    icon="mdi:radar",
).extend(
    {
        cv.GenerateID(CONF_SATELLITE1_RADAR_ID): cv.use_id(Satellite1Radar),
    }
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    parent = await cg.get_variable(config[CONF_SATELLITE1_RADAR_ID])
    cg.add(parent.set_radar_type_text_sensor(var))
