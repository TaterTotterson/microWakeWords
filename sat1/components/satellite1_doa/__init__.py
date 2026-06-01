import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, sensor
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
    UNIT_EMPTY,
)

from esphome.components.satellite1 import satellite1 as sat

AUTO_LOAD = ["binary_sensor", "sensor"]
DEPENDENCIES = ["satellite1"]
CODEOWNERS = ["@gnumpi"]

CONF_FRAME_COUNTER = "frame_counter"
CONF_CONFIDENCE = "confidence"
CONF_ENERGY = "energy"
CONF_ANGLE_INDEX = "angle_index"
CONF_VERTICAL_DELAY = "vertical_delay"
CONF_MIC_EAST_ENERGY = "mic_east_energy"
CONF_MIC_WEST_ENERGY = "mic_west_energy"
CONF_MIC_NORTH_ENERGY = "mic_north_energy"
CONF_MIC_SOUTH_ENERGY = "mic_south_energy"
CONF_SAMPLE_DELAY = "sample_delay"
CONF_VALID = "valid"

satellite1_doa_ns = cg.esphome_ns.namespace("satellite1_doa")
Satellite1DoA = satellite1_doa_ns.class_(
    "Satellite1DoA", cg.PollingComponent, sat.Satellite1SPIService
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Satellite1DoA),
        cv.GenerateID(sat.CONF_SATELLITE1): cv.use_id(sat.Satellite1),
        cv.Optional(CONF_SAMPLE_DELAY): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            icon="mdi:microphone-question",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_CONFIDENCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            icon="mdi:signal",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_ENERGY): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            icon="mdi:waveform",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_ANGLE_INDEX): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            icon="mdi:compass",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_VERTICAL_DELAY): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            icon="mdi:axis-y-arrow",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_MIC_EAST_ENERGY): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            icon="mdi:microphone",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_MIC_WEST_ENERGY): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            icon="mdi:microphone",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_MIC_NORTH_ENERGY): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            icon="mdi:microphone",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_MIC_SOUTH_ENERGY): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            icon="mdi:microphone",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_FRAME_COUNTER): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            icon="mdi:counter",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_VALID): binary_sensor.binary_sensor_schema(
            icon="mdi:check-decagram",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
).extend(cv.polling_component_schema("100ms"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[sat.CONF_SATELLITE1])

    if sample_delay_conf := config.get(CONF_SAMPLE_DELAY):
        sens = await sensor.new_sensor(sample_delay_conf)
        cg.add(var.set_sample_delay_sensor(sens))

    if confidence_conf := config.get(CONF_CONFIDENCE):
        sens = await sensor.new_sensor(confidence_conf)
        cg.add(var.set_confidence_sensor(sens))

    if energy_conf := config.get(CONF_ENERGY):
        sens = await sensor.new_sensor(energy_conf)
        cg.add(var.set_energy_sensor(sens))

    if angle_index_conf := config.get(CONF_ANGLE_INDEX):
        sens = await sensor.new_sensor(angle_index_conf)
        cg.add(var.set_angle_index_sensor(sens))

    if vertical_delay_conf := config.get(CONF_VERTICAL_DELAY):
        sens = await sensor.new_sensor(vertical_delay_conf)
        cg.add(var.set_vertical_delay_sensor(sens))

    if mic_east_energy_conf := config.get(CONF_MIC_EAST_ENERGY):
        sens = await sensor.new_sensor(mic_east_energy_conf)
        cg.add(var.set_mic_east_energy_sensor(sens))

    if mic_west_energy_conf := config.get(CONF_MIC_WEST_ENERGY):
        sens = await sensor.new_sensor(mic_west_energy_conf)
        cg.add(var.set_mic_west_energy_sensor(sens))

    if mic_north_energy_conf := config.get(CONF_MIC_NORTH_ENERGY):
        sens = await sensor.new_sensor(mic_north_energy_conf)
        cg.add(var.set_mic_north_energy_sensor(sens))

    if mic_south_energy_conf := config.get(CONF_MIC_SOUTH_ENERGY):
        sens = await sensor.new_sensor(mic_south_energy_conf)
        cg.add(var.set_mic_south_energy_sensor(sens))

    if frame_counter_conf := config.get(CONF_FRAME_COUNTER):
        sens = await sensor.new_sensor(frame_counter_conf)
        cg.add(var.set_frame_counter_sensor(sens))

    if valid_conf := config.get(CONF_VALID):
        sens = await binary_sensor.new_binary_sensor(valid_conf)
        cg.add(var.set_valid_binary_sensor(sens))
