import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, i2c, sensor
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
    UNIT_EMPTY,
)

AUTO_LOAD = ["binary_sensor", "sensor"]
DEPENDENCIES = ["i2c"]

CONF_FRAME_COUNTER = "frame_counter"
CONF_CONFIDENCE = "confidence"
CONF_ENERGY = "energy"
CONF_SAMPLE_DELAY = "sample_delay"
CONF_VALID = "valid"

voicepe_doa_ns = cg.esphome_ns.namespace("voicepe_doa")
VoicePEDoA = voicepe_doa_ns.class_("VoicePEDoA", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(VoicePEDoA),
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
    )
    .extend(cv.polling_component_schema("100ms"))
    .extend(i2c.i2c_device_schema(0x42))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if sample_delay_conf := config.get(CONF_SAMPLE_DELAY):
        sens = await sensor.new_sensor(sample_delay_conf)
        cg.add(var.set_sample_delay_sensor(sens))

    if confidence_conf := config.get(CONF_CONFIDENCE):
        sens = await sensor.new_sensor(confidence_conf)
        cg.add(var.set_confidence_sensor(sens))

    if energy_conf := config.get(CONF_ENERGY):
        sens = await sensor.new_sensor(energy_conf)
        cg.add(var.set_energy_sensor(sens))

    if frame_counter_conf := config.get(CONF_FRAME_COUNTER):
        sens = await sensor.new_sensor(frame_counter_conf)
        cg.add(var.set_frame_counter_sensor(sens))

    if valid_conf := config.get(CONF_VALID):
        sens = await binary_sensor.new_binary_sensor(valid_conf)
        cg.add(var.set_valid_binary_sensor(sens))
