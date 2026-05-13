import esphome.codegen as cg
import esphome.config_validation as cv


from . import satellite1 as sat
from . import sat_gpio

CODEOWNERS = ["@gnumpi"]


DEPENDENCIES = ["spi"]


CONFIG_SCHEMA = sat.SAT1_CONFIG_SCHEMA
    

async def to_code(config):
    sat1_var = await sat.register_satellite1(config)
    return sat1_var
