import voluptuous as vol
from esphome.components import binary_sensor
from esphome.components.ttp229_bsf import TTP229BSFComponent, CONF_TTP229_BSF_ID
import esphome.config_validation as cv
from esphome.const import CONF_CHANNEL, CONF_NAME
from esphome.cpp_generator import get_variable

DEPENDENCIES = ['ttp229_bsf']

TTP229BSFChannel = binary_sensor.binary_sensor_ns.class_(
    'TTP229BSFChannel', binary_sensor.BinarySensor)

PLATFORM_SCHEMA = cv.nameable(binary_sensor.BINARY_SENSOR_PLATFORM_SCHEMA.extend({
    cv.GenerateID(): cv.declare_variable_id(TTP229BSFChannel),
    cv.GenerateID(CONF_TTP229_BSF_ID): cv.use_variable_id(TTP229BSFComponent),
    vol.Required(CONF_CHANNEL): vol.All(vol.Coerce(int), vol.Range(min=0, max=15))
}))


def to_code(config):
    for hub in get_variable(config[CONF_TTP229_BSF_ID]):
        yield
    rhs = TTP229BSFChannel.new(config[CONF_NAME], config[CONF_CHANNEL])
    binary_sensor.register_binary_sensor(hub.add_channel(rhs), config)
    