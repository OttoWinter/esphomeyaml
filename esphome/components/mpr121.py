import voluptuous as vol

from esphome.components import i2c, sensor
import esphome.config_validation as cv
from esphome.const import CONF_ADDRESS, CONF_ID,  CONF_CHANNELS
from esphome.cpp_generator import Pvariable, add
from esphome.cpp_helpers import setup_component
from esphome.cpp_types import App, PollingComponent

DEPENDENCIES = ['i2c']
MULTI_CONF = True

CONF_MPR121_ID = 'mpr121_id'
MPR121 = sensor.sensor_ns.class_('MPR121_Sensor', PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = vol.Schema({
    cv.GenerateID(): cv.declare_variable_id(MPR121),
    vol.Optional(CONF_ADDRESS): cv.i2c_address
}).extend(cv.COMPONENT_SCHEMA.schema)


def to_code(config):
    rhs = App.make_mpr121(config.get(CONF_ADDRESS))
    var = Pvariable(config[CONF_ID], rhs)

    setup_component(var, config)


BUILD_FLAGS = '-DUSE_MPR121'