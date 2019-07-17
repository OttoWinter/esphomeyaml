import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import CONF_ID, CONF_TEMPERATURE, \
    ICON_THERMOMETER, UNIT_CELSIUS

DEPENDENCIES = ['i2c']

sts3x_ns = cg.esphome_ns.namespace('sts3x')
STS3XComponent = sts3x_ns.class_('STS3XComponent', cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(STS3XComponent),
    cv.Required(CONF_TEMPERATURE): sensor.sensor_schema(UNIT_CELSIUS, ICON_THERMOMETER, 1),
}).extend(cv.polling_component_schema('60s')).extend(i2c.i2c_device_schema(0x4A))


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield i2c.register_i2c_device(var, config)

    if CONF_TEMPERATURE in config:
        sens = yield sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

