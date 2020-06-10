import esphome.codegen as cg
from esphome import pins
# from esphome.components import output
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import CONF_ID, ICON_RADIATOR, UNIT_PARTS_PER_MILLION, \
    UNIT_PARTS_PER_BILLION, CONF_TEMPERATURE, CONF_HUMIDITY, ICON_PERIODIC_TABLE_CO2

DEPENDENCIES = ['i2c']

ccs811_ns = cg.esphome_ns.namespace('ccs811')
CCS811Component = ccs811_ns.class_('CCS811Component', cg.PollingComponent, i2c.I2CDevice)

CONF_ECO2 = 'eco2'
CONF_TVOC = 'tvoc'
CONF_BASELINE = 'baseline'
CONF_WAKE_PIN = 'wakeup_pin'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(CCS811Component),
    cv.Required(CONF_ECO2): sensor.sensor_schema(UNIT_PARTS_PER_MILLION, ICON_PERIODIC_TABLE_CO2,
                                                 0),
    cv.Required(CONF_TVOC): sensor.sensor_schema(UNIT_PARTS_PER_BILLION, ICON_RADIATOR, 0),

    cv.Optional(CONF_BASELINE): cv.hex_uint16_t,
    cv.Optional(CONF_TEMPERATURE): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_HUMIDITY): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_WAKE_PIN): pins.gpio_output_pin_schema,
}).extend(cv.polling_component_schema('60s')).extend(i2c.i2c_device_schema(0x5A))


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield i2c.register_i2c_device(var, config)

    sens = yield sensor.new_sensor(config[CONF_ECO2])
    cg.add(var.set_co2(sens))
    sens = yield sensor.new_sensor(config[CONF_TVOC])
    cg.add(var.set_tvoc(sens))

    if CONF_BASELINE in config:
        cg.add(var.set_baseline(config[CONF_BASELINE]))

    if CONF_TEMPERATURE in config:
        sens = yield cg.get_variable(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature(sens))
    if CONF_HUMIDITY in config:
        sens = yield cg.get_variable(config[CONF_HUMIDITY])
        cg.add(var.set_humidity(sens))

    if CONF_WAKE_PIN in config:
        wakepin = yield cg.gpio_pin_expression(config[CONF_WAKE_PIN])
        cg.add(var.set_wakepin(wakepin))
