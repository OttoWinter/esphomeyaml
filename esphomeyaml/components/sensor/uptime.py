import voluptuous as vol

from esphomeyaml.components import sensor
import esphomeyaml.config_validation as cv
from esphomeyaml.const import CONF_ID, CONF_NAME, CONF_UPDATE_INTERVAL
from esphomeyaml.cpp_generator import Pvariable
from esphomeyaml.cpp_helpers import setup_component
from esphomeyaml.cpp_types import App

UptimeSensor = sensor.sensor_ns.class_('UptimeSensor', sensor.PollingSensorComponent)

PLATFORM_SCHEMA = cv.nameable(sensor.SENSOR_PLATFORM_SCHEMA.extend({
    cv.GenerateID(): cv.declare_variable_id(UptimeSensor),
    vol.Optional(CONF_UPDATE_INTERVAL): cv.update_interval,
}).extend(cv.COMPONENT_SCHEMA.schema))


def to_code(config):
    rhs = App.make_uptime_sensor(config[CONF_NAME], config.get(CONF_UPDATE_INTERVAL))
    uptime = Pvariable(config[CONF_ID], rhs)

    sensor.setup_sensor(uptime, config)
    setup_component(uptime, config)


BUILD_FLAGS = '-DUSE_UPTIME_SENSOR'


def to_hass_config(data, config):
    return sensor.core_to_hass_config(data, config)
