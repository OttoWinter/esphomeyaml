import voluptuous as vol

from esphome import automation
from esphome.components import cover
import esphome.config_validation as cv
from esphome.const import CONF_CLOSE_ACTION, CONF_ID, CONF_LAMBDA, CONF_NAME, \
    CONF_OPEN_ACTION, CONF_OPTIMISTIC, CONF_STOP_ACTION
from esphome.cpp_generator import Pvariable, add, process_lambda
from esphome.cpp_helpers import setup_component
from esphome.cpp_types import App, NoArg, optional

TemplateCover = cover.cover_ns.class_('TemplateCover', cover.Cover)

PLATFORM_SCHEMA = cv.nameable(cover.COVER_PLATFORM_SCHEMA.extend({
    cv.GenerateID(): cv.declare_variable_id(TemplateCover),
    vol.Optional(CONF_LAMBDA): cv.lambda_,
    vol.Optional(CONF_OPTIMISTIC): cv.boolean,
    vol.Optional(CONF_OPEN_ACTION): automation.validate_automation(single=True),
    vol.Optional(CONF_CLOSE_ACTION): automation.validate_automation(single=True),
    vol.Optional(CONF_STOP_ACTION): automation.validate_automation(single=True),
}).extend(cv.COMPONENT_SCHEMA.schema), cv.has_at_least_one_key(CONF_LAMBDA, CONF_OPTIMISTIC))


def to_code(config):
    rhs = App.make_template_cover(config[CONF_NAME])
    var = Pvariable(config[CONF_ID], rhs)

    cover.setup_cover(var, config)
    setup_component(var, config)

    if CONF_LAMBDA in config:
        for template_ in process_lambda(config[CONF_LAMBDA], [],
                                        return_type=optional.template(cover.CoverState)):
            yield
        add(var.set_state_lambda(template_))
    if CONF_OPEN_ACTION in config:
        automation.build_automation(var.get_open_trigger(), NoArg,
                                    config[CONF_OPEN_ACTION])
    if CONF_CLOSE_ACTION in config:
        automation.build_automation(var.get_close_trigger(), NoArg,
                                    config[CONF_CLOSE_ACTION])
    if CONF_STOP_ACTION in config:
        automation.build_automation(var.get_stop_trigger(), NoArg,
                                    config[CONF_STOP_ACTION])
    if CONF_OPTIMISTIC in config:
        add(var.set_optimistic(config[CONF_OPTIMISTIC]))


BUILD_FLAGS = '-DUSE_TEMPLATE_COVER'


def to_hass_config(data, config):
    ret = cover.core_to_hass_config(data, config)
    if ret is None:
        return None
    if CONF_OPTIMISTIC in config:
        ret['optimistic'] = config[CONF_OPTIMISTIC]
    return ret
