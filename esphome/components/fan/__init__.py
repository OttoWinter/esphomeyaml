from esphome.automation import ACTION_REGISTRY, maybe_simple_id
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import mqtt
from esphome.const import CONF_ID, CONF_INTERNAL, CONF_MQTT_ID, CONF_OSCILLATING, \
    CONF_OSCILLATION_COMMAND_TOPIC, CONF_OSCILLATION_STATE_TOPIC, CONF_SPEED, \
    CONF_SPEED_COMMAND_TOPIC, CONF_SPEED_STATE_TOPIC, CONF_NAME
from esphome.core import CORE, coroutine

IS_PLATFORM_COMPONENT = True

fan_ns = cg.esphome_ns.namespace('fan')
FanState = fan_ns.class_('FanState', cg.Nameable, cg.Component)
MakeFan = cg.Application.struct('MakeFan')

# Actions
TurnOnAction = fan_ns.class_('TurnOnAction', cg.Action)
TurnOffAction = fan_ns.class_('TurnOffAction', cg.Action)
ToggleAction = fan_ns.class_('ToggleAction', cg.Action)

FanSpeed = fan_ns.enum('FanSpeed')
FAN_SPEEDS = {
    'OFF': FanSpeed.FAN_SPEED_OFF,
    'LOW': FanSpeed.FAN_SPEED_LOW,
    'MEDIuM': FanSpeed.FAN_SPEED_MEDIUM,
    'HIGH': FanSpeed.FAN_SPEED_HIGH,
}

FAN_SCHEMA = cv.MQTT_COMMAND_COMPONENT_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(FanState),
    cv.OnlyWith(CONF_MQTT_ID, 'mqtt'): cv.declare_id(mqtt.MQTTFanComponent),
    cv.Optional(CONF_OSCILLATION_STATE_TOPIC): cv.All(cv.requires_component('mqtt'),
                                                      cv.publish_topic),
    cv.Optional(CONF_OSCILLATION_COMMAND_TOPIC): cv.All(cv.requires_component('mqtt'),
                                                        cv.subscribe_topic),
})


@coroutine
def setup_fan_core_(var, config):
    cg.add(var.set_name(config[CONF_NAME]))
    if CONF_INTERNAL in config:
        cg.add(var.set_internal(config[CONF_INTERNAL]))

    if CONF_MQTT_ID in config:
        mqtt_ = cg.new_Pvariable(config[CONF_MQTT_ID], var)
        yield mqtt.register_mqtt_component(mqtt_, config)

        if CONF_OSCILLATION_STATE_TOPIC in config:
            cg.add(mqtt_.set_custom_oscillation_state_topic(config[CONF_OSCILLATION_STATE_TOPIC]))
        if CONF_OSCILLATION_COMMAND_TOPIC in config:
            cg.add(mqtt_.set_custom_oscillation_command_topic(
                config[CONF_OSCILLATION_COMMAND_TOPIC]))
        if CONF_SPEED_STATE_TOPIC in config:
            cg.add(mqtt_.set_custom_speed_state_topic(config[CONF_SPEED_STATE_TOPIC]))
        if CONF_SPEED_COMMAND_TOPIC in config:
            cg.add(mqtt_.set_custom_speed_command_topic(config[CONF_SPEED_COMMAND_TOPIC]))


@coroutine
def register_fan(var, config):
    if not CORE.has_id(config[CONF_ID]):
        var = cg.Pvariable(config[CONF_ID], var)
    cg.add(cg.App.register_fan(var))
    yield setup_fan_core_(var, config)


@coroutine
def create_fan_state(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield register_fan(var, config)
    yield var


FAN_ACTION_SCHEMA = maybe_simple_id({
    cv.Required(CONF_ID): cv.use_id(FanState),
})


@ACTION_REGISTRY.register('fan.toggle', FAN_ACTION_SCHEMA)
def fan_toggle_to_code(config, action_id, template_arg, args):
    var = yield cg.get_variable(config[CONF_ID])
    type = ToggleAction.template(template_arg)
    rhs = type.new(var)
    yield cg.Pvariable(action_id, rhs, type=type)


@ACTION_REGISTRY.register('fan.turn_off', FAN_ACTION_SCHEMA)
def fan_turn_off_to_code(config, action_id, template_arg, args):
    var = yield cg.get_variable(config[CONF_ID])
    type = TurnOffAction.template(template_arg)
    rhs = type.new(var)
    yield cg.Pvariable(action_id, rhs, type=type)


@ACTION_REGISTRY.register('fan.turn_on', maybe_simple_id({
    cv.Required(CONF_ID): cv.use_id(FanState),
    cv.Optional(CONF_OSCILLATING): cv.templatable(cv.boolean),
    cv.Optional(CONF_SPEED): cv.templatable(cv.enum(FAN_SPEEDS, upper=True)),
}))
def fan_turn_on_to_code(config, action_id, template_arg, args):
    var = yield cg.get_variable(config[CONF_ID])
    type = TurnOnAction.template(template_arg)
    rhs = type.new(var)
    action = cg.Pvariable(action_id, rhs, type=type)
    if CONF_OSCILLATING in config:
        template_ = yield cg.templatable(config[CONF_OSCILLATING], args, bool)
        cg.add(action.set_oscillating(template_))
    if CONF_SPEED in config:
        template_ = yield cg.templatable(config[CONF_SPEED], args, FanSpeed)
        cg.add(action.set_speed(template_))
    yield action


def to_code(config):
    cg.add_define('USE_FAN')
    cg.add_global(fan_ns.using)
