import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import mqtt
from esphome.const import CONF_AWAY, CONF_BOOST, CONF_SLEEP, CONF_ID, CONF_INTERNAL, \
    CONF_MAX_TEMPERATURE, CONF_MIN_TEMPERATURE, CONF_MODE, CONF_TARGET_TEMPERATURE, \
    CONF_TARGET_TEMPERATURE_HIGH, CONF_TARGET_TEMPERATURE_LOW, CONF_TEMPERATURE_STEP, \
    CONF_VISUAL, CONF_MQTT_ID, CONF_NAME, CONF_FAN_MODE, CONF_SWING_MODE
from esphome.core import CORE, coroutine, coroutine_with_priority

IS_PLATFORM_COMPONENT = True

CODEOWNERS = ['@esphome/core']
climate_ns = cg.esphome_ns.namespace('climate')

Climate = climate_ns.class_('Climate', cg.Nameable)
ClimateCall = climate_ns.class_('ClimateCall')
ClimateTraits = climate_ns.class_('ClimateTraits')

ClimateMode = climate_ns.enum('ClimateMode')
CLIMATE_MODES = {
    'OFF': ClimateMode.CLIMATE_MODE_OFF,
    'AUTO': ClimateMode.CLIMATE_MODE_AUTO,
    'COOL': ClimateMode.CLIMATE_MODE_COOL,
    'HEAT': ClimateMode.CLIMATE_MODE_HEAT,
    'DRY': ClimateMode.CLIMATE_MODE_DRY,
    'FAN_ONLY': ClimateMode.CLIMATE_MODE_FAN_ONLY,
}
validate_climate_mode = cv.enum(CLIMATE_MODES, upper=True)

ClimateFanMode = climate_ns.enum('ClimateFanMode')
CLIMATE_FAN_MODES = {
    'ON': ClimateFanMode.CLIMATE_FAN_ON,
    'OFF': ClimateFanMode.CLIMATE_FAN_OFF,
    'AUTO': ClimateFanMode.CLIMATE_FAN_AUTO,
    'LOW': ClimateFanMode.CLIMATE_FAN_LOW,
    'MEDIUM': ClimateFanMode.CLIMATE_FAN_MEDIUM,
    'HIGH': ClimateFanMode.CLIMATE_FAN_HIGH,
    'MIDDLE': ClimateFanMode.CLIMATE_FAN_MIDDLE,
    'FOCUS': ClimateFanMode.CLIMATE_FAN_FOCUS,
    'DIFFUSE': ClimateFanMode.CLIMATE_FAN_DIFFUSE,
}

validate_climate_fan_mode = cv.enum(CLIMATE_FAN_MODES, upper=True)

ClimateSwingMode = climate_ns.enum('ClimateSwingMode')
CLIMATE_SWING_MODES = {
    'OFF': ClimateSwingMode.CLIMATE_SWING_OFF,
    'BOTH': ClimateSwingMode.CLIMATE_SWING_BOTH,
    'VERTICAL': ClimateSwingMode.CLIMATE_SWING_VERTICAL,
    'HORIZONTAL': ClimateSwingMode.CLIMATE_SWING_HORIZONTAL,
}

validate_climate_swing_mode = cv.enum(CLIMATE_SWING_MODES, upper=True)

# Actions
ControlAction = climate_ns.class_('ControlAction', automation.Action)

CLIMATE_SCHEMA = cv.MQTT_COMMAND_COMPONENT_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(Climate),
    cv.OnlyWith(CONF_MQTT_ID, 'mqtt'): cv.declare_id(mqtt.MQTTClimateComponent),
    cv.Optional(CONF_VISUAL, default={}): cv.Schema({
        cv.Optional(CONF_MIN_TEMPERATURE): cv.temperature,
        cv.Optional(CONF_MAX_TEMPERATURE): cv.temperature,
        cv.Optional(CONF_TEMPERATURE_STEP): cv.temperature,
    }),
    # TODO: MQTT topic options
})


@coroutine
def setup_climate_core_(var, config):
    cg.add(var.set_name(config[CONF_NAME]))
    if CONF_INTERNAL in config:
        cg.add(var.set_internal(config[CONF_INTERNAL]))
    visual = config[CONF_VISUAL]
    if CONF_MIN_TEMPERATURE in visual:
        cg.add(var.set_visual_min_temperature_override(visual[CONF_MIN_TEMPERATURE]))
    if CONF_MAX_TEMPERATURE in visual:
        cg.add(var.set_visual_max_temperature_override(visual[CONF_MAX_TEMPERATURE]))
    if CONF_TEMPERATURE_STEP in visual:
        cg.add(var.set_visual_temperature_step_override(visual[CONF_TEMPERATURE_STEP]))

    if CONF_MQTT_ID in config:
        mqtt_ = cg.new_Pvariable(config[CONF_MQTT_ID], var)
        yield mqtt.register_mqtt_component(mqtt_, config)


@coroutine
def register_climate(var, config):
    if not CORE.has_id(config[CONF_ID]):
        var = cg.Pvariable(config[CONF_ID], var)
    cg.add(cg.App.register_climate(var))
    yield setup_climate_core_(var, config)


CLIMATE_CONTROL_ACTION_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.use_id(Climate),
    cv.Optional(CONF_MODE): cv.templatable(validate_climate_mode),
    cv.Optional(CONF_TARGET_TEMPERATURE): cv.templatable(cv.temperature),
    cv.Optional(CONF_TARGET_TEMPERATURE_LOW): cv.templatable(cv.temperature),
    cv.Optional(CONF_TARGET_TEMPERATURE_HIGH): cv.templatable(cv.temperature),
    cv.Optional(CONF_AWAY): cv.templatable(cv.boolean),
    cv.Optional(CONF_BOOST): cv.templatable(cv.boolean),
    cv.Optional(CONF_SLEEP): cv.templatable(cv.boolean),
    cv.Optional(CONF_FAN_MODE): cv.templatable(validate_climate_fan_mode),
    cv.Optional(CONF_SWING_MODE): cv.templatable(validate_climate_swing_mode),
})


@automation.register_action('climate.control', ControlAction, CLIMATE_CONTROL_ACTION_SCHEMA)
def climate_control_to_code(config, action_id, template_arg, args):
    paren = yield cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    if CONF_MODE in config:
        template_ = yield cg.templatable(config[CONF_MODE], args, ClimateMode)
        cg.add(var.set_mode(template_))
    if CONF_TARGET_TEMPERATURE in config:
        template_ = yield cg.templatable(config[CONF_TARGET_TEMPERATURE], args, float)
        cg.add(var.set_target_temperature(template_))
    if CONF_TARGET_TEMPERATURE_LOW in config:
        template_ = yield cg.templatable(config[CONF_TARGET_TEMPERATURE_LOW], args, float)
        cg.add(var.set_target_temperature_low(template_))
    if CONF_TARGET_TEMPERATURE_HIGH in config:
        template_ = yield cg.templatable(config[CONF_TARGET_TEMPERATURE_HIGH], args, float)
        cg.add(var.set_target_temperature_high(template_))
    if CONF_AWAY in config:
        template_ = yield cg.templatable(config[CONF_AWAY], args, bool)
        cg.add(var.set_away(template_))
    if CONF_BOOST in config:
        template_ = yield cg.templatable(config[CONF_BOOST], args, bool)
        cg.add(var.set_boost(template_))
    if CONF_SLEEP in config:
        template_ = yield cg.templatable(config[CONF_SLEEP], args, bool)
        cg.add(var.set_night(template_))
    if CONF_FAN_MODE in config:
        template_ = yield cg.templatable(config[CONF_FAN_MODE], args, ClimateFanMode)
        cg.add(var.set_fan_mode(template_))
    if CONF_SWING_MODE in config:
        template_ = yield cg.templatable(config[CONF_SWING_MODE], args, ClimateSwingMode)
        cg.add(var.set_swing_mode(template_))
    yield var


@coroutine_with_priority(100.0)
def to_code(config):
    cg.add_define('USE_CLIMATE')
    cg.add_global(climate_ns.using)
