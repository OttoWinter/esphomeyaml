import math

from esphome import pins
from esphome.components import output
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_BIT_DEPTH, CONF_CHANNEL, CONF_FREQUENCY, \
    CONF_ID, CONF_PIN, ESP_PLATFORM_ESP32

ESP_PLATFORMS = [ESP_PLATFORM_ESP32]


def validate_frequency_bit_depth(obj):
    frequency = obj[CONF_FREQUENCY]
    bit_depth = obj[CONF_BIT_DEPTH]
    apb_freq = 80e6
    max_freq = apb_freq / (2**bit_depth)
    if frequency > max_freq:
        raise cv.Invalid('Maximum frequency for bit depth {} is {}Hz. Please decrease the '
                         'bit_depth.'.format(bit_depth, int(math.floor(max_freq))))
    # LEDC_DIV_NUM_HSTIMER is 15-bit unsigned integer
    # lower 8 bits represent fractional part
    max_div_num = ((1 << 16) - 1) / 256.0
    min_freq = apb_freq / (max_div_num * (2**bit_depth))
    if frequency < min_freq:
        raise cv.Invalid('Minimum frequency for bit depth {} is {}Hz. Please increase the '
                         'bit_depth.'.format(bit_depth, int(math.ceil(min_freq))))
    return obj


ledc_ns = cg.esphome_ns.namespace('ledc')
LEDCOutput = ledc_ns.class_('LEDCOutput', output.FloatOutput, cg.Component)

CONFIG_SCHEMA = cv.All(output.FLOAT_OUTPUT_SCHEMA.extend({
    cv.Required(CONF_ID): cv.declare_id(LEDCOutput),
    cv.Required(CONF_PIN): pins.internal_gpio_output_pin_schema,
    cv.Optional(CONF_FREQUENCY, default='1kHz'): cv.frequency,
    cv.Optional(CONF_BIT_DEPTH, default=12): cv.int_range(min=1, max=15),
    cv.Optional(CONF_CHANNEL): cv.int_range(min=0, max=15),
}).extend(cv.COMPONENT_SCHEMA), validate_frequency_bit_depth)


def to_code(config):
    gpio = yield cg.gpio_pin_expression(config[CONF_PIN])
    var = cg.new_Pvariable(config[CONF_ID], gpio)
    yield cg.register_component(var, config)
    yield output.register_output(var, config)
    if CONF_CHANNEL in config:
        cg.add(var.set_channel(config[CONF_CHANNEL]))
