import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi, canbus
from esphome.const import CONF_ID
from esphome.components.canbus import CanbusComponent

print("mcp2515.canbus.py")
AUTO_LOAD = ['canbus']
DEPENDENCIES = ['spi']

mcp2515_ns = cg.esphome_ns.namespace('mcp2515')
mcp2515 = mcp2515_ns.class_('MCP2515', CanbusComponent, spi.SPIDevice)

CONFIG_SCHEMA = canbus.CONFIG_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(mcp2515),
}).extend(spi.SPI_DEVICE_SCHEMA)


def to_code(config):
    rhs = mcp2515.new()
    var = cg.Pvariable(config[CONF_ID], rhs)
    yield canbus.register_canbus(var, config)
    yield spi.register_spi_device(var, config)
