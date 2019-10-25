import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, esp32_ble_tracker
from esphome.const import CONF_SERVICE_UUID, CONF_MAC_ADDRESS, CONF_ID, UNIT_DECIBEL, ICON_SIGNAL

DEPENDENCIES = ['esp32_ble_tracker']

ble_rssi_ns = cg.esphome_ns.namespace('ble_rssi')
BLERSSISensor = ble_rssi_ns.class_('BLERSSISensor', sensor.Sensor, cg.Component,
                                   esp32_ble_tracker.ESPBTDeviceListener)

CONFIG_SCHEMA = cv.All(sensor.sensor_schema(UNIT_DECIBEL, ICON_SIGNAL, 0).extend({
    cv.GenerateID(): cv.declare_id(BLERSSISensor),
    cv.Optional(CONF_MAC_ADDRESS): cv.mac_address,
    cv.Optional(CONF_SERVICE_UUID) : cv.bt_uuid,
}).extend(esp32_ble_tracker.ESP_BLE_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA),
cv.has_exactly_one_key(CONF_MAC_ADDRESS, CONF_SERVICE_UUID))


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield esp32_ble_tracker.register_ble_device(var, config)
    yield sensor.register_sensor(var, config)

    if CONF_MAC_ADDRESS in config:
        cg.add(var.set_address(config[CONF_MAC_ADDRESS].as_hex))

    if CONF_SERVICE_UUID in config:
        if config[CONF_SERVICE_UUID].get_uuid_format() == cv.bt_uuid16_format:
            cg.add(var.set_service_uuid16(config[CONF_SERVICE_UUID].as_hex))
        elif config[CONF_SERVICE_UUID].get_uuid_format() == cv.bt_uuid32_format:
            cg.add(var.set_service_uuid32(config[CONF_SERVICE_UUID].as_hex))
        elif config[CONF_SERVICE_UUID].get_uuid_format() == cv.bt_uuid128_format:
            uuid128 = config[CONF_SERVICE_UUID].as_hex_array
            cg.add(var.set_service_uuid128(uuid128))
