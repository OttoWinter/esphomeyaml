#include "xiaomi_ble.h"
#include "esphome/core/log.h"

#ifdef ARDUINO_ARCH_ESP32

namespace esphome {
namespace xiaomi_ble {

static const char *TAG = "xiaomi_ble";

bool parse_xiaomi_data_byte(uint8_t data_type, const uint8_t *data, uint8_t data_length, XiaomiParseResult &result) {
if (result.type == XiaomiParseResult::TYPE_MISCALE) {
  switch(data_type) {
    case 0x00:{
    const uint16_t weight = uint16_t(data[1]) | (uint16_t(data[2]) << 8);
    if (data[0] == 0x22 || data[0] == 0xa2)
      result.weight = weight * 0.01f / 2.0f;
    else if (data[0] == 0x03 || data[0] == 0xb2)
      result.weight = weight * 0.01f * 0.453592;
    else
      return false;
      return true;
  }
  case 0x01: {
    const uint16_t weight = uint16_t(data[11]) | (uint16_t(data[12]) << 8);
    if (data[0] == 0x02)
      result.weight = weight * 0.01f / 2.0f;
    else if (data[0] == 0x03)
      result.weight = weight * 0.01f * 0.453592;
    else
      return false;
      return true;
    }
  }
}

  switch (data_type) {
    case 0x0D: {  // temperature+humidity, 4 bytes, 16-bit signed integer (LE) each, 0.1 °C, 0.1 %
      if (data_length != 4)
        return false;
      const int16_t temperature = uint16_t(data[0]) | (uint16_t(data[1]) << 8);
      const int16_t humidity = uint16_t(data[2]) | (uint16_t(data[3]) << 8);
      result.temperature = temperature / 10.0f;
      result.humidity = humidity / 10.0f;
      return true;
    }
    case 0x0A: {  // battery, 1 byte, 8-bit unsigned integer, 1 %
      if (data_length != 1)
        return false;
      result.battery_level = data[0];
      return true;
    }
    case 0x06: {  // humidity, 2 bytes, 16-bit signed integer (LE), 0.1 %
      if (data_length != 2)
        return false;
      const int16_t humidity = uint16_t(data[0]) | (uint16_t(data[1]) << 8);
      result.humidity = humidity / 10.0f;
      return true;
    }
    case 0x04: {  // temperature, 2 bytes, 16-bit signed integer (LE), 0.1 °C
      if (data_length != 2)
        return false;
      const int16_t temperature = uint16_t(data[0]) | (uint16_t(data[1]) << 8);
      result.temperature = temperature / 10.0f;
      return true;
    }
    case 0x09: {  // conductivity, 2 bytes, 16-bit unsigned integer (LE), 1 µS/cm
      if (data_length != 2)
        return false;
      const uint16_t conductivity = uint16_t(data[0]) | (uint16_t(data[1]) << 8);
      result.conductivity = conductivity;
      return true;
    }
    case 0x07: {  // illuminance, 3 bytes, 24-bit unsigned integer (LE), 1 lx
      if (data_length != 3)
        return false;
      const uint32_t illuminance = uint32_t(data[0]) | (uint32_t(data[1]) << 8) | (uint32_t(data[2]) << 16);
      result.illuminance = illuminance;
      return true;
    }
    case 0x08: {  // soil moisture, 1 byte, 8-bit unsigned integer, 1 %
      if (data_length != 1)
        return false;
      result.moisture = data[0];
      return true;
    }
    default:
      return false;
  }
}
optional<XiaomiParseResult> parse_xiaomi(const esp32_ble_tracker::ESPBTDevice &device) {
  if (!device.get_service_data_uuid().has_value()) {
    // ESP_LOGVV(TAG, "Xiaomi no service data");
    return {};
  }

  if (!device.get_service_data_uuid()->contains(0x95, 0xFE)) and !device.get_service_data_uuid()->contains(0x1D, 0x18) and !device.get_service_data_uuid()->contains(0x1B, 0x18)) {
    // ESP_LOGVV(TAG, "Xiaomi no service data UUID magic bytes");
    return {};
  }

  const auto *raw = reinterpret_cast<const uint8_t *>(device.get_service_data().data());

  if (device.get_service_data().size() < 9) {
    // ESP_LOGVV(TAG, "Xiaomi service data too short!");
    return {};
  }

  bool is_mijia = (raw[1] & 0x20) == 0x20 && raw[2] == 0xAA && raw[3] == 0x01;
  bool is_miflora = (raw[1] & 0x20) == 0x20 && raw[2] == 0x98 && raw[3] == 0x00;
  bool is_miscale =  raw[3] == 0xE3 && raw[4] == 0x07 && raw[5] == 0x08;
  bool is_mibfs = raw[2] == 0xE3 && raw[3] == 0x07 && raw[4] == 0x08;
  if (!is_mijia && !is_miflora && !is_miscale && !is_mibfs) {
    // ESP_LOGVV(TAG, "Xiaomi no magic bytes");
    return {};
  }

  bool success;
  XiaomiParseResult result;
if (is_mijia || is_miflora) {
  uint8_t raw_offset = is_mijia ? 11 : 12;

  const uint8_t raw_type = raw[raw_offset];
  const uint8_t data_length = raw[raw_offset + 2];
  const uint8_t *data = &raw[raw_offset + 3];
  const uint8_t expected_length = data_length + raw_offset + 3;
  const uint8_t actual_length = device.get_service_data().size();
  if (expected_length != actual_length) {
    // ESP_LOGV(TAG, "Xiaomi %s data length mismatch (%u != %d)", raw_type, expected_length, actual_length);
    return {};
  }
  result.type = is_miflora ? XiaomiParseResult::TYPE_MIFLORA : XiaomiParseResult::TYPE_MIJIA;
  success = parse_xiaomi_data_byte(raw_type, data, data_length, result);
} else {
  const uint8_t *data = &raw[0];
  const uint8_t data_length = 2;

  result.type = XiaomiParseResult::TYPE_MISCALE;
  if (device.get_service_data_uuid()->contains(0x1B, 0x18) && !is_mibfs) {
    const uint8_t raw_type = 0x0;
    success = parse_xiaomi_data_byte(raw_type, data, data_length, result);
  } else {
    const uint8_t raw_type = 0x1;
    success = parse_xiaomi_data_byte(raw_type, data, data_length, result);
  }
}
  if (!success)
    return {};
  return result;
}

bool XiaomiListener::parse_device(const esp32_ble_tracker::ESPBTDevice &device) {
  auto res = parse_xiaomi(device);
  if (!res.has_value())
    return false;

  const char *name = res->type == XiaomiParseResult::TYPE_MIFLORA ? "Mi Flora" : (res->type == XiaomiParseResult::TYPE_MIJIA ? "Mi Jia": "Mi Scale");

  ESP_LOGD(TAG, "Got Xiaomi %s (%s):", name, device.address_str().c_str());

  if (res->temperature.has_value()) {
    ESP_LOGD(TAG, "  Temperature: %.1f°C", *res->temperature);
  }
  if (res->humidity.has_value()) {
    ESP_LOGD(TAG, "  Humidity: %.1f%%", *res->humidity);
  }
  if (res->battery_level.has_value()) {
    ESP_LOGD(TAG, "  Battery Level: %.0f%%", *res->battery_level);
  }
  if (res->conductivity.has_value()) {
    ESP_LOGD(TAG, "  Conductivity: %.0fµS/cm", *res->conductivity);
  }
  if (res->illuminance.has_value()) {
    ESP_LOGD(TAG, "  Illuminance: %.0flx", *res->illuminance);
  }
  if (res->moisture.has_value()) {
    ESP_LOGD(TAG, "  Moisture: %.0f%%", *res->moisture);
  }
  if (res->weight.has_value()) {
    ESP_LOGD(TAG, "  Weight: %.1fkg", *res->weight);
  }
  return true;
}

}  // namespace xiaomi_ble
}  // namespace esphome

#endif
