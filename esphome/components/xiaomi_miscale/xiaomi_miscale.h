#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/xiaomi_ble/xiaomi_ble.h"

#ifdef ARDUINO_ARCH_ESP32

namespace esphome {
namespace xiaomi_miscale {

class XiaomiMiscale : public Component, public esp32_ble_tracker::ESPBTDeviceListener {
 public:
  void set_address(uint64_t address) { address_ = address; }

  bool parse_device(const esp32_ble_tracker::ESPBTDevice &device) override {

    if (device.address_uint64() != this->address_)
      return false;

    auto res = xiaomi_ble::parse_xiaomi(device);
    if (!res.has_value())
      return false;

    if (res->weight.has_value() && this->weight_ != nullptr)
      this->weight_->publish_state(*res->weight);
    if (res->battery_level.has_value() && this->battery_level_ != nullptr)
      this->battery_level_->publish_state(*res->battery_level);
    return true;
  }

  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void set_weight(sensor::Sensor *weight) { weight_ = weight; }
  void set_battery_level(sensor::Sensor *battery_level) { battery_level_ = battery_level; }

 protected:
  uint64_t address_;
  sensor::Sensor *weight_{nullptr};
  sensor::Sensor *battery_level_{nullptr};
};

}  // namespace xiaomi_miscale
}  // namespace esphome

#endif
