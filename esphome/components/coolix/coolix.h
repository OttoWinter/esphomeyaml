#pragma once

#include "esphome/components/climate_ir/climate_ir.h"

namespace esphome {
namespace coolix {

// Temperature
const uint8_t COOLIX_TEMP_MIN = 17;  // Celsius
const uint8_t COOLIX_TEMP_MAX = 30;  // Celsius

class CoolixClimate : public climate_ir::ClimateIR {
 public:
  CoolixClimate()
      : climate_ir::ClimateIR(COOLIX_TEMP_MIN, COOLIX_TEMP_MAX, 1.0f, true, true,
                              {climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_LOW, climate::CLIMATE_FAN_MEDIUM,
                               climate::CLIMATE_FAN_HIGH},
                              {climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_VERTICAL}) {}

 protected:
  /// Transmit via IR the state of this climate controller.
  void transmit_state() override;
  /// Handle received IR Buffer
  bool on_receive(remote_base::RemoteReceiveData data) override;
};

}  // namespace coolix
}  // namespace esphome
