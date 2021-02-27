#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SENSOR

#include "esphome/components/sensor/sensor.h"
#include "lora_component.h"

namespace esphome {
namespace lora {

class LoraSensorComponent : public LoraComponent {
 public:
  /** Construct this LoraSensorComponent instance with the provided friendly_name and sensor
   *
   * Note the sensor is never stored and is only used for initializing some values of this class.
   * If sensor is nullptr, then automatic initialization of these fields is disabled.
   *
   * @param sensor The sensor, this can be null to disable automatic setup.
   */
  explicit LoraSensorComponent(sensor::Sensor *sensor);

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  /// Override setup.
  void setup() override;

  void dump_config() override;

  bool publish_state(float value);
  bool send_initial_state() override;
  bool is_internal() override;

 protected:
  /// Override for MQTTComponent, returns "sensor".
  std::string component_type() const override;

  std::string friendly_name() const override;

  std::string unique_id() override;

  sensor::Sensor *sensor_;
};

}  // namespace lora
}  // namespace esphome

#endif
