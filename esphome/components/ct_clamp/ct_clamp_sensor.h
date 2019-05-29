#pragma once

#include "esphome/core/component.h"
#include "esphome/core/esphal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"

namespace esphome {
namespace ct_clamp {

class CTClampSensor : public sensor::Sensor, public PollingComponent {
 public:
  /// Update CT Clamp sensor values.
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_sample_size(uint32_t sample_size) { this->sample_size_ = sample_size; }

  void set_source(voltage_sampler::VoltageSampler *source) { source_ = source; }

 protected:
  uint32_t sample_size_;
  voltage_sampler::VoltageSampler *source_;

  float offset_ = 0.5f;
};

}  // namespace ct_clamp
}  // namespace esphome
