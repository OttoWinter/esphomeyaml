#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/core/automation.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace accumulator {

class AccumulatorSensor : public sensor::Sensor, public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void set_sensor(Sensor *sensor) { sensor_ = sensor; }
  void set_reset(bool reset) { reset_ = reset; }
  void set_reset_value(float reset_value) { initial_value_ = reset_value; }

  void set_save_on_value_delta(float value) { save_on_value_delta_ = value; }
  void set_save_min_interval(int value) { save_min_interval_ = value; }
  void set_save_max_interval(int value) { save_max_interval_ = value; }

  void reset() { initial_value_ = -sensor_->state; }

  void process_sensor_value(float value);
  bool needs_save(double value);
  void save(double value);

 protected:
  std::string unit_of_measurement() override { return this->sensor_->get_unit_of_measurement(); }
  std::string icon() override { return this->sensor_->get_icon(); }
  int8_t accuracy_decimals() override { return this->sensor_->get_accuracy_decimals() + 2; }

  sensor::Sensor *sensor_;
  ESPPreferenceObject rtc_;

  // settings
  float save_on_value_delta_{0.0f};
  int save_min_interval_{0};
  int save_max_interval_{0};
  double initial_value_{0.0f};
  bool reset_;

  // Track last save
  double last_saved_value_ = 0;
  uint last_saved_time_ = 0;
};

template<typename... Ts> class ResetAction : public Action<Ts...> {
 public:
  explicit ResetAction(AccumulatorSensor *parent) : parent_(parent) {}

  void play(Ts... x) override { this->parent_->reset(); }

 protected:
  AccumulatorSensor *parent_;
};

}  // namespace accumulator
}  // namespace esphome
