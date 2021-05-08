#pragma once

#include "esphome/core/component.h"
#include "esphome/core/esphal.h"
#include "esphome/core/defines.h"
#include "esphome/components/sensor/sensor.h"

#ifdef USE_HARDWARE_PULSECOUNTER
#include <driver/pcnt.h>
#endif

namespace esphome {
namespace pulse_counter {

enum PulseCounterCountMode {
  PULSE_COUNTER_DISABLE = 0,
  PULSE_COUNTER_INCREMENT,
  PULSE_COUNTER_DECREMENT,
};

#ifdef USE_HARDWARE_PULSECOUNTER
using pulse_counter_t = int16_t;
#else
using pulse_counter_t = int32_t;
#endif

struct PulseCounterStorage {
  bool pulse_counter_setup(GPIOPin *pin);
  pulse_counter_t read_raw_value();

  static void gpio_intr(PulseCounterStorage *arg);

#ifdef USE_SOFTWARE_PULSECOUNTER
  volatile pulse_counter_t counter{0};
  volatile uint32_t last_pulse{0};
#endif

  GPIOPin *pin;
#ifdef USE_HARDWARE_PULSECOUNTER
  pcnt_unit_t pcnt_unit;
#endif
#ifdef USE_SOFTWARE_PULSECOUNTER
  ISRInternalGPIOPin *isr_pin;
#endif
  PulseCounterCountMode rising_edge_mode{PULSE_COUNTER_INCREMENT};
  PulseCounterCountMode falling_edge_mode{PULSE_COUNTER_DISABLE};
  uint32_t filter_us{0};
#ifdef USE_HARDWARE_PULSECOUNTER
  bool hardware{true};
#else
  bool hardware{false};
#endif
  pulse_counter_t last_value{0};
};

class PulseCounterSensor : public sensor::Sensor, public PollingComponent {
 public:
  void set_pin(GPIOPin *pin) { pin_ = pin; }
  void set_rising_edge_mode(PulseCounterCountMode mode) { storage_.rising_edge_mode = mode; }
  void set_falling_edge_mode(PulseCounterCountMode mode) { storage_.falling_edge_mode = mode; }
  void set_filter_us(uint32_t filter) { storage_.filter_us = filter; }
  void set_hardware_pulsecounter(bool mode) { storage_.hardware = mode; }
  void set_total_sensor(sensor::Sensor *total_sensor) { total_sensor_ = total_sensor; }

  /// Unit of measurement is "pulses/min".
  void setup() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void dump_config() override;

 protected:
  GPIOPin *pin_;
  PulseCounterStorage storage_;
  uint32_t current_total_ = 0;
  sensor::Sensor *total_sensor_;
};

#ifdef USE_HARDWARE_PULSECOUNTER
extern pcnt_unit_t next_pcnt_unit;
#endif

}  // namespace pulse_counter
}  // namespace esphome
