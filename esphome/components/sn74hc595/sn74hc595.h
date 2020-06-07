#pragma once

#include "esphome/core/component.h"
#include "esphome/core/esphal.h"

namespace esphome {
namespace sn74hc595 {

class SN74HC595Component : public Component {
 public:
  SN74HC595Component() = default;

  void setup() override;
  float get_setup_priority() const override;
  void dump_config() override;

  void set_data_pin(GPIOPin *pin) { data_pin_ = pin; }
  void set_clock_pin(GPIOPin *pin) { clock_pin_ = pin; }
  void set_latch_pin(GPIOPin *pin) { latch_pin_ = pin; }
  void set_oe_pin(GPIOPin *pin) { oe_pin_ = pin; have_oe_pin_=true; }
  void set_sr_count(uint8_t count) { sr_count_ = count; }

 protected:
  friend class SN74HC595GPIOPin;
  bool digital_read(uint8_t pin);
  void digital_write(uint8_t pin, bool value);
  bool write_gpio_();

  GPIOPin *data_pin_;
  GPIOPin *clock_pin_;
  GPIOPin *latch_pin_;
  GPIOPin *oe_pin_;
  uint8_t sr_count_;
  bool have_oe_pin_{false};
  uint32_t output_bits_{0x00};
};

/// Helper class to expose a SC74HC595 pin as an internal output GPIO pin.
class SN74HC595GPIOPin : public GPIOPin {
 public:
  SN74HC595GPIOPin(SN74HC595Component *parent, uint8_t pin, bool inverted = false);

  void setup() override;
  bool digital_read() override;
  void digital_write(bool value) override;

 protected:
  SN74HC595Component *parent_;
};

}  // namespace sn74hc595
}  // namespace esphome
