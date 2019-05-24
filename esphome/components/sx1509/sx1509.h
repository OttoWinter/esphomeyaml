#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/output/float_output.h"
#include "esphome/core/component.h"
#include "sx1509_registers.h"

namespace esphome {
namespace sx1509 {

// These are used for setting LED driver to linear or log mode:
#define LINEAR 0
#define LOGARITHMIC 1

// These are used for clock config:
#define INTERNAL_CLOCK_2MHZ 2
#define EXTERNAL_CLOCK 1

#define SOFTWARE_RESET 0
#define HARDWARE_RESET 1

#define ANALOG_OUTPUT 0x3 // To set a pin mode for PWM output

class SX1509Component;

class SX1509FloatOutputChannel : public output::FloatOutput {
public:
  SX1509FloatOutputChannel(SX1509Component *parent, uint8_t channel)
      : parent_(parent), channel_(channel) {}
  void setup_channel();

protected:
  void write_state(float state) override;

  SX1509Component *parent_;
  uint8_t channel_;
};

/// SX1509 float output component.
class SX1509Component : public Component, public i2c::I2CDevice {

public:
  SX1509Component() {}

  SX1509FloatOutputChannel *create_float_output_channel(uint8_t channel);

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void loop() override;

protected:
  friend SX1509FloatOutputChannel;
  std::vector<SX1509FloatOutputChannel *> float_output_channels_{};

  void pinMode(uint8_t channel, uint8_t inOut);
  void ledDriverInit(uint8_t channel, uint8_t freq = 1, bool log = false);
  void clock(uint8_t oscSource = 2, uint8_t oscDivider = 1,
             uint8_t oscPinFunction = 0, uint8_t oscFreqOut = 0);
  void digitalWrite(uint8_t channel, uint8_t highLow);
  void set_channel_value_(uint8_t channel, uint8_t iOn);

  // Pin definitions:
  uint8_t pinInterrupt_;
  uint8_t pinOscillator_;
  uint8_t pinReset_;
  // variables:
  u_long _clkX;
  uint8_t frequency_ = 0;

  bool update_{true};

  byte REG_I_ON[16] = {REG_I_ON_0,  REG_I_ON_1,  REG_I_ON_2,  REG_I_ON_3,
                       REG_I_ON_4,  REG_I_ON_5,  REG_I_ON_6,  REG_I_ON_7,
                       REG_I_ON_8,  REG_I_ON_9,  REG_I_ON_10, REG_I_ON_11,
                       REG_I_ON_12, REG_I_ON_13, REG_I_ON_14, REG_I_ON_15};

  byte REG_T_ON[16] = {REG_T_ON_0,  REG_T_ON_1,  REG_T_ON_2,  REG_T_ON_3,
                       REG_T_ON_4,  REG_T_ON_5,  REG_T_ON_6,  REG_T_ON_7,
                       REG_T_ON_8,  REG_T_ON_9,  REG_T_ON_10, REG_T_ON_11,
                       REG_T_ON_12, REG_T_ON_13, REG_T_ON_14, REG_T_ON_15};

  byte REG_OFF[16] = {REG_OFF_0,  REG_OFF_1,  REG_OFF_2,  REG_OFF_3,
                      REG_OFF_4,  REG_OFF_5,  REG_OFF_6,  REG_OFF_7,
                      REG_OFF_8,  REG_OFF_9,  REG_OFF_10, REG_OFF_11,
                      REG_OFF_12, REG_OFF_13, REG_OFF_14, REG_OFF_15};

  byte REG_T_RISE[16] = {
      0xFF,          0xFF,          0xFF,          0xFF,
      REG_T_RISE_4,  REG_T_RISE_5,  REG_T_RISE_6,  REG_T_RISE_7,
      0xFF,          0xFF,          0xFF,          0xFF,
      REG_T_RISE_12, REG_T_RISE_13, REG_T_RISE_14, REG_T_RISE_15};

  byte REG_T_FALL[16] = {
      0xFF,          0xFF,          0xFF,          0xFF,
      REG_T_FALL_4,  REG_T_FALL_5,  REG_T_FALL_6,  REG_T_FALL_7,
      0xFF,          0xFF,          0xFF,          0xFF,
      REG_T_FALL_12, REG_T_FALL_13, REG_T_FALL_14, REG_T_FALL_15};
};

} // namespace sx1509
} // namespace esphome
