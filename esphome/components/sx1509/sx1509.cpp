#include "sx1509.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sx1509 {

static const char *TAG = "sx1509";

void SX1509Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SX1509Component...");

  ESP_LOGV(TAG, "  Resetting devices...");
  this->write_byte(REG_RESET, 0x12);
  this->write_byte(REG_RESET, 0x34);

  // Communication test.
  uint16_t data;
  this->read_byte_16(REG_INTERRUPT_MASK_A, &data);
  if (data == 0xFF00) {
    clock(INTERNAL_CLOCK_2MHZ);
  } else {
    this->mark_failed();
    return;
  }

  // for (auto *channel : this->float_output_channels_)
  //   channel->setup();

  delayMicroseconds(500);
}

void SX1509Component::dump_config() {
  ESP_LOGCONFIG(TAG, "SX1509:");
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Setting up SX1509 failed!");
  }
}

uint8_t SX1509Component::digital_read(uint8_t pin) {
  uint16_t tempRegDir;
  this->read_byte_16(REG_DIR_B, &tempRegDir);

  if (tempRegDir & (1 << pin)) {
    uint16_t tempRegData;
    this->read_byte_16(REG_DATA_B, &tempRegData);
    if (tempRegData & (1 << pin))
      return 1;
  }
  return 0;
}

void SX1509Component::digital_write(uint8_t pin, uint8_t bit_value) {
  uint16_t temp_reg_dir = 0;
  this->read_byte_16(REG_DIR_B, &temp_reg_dir);

  if ((0xFFFF ^ temp_reg_dir) & (1 << pin))  // If the pin is an output, write high/low
  {
    uint16_t temp_reg_data = 0;
    this->read_byte_16(REG_DATA_B, &temp_reg_data);
    if (bit_value)
      temp_reg_data |= (1 << pin);
    else
      temp_reg_data &= ~(1 << pin);
    this->write_byte_16(REG_DATA_B, temp_reg_data);
  } else  // Otherwise the pin is an input, pull-up/down
  {
    uint16_t temp_pullup;
    this->read_byte_16(REG_PULL_UP_B, &temp_pullup);
    uint16_t temp_pull_down;
    this->read_byte_16(REG_PULL_DOWN_B, &temp_pull_down);

    if (bit_value)  // if HIGH, do pull-up, disable pull-down
    {
      temp_pullup |= (1 << pin);
      temp_pull_down &= ~(1 << pin);
      this->write_byte_16(REG_PULL_UP_B, temp_pullup);
      this->write_byte_16(REG_PULL_DOWN_B, temp_pull_down);
    } else  // If LOW do pull-down, disable pull-up
    {
      temp_pull_down |= (1 << pin);
      temp_pullup &= ~(1 << pin);
      this->write_byte_16(REG_PULL_UP_B, temp_pullup);
      this->write_byte_16(REG_PULL_DOWN_B, temp_pull_down);
    }
  }
}

void SX1509Component::pin_mode(uint8_t pin, uint8_t mode) {
  uint8_t mode_bit;
  if ((mode == OUTPUT) || (mode == ANALOG_OUTPUT))
    mode_bit = 0;
  else
    mode_bit = 1;

  uint16_t temp_reg_dir = 0;
  this->read_byte_16(REG_DIR_B, &temp_reg_dir);

  if (mode_bit)
    temp_reg_dir |= (1 << pin);
  else
    temp_reg_dir &= ~(1 << pin);

  this->write_byte_16(REG_DIR_B, temp_reg_dir);

  if (mode == INPUT_PULLUP)
    digital_write(pin, HIGH);

  if (mode == ANALOG_OUTPUT) {
    setup_led_driver(pin);
  }
}

void SX1509Component::setup_led_driver(uint8_t pin, uint8_t freq, bool log) {
  uint16_t temp_word;
  uint8_t temp_byte;

  this->read_byte_16(REG_INPUT_DISABLE_B, &temp_word);
  temp_word |= (1 << pin);
  this->write_byte_16(REG_INPUT_DISABLE_B, temp_word);

  this->read_byte_16(REG_PULL_UP_B, &temp_word);
  temp_word &= ~(1 << pin);
  this->write_byte_16(REG_PULL_UP_B, temp_word);

  this->read_byte_16(REG_DIR_B, &temp_word);
  temp_word &= ~(1 << pin);  // 0=output
  this->write_byte_16(REG_DIR_B, temp_word);

  this->read_byte(REG_CLOCK, &temp_byte);
  temp_byte |= (1 << 6);   // Internal 2MHz oscillator part 1 (set bit 6)
  temp_byte &= ~(1 << 5);  // Internal 2MHz oscillator part 2 (clear bit 5)
  this->write_byte(REG_CLOCK, temp_byte);

  this->read_byte(REG_MISC, &temp_byte);
  if (log) {
    temp_byte |= (1 << 7);  // set logarithmic mode bank B
    temp_byte |= (1 << 3);  // set logarithmic mode bank A
  } else {
    temp_byte &= ~(1 << 7);  // set linear mode bank B
    temp_byte &= ~(1 << 3);  // set linear mode bank A
  }

  if (clk_x_ == 0)  // Make clckX non-zero
  {
    clk_x_ = 2000000.0 / (1 << (1 - 1));  // Update private clock variable

    uint8_t freq = (1 & 0x07) << 4;  // freq should only be 3 bits from 6:4
    temp_byte |= freq;
  }
  this->write_byte(REG_MISC, temp_byte);

  this->read_byte_16(REG_LED_DRIVER_ENABLE_B, &temp_word);
  temp_word |= (1 << pin);
  this->write_byte_16(REG_LED_DRIVER_ENABLE_B, temp_word);

  this->read_byte_16(REG_DATA_B, &temp_word);
  temp_word &= ~(1 << pin);
  this->write_byte_16(REG_DATA_B, temp_word);
}

void SX1509Component::setup_blink(uint8_t pin, uint8_t t_on, uint8_t t_off, uint8_t on_intensity, uint8_t off_intensity,
                                  uint8_t t_rise, uint8_t t_fall, bool log) {
  setup_led_driver(pin, log);

  t_on &= 0x1F;   // t_on should be a 5-bit value
  t_off &= 0x1F;  // t_off should be a 5-bit value
  off_intensity &= 0x07;
  // Write the time on
  this->write_byte(REG_T_ON[pin], t_on);
  this->write_byte(REG_OFF[pin], (t_off << 3) | off_intensity);
  this->write_byte(REG_I_ON[pin], on_intensity);

  t_rise &= 0x1F;
  t_fall &= 0x1F;
  if (REG_T_RISE[pin] != 0xFF)
    this->write_byte(REG_T_RISE[pin], t_rise);
  if (REG_T_FALL[pin] != 0xFF)
    this->write_byte(REG_T_FALL[pin], t_fall);
}

void SX1509Component::clock(byte osc_source, byte osc_pin_function, byte osc_freq_out, byte osc_divider) {
  osc_source = (osc_source & 0b11) << 5;           // 2-bit value, bits 6:5
  osc_pin_function = (osc_pin_function & 1) << 4;  // 1-bit value bit 4
  osc_freq_out = (osc_freq_out & 0b1111);          // 4-bit value, bits 3:0
  byte reg_clock = osc_source | osc_pin_function | osc_freq_out;
  this->write_byte(REG_CLOCK, reg_clock);

  osc_divider = constrain(osc_divider, 1, 7);
  clk_x_ = 2000000.0 / (1 << (osc_divider - 1));  // Update private clock variable
  osc_divider = (osc_divider & 0b111) << 4;       // 3-bit value, bits 6:4

  uint8_t reg_misc;
  this->read_byte(REG_MISC, &reg_misc);
  reg_misc &= ~(0b111 << 4);
  reg_misc |= osc_divider;
  this->write_byte(REG_MISC, reg_misc);
}

void SX1509Component::set_pin_value_(uint8_t pin, uint8_t i_on) {
  ESP_LOGD(TAG, "set_pin_value_ for pin %d to %d", pin, i_on);
  this->write_byte(REG_I_ON[pin], i_on);
}

void SX1509Component::setup_keypad(uint8_t rows, uint8_t columns, uint16_t sleep_time, uint8_t scan_time,
                                   uint8_t debounce_time) {
  uint16_t temp_word;
  uint8_t temp_byte;

  if (clk_x_ == 0)
    clock(INTERNAL_CLOCK_2MHZ);
  this->read_byte_16(REG_DIR_B, &temp_word);
  for (int i = 0; i < rows; i++)
    temp_word &= ~(1 << i);
  for (int i = 8; i < (columns * 2); i++)
    temp_word |= (1 << i);
  this->write_byte_16(REG_DIR_B, temp_word);
  this->read_byte(REG_OPEN_DRAIN_A, &temp_byte);
  for (int i = 0; i < rows; i++)
    temp_byte |= (1 << i);
  this->write_byte(REG_OPEN_DRAIN_A, temp_byte);
  this->read_byte(REG_PULL_UP_B, &temp_byte);
  for (int i = 0; i < columns; i++)
    temp_byte |= (1 << i);
  this->write_byte(REG_PULL_UP_B, temp_byte);
  debounce_time = constrain(debounce_time, 1, 64);
  scan_time = constrain(scan_time, 1, 128);
  if (debounce_time >= scan_time) {
    debounce_time = scan_time >> 1;  // Force debounce_time to be less than scan_time
  }
  debounce_keypad(debounce_time, rows, columns);
  uint8_t scan_time_bits = 0;
  for (uint8_t i = 7; i > 0; i--) {
    if (scan_time & (1 << i)) {
      scan_time_bits = i;
      break;
    }
  }
  uint8_t sleep_time_bits = 0;
  if (sleep_time != 0) {
    for (uint8_t i = 7; i > 0; i--) {
      if (sleep_time & ((unsigned int) 1 << (i + 6))) {
        sleep_time_bits = i;
        break;
      }
    }
    if (sleep_time_bits == 0)
      sleep_time_bits = 1;
  }
  sleep_time_bits = (sleep_time_bits & 0b111) << 4;
  scan_time_bits &= 0b111;  // Scan time is bits 2:0
  temp_byte = sleep_time | scan_time_bits;
  this->write_byte(REG_KEY_CONFIG_1, temp_byte);
  rows = (rows - 1) & 0b111;        // 0 = off, 0b001 = 2 rows, 0b111 = 8 rows, etc.
  columns = (columns - 1) & 0b111;  // 0b000 = 1 column, ob111 = 8 columns, etc.
  this->write_byte(REG_KEY_CONFIG_2, (rows << 3) | columns);
}

uint16_t SX1509Component::read_key_data() {
  uint16_t key_data;
  this->read_byte_16(REG_KEY_DATA_1, &key_data);
  return (0xFFFF ^ key_data);
}

void SX1509Component::debounce_config(uint8_t config_value) {
  // First make sure clock is configured
  uint8_t temp_byte;
  this->read_byte(REG_MISC, &temp_byte);
  if ((temp_byte & 0x70) == 0) {
    temp_byte |= (1 << 4);  // Just default to no divider if not set
    this->write_byte(REG_MISC, temp_byte);
  }
  this->read_byte(REG_CLOCK, &temp_byte);
  if ((temp_byte & 0x60) == 0) {
    temp_byte |= (1 << 6);  // default to internal osc.
    this->write_byte(REG_CLOCK, temp_byte);
  }

  config_value &= 0b111;  // 3-bit value
  this->write_byte(REG_DEBOUNCE_CONFIG, config_value);
}

void SX1509Component::debounce_time(uint8_t time) {
  if (clk_x_ == 0)                  // If clock hasn't been set up.
    clock(INTERNAL_CLOCK_2MHZ, 1);  // Set clock to 2MHz.

  uint8_t config_value = 0;

  for (int i = 7; i >= 0; i--) {
    if (time & (1 << i)) {
      config_value = i + 1;
      break;
    }
  }
  config_value = constrain(config_value, 0, 7);

  debounce_config(config_value);
}

void SX1509Component::debounce_enable(uint8_t pin) {
  uint16_t debounce_enable;
  this->read_byte_16(REG_DEBOUNCE_ENABLE_B, &debounce_enable);
  debounce_enable |= (1 << pin);
  this->write_byte_16(REG_DEBOUNCE_ENABLE_B, debounce_enable);
}

void SX1509Component::debounce_pin(uint8_t pin) { debounce_enable(pin); }

void SX1509Component::debounce_keypad(uint8_t time, uint8_t num_rows, uint8_t num_cols) {
  debounce_time(time);
  for (uint16_t i = 0; i < num_rows; i++)
    debounce_pin(i);
  for (uint16_t i = 0; i < (8 + num_cols); i++)
    debounce_pin(i);
}

}  // namespace sx1509
}  // namespace esphome
