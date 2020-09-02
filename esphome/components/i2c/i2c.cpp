#include "i2c.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/application.h"

namespace esphome {
namespace i2c {

static const char *TAG = "i2c";

I2CComponent::I2CComponent() {
#ifdef ARDUINO_ARCH_ESP32
  if (next_i2c_bus_num_ == 0)
    this->wire_ = &Wire;
  else
    this->wire_ = new TwoWire(next_i2c_bus_num_);
  next_i2c_bus_num_++;
#else
  this->wire_ = &Wire;
#endif
}

void I2CComponent::setup() {
  this->wire_->begin(this->sda_pin_, this->scl_pin_);
  this->wire_->setClock(this->frequency_);
}
void I2CComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "I2C Bus:");
  ESP_LOGCONFIG(TAG, "  SDA Pin: GPIO%u", this->sda_pin_);
  ESP_LOGCONFIG(TAG, "  SCL Pin: GPIO%u", this->scl_pin_);
  ESP_LOGCONFIG(TAG, "  Frequency: %u Hz", this->frequency_);
  if (this->scan_) {
    ESP_LOGI(TAG, "Scanning i2c bus for active devices...");
    uint8_t found = 0;
    for (uint8_t address = 1; address < 120; address++) {
      this->wire_->beginTransmission(address);
      uint8_t error = this->wire_->endTransmission();

      if (error == 0) {
        ESP_LOGI(TAG, "Found i2c device at address 0x%02X", address);
        found++;
      } else if (error == 4) {
        ESP_LOGI(TAG, "Unknown error at address 0x%02X", address);
      }

      delay(1);
    }
    if (found == 0) {
      ESP_LOGI(TAG, "Found no i2c devices!");
    }
  }
}
float I2CComponent::get_setup_priority() const { return setup_priority::BUS; }

void I2CComponent::raw_begin_transmission(uint8_t address) {
  ESP_LOGVV(TAG, "Beginning Transmission to 0x%02X:", address);
  this->wire_->beginTransmission(address);
}
bool I2CComponent::raw_end_transmission(uint8_t address) {
  uint8_t status = this->wire_->endTransmission();
  ESP_LOGVV(TAG, "    Transmission ended. Status code: 0x%02X", status);

  switch (status) {
    case 0:
      break;
    case 1:
      ESP_LOGW(TAG, "Too much data to fit in transmitter buffer for address 0x%02X", address);
      break;
    case 2:
      ESP_LOGW(TAG, "Received NACK on transmit of address 0x%02X", address);
      break;
    case 3:
      ESP_LOGW(TAG, "Received NACK on transmit of data for address 0x%02X", address);
      break;
    default:
      ESP_LOGW(TAG, "Unknown transmit error %u for address 0x%02X", status, address);
      break;
  }

  return status == 0;
}
bool I2CComponent::raw_request_from(uint8_t address, uint8_t len) {
  ESP_LOGVV(TAG, "Requesting %u bytes from 0x%02X:", len, address);
  uint8_t ret = this->wire_->requestFrom(address, len);
  if (ret != len) {
    ESP_LOGW(TAG, "Requesting %u bytes from 0x%02X failed!", len, address);
    return false;
  }
  return true;
}
void HOT I2CComponent::raw_write(uint8_t address, const uint8_t *data, uint8_t len) {
  for (size_t i = 0; i < len; i++) {
    ESP_LOGVV(TAG, "    Writing 0b" BYTE_TO_BINARY_PATTERN " (0x%02X)", BYTE_TO_BINARY(data[i]), data[i]);
    this->wire_->write(data[i]);
    App.feed_wdt();
  }
}
void HOT I2CComponent::raw_write_16(uint8_t address, const uint16_t *data, uint8_t len) {
  for (size_t i = 0; i < len; i++) {
    ESP_LOGVV(TAG, "    Writing 0b" BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN " (0x%04X)",
              BYTE_TO_BINARY(data[i] >> 8), BYTE_TO_BINARY(data[i]), data[i]);
    this->wire_->write(data[i] >> 8);
    this->wire_->write(data[i]);
    App.feed_wdt();
  }
}

bool I2CComponent::raw_receive(uint8_t address, uint8_t *data, uint8_t len) {
  if (!this->raw_request_from(address, len))
    return false;
  for (uint8_t i = 0; i < len; i++) {
    data[i] = this->wire_->read();
    ESP_LOGVV(TAG, "    Received 0b" BYTE_TO_BINARY_PATTERN " (0x%02X)", BYTE_TO_BINARY(data[i]), data[i]);
    App.feed_wdt();
  }
  return true;
}
bool I2CComponent::raw_receive_16(uint8_t address, uint16_t *data, uint8_t len) {
  if (!this->raw_request_from(address, len * 2))
    return false;
  auto *data_8 = reinterpret_cast<uint8_t *>(data);
  for (uint8_t i = 0; i < len; i++) {
    data_8[i * 2 + 1] = this->wire_->read();
    data_8[i * 2] = this->wire_->read();
    ESP_LOGVV(TAG, "    Received 0b" BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN " (0x%04X)",
              BYTE_TO_BINARY(data_8[i * 2 + 1]), BYTE_TO_BINARY(data_8[i * 2]), data[i]);
  }
  return true;
}
bool I2CComponent::read_bytes(uint8_t address, uint16_t a_register, uint8_t *data, uint8_t len, uint32_t conversion, uint8_t a_register_width) {
  if (!this->write_bytes(address, a_register, nullptr, 0, a_register_width))
    return false;
  if (conversion > 0)
    delay(conversion);
  return this->raw_receive(address, data, len);
}
bool I2CComponent::read_bytes_raw(uint8_t address, uint8_t *data, uint8_t len) {
  return this->raw_receive(address, data, len);
}
bool I2CComponent::read_bytes_16(uint8_t address, uint16_t a_register, uint16_t *data, uint8_t len,
                                 uint32_t conversion, uint8_t a_register_width) {
  if (!this->write_bytes(address, a_register, nullptr, 0))
    return false;

  if (conversion > 0)
    delay(conversion);
  return this->raw_receive_16(address, data, len);
}
bool I2CComponent::read_byte(uint8_t address, uint16_t a_register, uint8_t *data, uint32_t conversion, uint8_t a_register_width) {
  return this->read_bytes(address, a_register, data, 1, conversion, a_register_width);
}

bool I2CComponent::read_byte_16(uint8_t address, uint16_t a_register, uint16_t *data, uint32_t conversion, uint8_t a_register_width) {
  return this->read_bytes_16(address, a_register, data, 1, conversion, a_register_width);
}
bool I2CComponent::write_bytes(uint8_t address, uint16_t a_register, const uint8_t *data, uint8_t len, uint8_t a_register_width) {
  this->raw_begin_transmission(address);
  (a_register_width == 8) ? this->raw_write(address, (uint8_t*)&a_register, 1) : this->raw_write_16(address, &a_register, 1);
  this->raw_write(address, data, len);
  return this->raw_end_transmission(address);
}
bool I2CComponent::write_bytes_raw(uint8_t address, const uint8_t *data, uint8_t len) {
  this->raw_begin_transmission(address);
  this->raw_write(address, data, len);
  return this->raw_end_transmission(address);
}
bool I2CComponent::write_bytes_16(uint8_t address, uint16_t a_register, const uint16_t *data, uint8_t len, uint8_t a_register_width) {
  this->raw_begin_transmission(address);
  (a_register_width == 8) ? this->raw_write(address, (uint8_t*)&a_register, 1) : this->raw_write_16(address, &a_register, 1);
  this->raw_write_16(address, data, len);
  return this->raw_end_transmission(address);
}
bool I2CComponent::write_byte(uint8_t address, uint16_t a_register, uint8_t data, uint8_t a_register_width) {
  return this->write_bytes(address, a_register, &data, 1, a_register_width);
}
bool I2CComponent::write_byte_16(uint8_t address, uint16_t a_register, uint16_t data, uint8_t a_register_width) {
  return this->write_bytes_16(address, a_register, &data, 1, a_register_width);
}

void I2CDevice::set_i2c_address(uint8_t address) { this->address_ = address; }
bool I2CDevice::read_bytes(uint16_t a_register, uint8_t *data, uint8_t len, uint32_t conversion, uint8_t a_register_width) {  // NOLINT
  return this->parent_->read_bytes(this->address_, a_register, data, len, conversion, a_register_width);
}
bool I2CDevice::read_byte(uint16_t a_register, uint8_t *data, uint32_t conversion, uint8_t a_register_width) {  // NOLINT
  return this->parent_->read_byte(this->address_, a_register, data, conversion, a_register_width);
}
bool I2CDevice::write_bytes(uint16_t a_register, const uint8_t *data, uint8_t len, uint8_t a_register_width) {  // NOLINT
  return this->parent_->write_bytes(this->address_, a_register, data, len, a_register_width);
}
bool I2CDevice::write_byte(uint16_t a_register, uint8_t data, uint8_t a_register_width) {  // NOLINT
  return this->parent_->write_byte(this->address_, a_register, data, a_register_width);
}
bool I2CDevice::read_bytes_16(uint16_t a_register, uint16_t *data, uint8_t len, uint32_t conversion, uint8_t a_register_width) {  // NOLINT
  return this->parent_->read_bytes_16(this->address_, a_register, data, len, conversion, a_register_width);
}
bool I2CDevice::read_byte_16(uint16_t a_register, uint16_t *data, uint32_t conversion, uint8_t a_register_width) {  // NOLINT
  return this->parent_->read_byte_16(this->address_, a_register, data, conversion, a_register_width);
}
bool I2CDevice::write_bytes_16(uint16_t a_register, const uint16_t *data, uint8_t len, uint8_t a_register_width) {  // NOLINT
  return this->parent_->write_bytes_16(this->address_, a_register, data, len, a_register_width);
}
bool I2CDevice::write_byte_16(uint16_t a_register, uint16_t data, uint8_t a_register_width) {  // NOLINT
  return this->parent_->write_byte_16(this->address_, a_register, data, a_register_width);
}
void I2CDevice::set_i2c_parent(I2CComponent *parent) { this->parent_ = parent; }

#ifdef ARDUINO_ARCH_ESP32
uint8_t next_i2c_bus_num_ = 0;
#endif

// As the internal representation of an I2C register is now always 16 bit,
// a check if the actual address is 8 bit or 16 bit is neccecary 
I2CRegister &I2CRegister::operator=(uint8_t value) {
  this->parent_->write_byte(this->register_, value, this->register_width_);
  return *this;
}

I2CRegister &I2CRegister::operator&=(uint8_t value) {
  this->parent_->write_byte(this->register_, this->get() & value, this->register_width_);
  return *this;
}

I2CRegister &I2CRegister::operator|=(uint8_t value) {
  this->parent_->write_byte(this->register_, this->get() | value, this->register_width_);
  return *this;
}

uint8_t I2CRegister::get() {
  uint8_t value = 0x00;
  this->parent_->read_byte(this->register_, &value, this->register_width_);
  return value;
}

I2CRegister &I2CRegister::operator=(const std::vector<uint8_t> &value) {
  this->parent_->write_bytes(this->register_, value, this->register_width_);
  return *this;
}


uint16_t switch_lsb_msb(uint16_t value) {
  return (value<<8) | (value>>(8));
}

}  // namespace i2c
}  // namespace esphome
