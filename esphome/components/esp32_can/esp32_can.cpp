#include "esp32_can.h"
#include "esphome/core/log.h"

namespace esphome {
namespace esp32_can {

static const char *TAG = "esp32_can";

bool ESP32Can::send_internal_(int can_id, uint8_t *data) { return true; };

bool ESP32Can::setup_internal_ () { return true; };

canbus::Error ESP32Can::set_bitrate_(const canbus::CanSpeed canSpeed) { return canbus::ERROR_OK; };

}  // namespace esp32_can
}  // namespace esphome
