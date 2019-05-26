#include "esphome/components/ct_clamp/ct_clamp_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ct_clamp {

static const char *TAG = "ct_clamp";

void CTClampSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CT Clamp '%s'...", this->get_name().c_str());
  GPIOPin(this->pin_, INPUT).setup();

  offsetI = ADC_COUNTS>>1;

}

void CTClampSensor::dump_config() {
  LOG_SENSOR("", "CT Clamp Sensor", this);
  ESP_LOGCONFIG(TAG, "  Pin: %u", this->pin_);
  ESP_LOGCONFIG(TAG, "  Calibration: %u", this->calibration_);
  ESP_LOGCONFIG(TAG, "  Sample Size: %u", this->sample_size_);
  ESP_LOGCONFIG(TAG, "  Supply Voltage: %0.2fV", this->supply_voltage_);
  LOG_UPDATE_INTERVAL(this);
}

float CTClampSensor::get_setup_priority() const { return setup_priority::DATA; }

void CTClampSensor::update() {

  for (unsigned int n = 0; n < this->sample_size_; n++)
  {
    sampleI = analogRead(this->pin_);

    // Digital low pass filter extracts the 1.65 V dc offset,
    //  then subtract this - signal is now centered on 0 counts.
    offsetI = (offsetI + (sampleI-offsetI)/(ADC_COUNTS));
    filteredI = sampleI - offsetI;

    // Root-mean-square method current
    // 1) square current values
    sqI = filteredI * filteredI;
    // 2) sum
    sumI += sqI;
  }

  double I_RATIO = this->calibration_ * ((supply_voltage_) / (ADC_COUNTS));
  Irms = I_RATIO * sqrt(sumI / this->sample_size_);

  //Reset accumulators
  sumI = 0;

  ESP_LOGD(TAG, "'%s'", this->get_name().c_str(), Irms);
  ESP_LOGD(TAG, "   Amps=%.2fA", Irms);
  ESP_LOGD(TAG, "   Watts=%.0fW", Irms * 230.0);

  this->publish_state(Irms);
}

#ifdef ARDUINO_ARCH_ESP8266
std::string CTClampSensor::unique_id() { return get_mac_address() + "-ct_clamp"; }
#endif

}  // namespace ct_clamp
}  // namespace esphome
