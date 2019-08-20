#pragma once

#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/wifi/wifi_component.h"

namespace esphome {
namespace wifi_info {

static const char TAG[] = "wifi_info.text_sensor";

class IPAddressWiFiInfo : public Component, public text_sensor::TextSensor {
 public:
  void loop() override {
    IPAddress ip = WiFi.localIP();
    if (ip != this->last_ip_) {
      this->last_ip_ = ip;
      this->publish_state(ip.toString().c_str());
    }
  }
  void dump_config() override { LOG_TEXT_SENSOR("", "WifiInfo IPAddress Text Sensor", this); }
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }
  std::string unique_id() override { return get_mac_address() + "-wifiinfo-ip"; }

 protected:
  IPAddress last_ip_;
};

class SSIDWiFiInfo : public Component, public text_sensor::TextSensor {
 public:
  void loop() override {
    String ssid = WiFi.SSID();
    if (this->last_ssid_ != ssid.c_str()) {
      this->last_ssid_ = std::string(ssid.c_str());
      this->publish_state(this->last_ssid_);
    }
  }
  void dump_config() override { LOG_TEXT_SENSOR("", "WifiInfo SSDID Text Sensor", this); }
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }
  std::string unique_id() override { return get_mac_address() + "-wifiinfo-ssid"; }

 protected:
  std::string last_ssid_;
};

class BSSIDWiFiInfo : public Component, public text_sensor::TextSensor {
 public:
  void loop() override {
    uint8_t *bssid = WiFi.BSSID();
    if (memcmp(bssid, this->last_bssid_.data(), 6) != 0) {
      std::copy(bssid, bssid + 6, this->last_bssid_.data());
      char buf[30];
      sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
      this->publish_state(buf);
    }
  }
  void dump_config() override { LOG_TEXT_SENSOR("", "WifiInfo BSSID Text Sensor", this); }
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }
  std::string unique_id() override { return get_mac_address() + "-wifiinfo-bssid"; }

 protected:
  wifi::bssid_t last_bssid_;
};

}  // namespace wifi_info
}  // namespace esphome
