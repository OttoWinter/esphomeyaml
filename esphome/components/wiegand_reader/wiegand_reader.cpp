#include "wiegand_reader.h"
#include "esphome/core/log.h"

namespace esphome {
namespace wiegand_reader {

static const char *TAG = "wiegand_reader";

void ICACHE_RAM_ATTR pin_state_changed_(WiegandReader *arg) {
  arg->wiegand_.setPin0State(arg->pin_d0_->digital_read());
  arg->wiegand_.setPin1State(arg->pin_d1_->digital_read());
}

static void receivedData(uint8_t* data, uint8_t bits, WiegandReader* device) {
    uint8_t byteCount = (bits+7)/8;

    String code = "";
    for (int i=0; i<byteCount; i++) {
      code = code + String(data[i], 16);
     }
     ESP_LOGD(TAG, "Data received : %s", code.c_str());

     for (auto *trigger : device->triggers_)
        trigger->process(code);
  }

void receivedDataError(Wiegand::DataError error, uint8_t* rawData, uint8_t rawBits, const char* message) {
    ESP_LOGE(TAG, "FAILED : %s", message);
    ESP_LOGE(TAG, "   ERROR : %s", Wiegand::DataErrorStr(error));

    uint8_t byteCount = (rawBits+7)/8;
    ESP_LOGE(TAG, "   BYTE COUNT : %i", byteCount);

    String code = "";
    for (int i=0; i<byteCount; i++) {
      code = code + String(rawData[i], 16);
    }
    ESP_LOGE(TAG, "   DECODED : %s", code.c_str());
}

void WiegandReader::setup() {
  this->pin_d0_->pin_mode(INPUT);
  this->pin_d1_->pin_mode(INPUT);
  this->wiegand_.onReceive(receivedData, this);
  this->wiegand_.onReceiveError(receivedDataError, "Card read error: ");

  this->dump_config();

  this->wiegand_.begin(0xFF, true);

  this->pin_d0_->attach_interrupt(pin_state_changed_, this,CHANGE);
  this->pin_d1_->attach_interrupt(pin_state_changed_, this,CHANGE);
  pin_state_changed_(this);

}

void WiegandReader::update() {
  noInterrupts();
  this->wiegand_.flush();
  interrupts();
}

void WiegandReader::set_data_pins(GPIOPin *pin_d0, GPIOPin *pin_d1){
    this->pin_d0_ = pin_d0;
    this->pin_d1_ = pin_d1;
}

float WiegandReader::get_setup_priority() const { return setup_priority::DATA; }

void WiegandReader::dump_config() {
  ESP_LOGCONFIG(TAG, "WiegandReader:");
  LOG_UPDATE_INTERVAL(this);
}

//************************ TRIGGER *******************************************
void WiegandReaderTrigger::process(String tag) {
  this->trigger(tag.c_str());
}

}  // namespace wiegand_reader
}  // namespace esphome
