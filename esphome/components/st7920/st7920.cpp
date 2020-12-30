#include "st7920.h"
#include "esphome/core/log.h"
#include "esphome/components/display/display_buffer.h"

namespace esphome {
namespace st7920 {

static const char *TAG = "st7920";

// ST7920 COMMANDS
static const uint8_t LCD_DATA 0xFA       // Data bit
static const uint8_t LCD_COMMAND 0xF8       // Command bit
static const uint8_t LCD_CLS 0x01
static const uint8_t LCD_HOME 0x02
static const uint8_t LCD_ADDRINC 0x06
static const uint8_t LCD_DISPLAYON 0x0C
static const uint8_t LCD_DISPLAYOFF 0x08
static const uint8_t LCD_CURSORON 0x0E
static const uint8_t LCD_CURSORBLINK 0x0F
static const uint8_t LCD_BASIC 0x30
static const uint8_t LCD_GFXMODE 0x36
static const uint8_t LCD_EXTEND 0x34
static const uint8_t LCD_TXTMODE 0x34
static const uint8_t LCD_STANDBY 0x01
static const uint8_t LCD_SCROLL 0x03
static const uint8_t LCD_SCROLLADDR 0x40
static const uint8_t LCD_ADDR 0x80
static const uint8_t LCD_LINE0 0x80
static const uint8_t LCD_LINE1 0x90
static const uint8_t LCD_LINE2 0x88
static const uint8_t LCD_LINE3 0x98

void ST7920::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ST7920...");
  this->dump_config();
  this->spi_setup();
  this->rs_pin_->setup();
  this->rs_pin_->digital_write(LOW);
  this->init_internal_(this->get_buffer_length_());
  display_init_();
}

void ST7920::command_(uint8_t value) {
  this->start_transaction_();
  send_(LCD_COMMAND, value);
  this->end_transaction_();
}

void ST7920::data_(uint8_t value) {
  this->start_transaction_();
  send_(LCD_DATA, value);
  this->end_transaction_();
}

void ST7920::send_(uint8_t type, uint8_t value) {
  this->write_byte(type);
  this->write_byte(value & 0xF0);
  this->write_byte(value << 4);
}

void ST7920::goto_xy_(uint16_t x, uint16_t y) {
  if (y >= 32 && y < 64) {
    y -= 32;
    x += 8;
  } else if (y >= 64 && y < 64 + 32) {
    y -= 32;
    x += 0;
  } else if (y >= 64 + 32 && y < 64 + 64) {
    y -= 64;
    x += 8;
  }
  command_(LCD_ADDR | y);  // 6-bit (0..63)
  command_(LCD_ADDR | x);  // 4-bit (0..15)
}

void HOT ST7920::write_display_data() {
  byte i, j, b;
  for (j = 0; j < this->get_height_internal() / 2; j++) {
    this->goto_xy_(0, j);
    this->start_transaction_();
    for (i = 0; i < 16; i++) {  // 16 bytes from line #0+
      b=this->buffer_[i + j * 16];
      send_(LCD_DATA, b);
    }
    for (i = 0; i < 16; i++) {  // 16 bytes from line #32+
      b=this->buffer_[i + (j + 32) * 16];
      send_(LCD_DATA, b);
    }
    this->end_transaction_();
    App.feed_wdt();
  }
}


void ST7920::fill(Color color) { memset(this->buffer_, 0, this->get_buffer_length_()); }

void ST7920::dump_config() {
  LOG_DISPLAY("", "ST7920", this);
  LOG_PIN("  RS Pin: ", this->rs_pin_);
  ESP_LOGD(TAG, "  Height: %d", this->height_);
  ESP_LOGD(TAG, "  Width: %d", this->width_);
}

float ST7920::get_setup_priority() const {
  return setup_priority::PROCESSOR;
}

void ST7920::update() {
  this->fill(COLOR_OFF);
  if (this->writer_local_.has_value())  // insert Labda function if available
    (*this->writer_local_)(*this);
  this->write_display_data();
}

void ST7920::loop() {}

int ST7920::get_width_internal() { return width_; }

int ST7920::get_height_internal() { return height_; }

size_t ST7920::get_buffer_length_() {
  return size_t(this->get_width_internal()) * size_t(this->get_height_internal()) / 8u;
}

void HOT ST7920::draw_absolute_pixel_internal(int x, int y, Color color) {
  if (x >= this->get_width_internal() || x < 0 || y >= this->get_height_internal() || y < 0) {
    ESP_LOGW(TAG, "Posiotion out of area: %dx%d", x, y);
    return;
  }
  int width = this->get_width_internal() / 8u;
  if (color.is_on()) {
    this->buffer_[y * width + x / 8] |= (0x80 >> (x & 7));
  } else {
    this->buffer_[y * width + x / 8] &= ~(0x80 >> (x & 7));
  }
}

void ST7920::display_init_() {
  ESP_LOGCONFIG(TAG, "displayInit...");
  command_(LCD_BASIC);  // 8bit mode
  command_(LCD_BASIC);  // 8bit mode
  command_(LCD_CLS);  // clear screen
  delay(12);  // >10 ms delay
  command_(LCD_ADDRINC);  // cursor increment right no shift
  command_(LCD_DISPLAYON);  // D=1, C=0, B=0
  command_(LCD_EXTEND); //LCD_EXTEND);
  command_(LCD_GFXMODE); //LCD_GFXMODE);
  write_display_data();
}

void ST7920::start_transaction_() {
  this->enable();
  this->rs_pin_->digital_write(HIGH);
}

void ST7920::end_transaction_() {
  this->disable();
  this->rs_pin_->digital_write(LOW);
}

}  // namespace st7920
}  // namespace esphome