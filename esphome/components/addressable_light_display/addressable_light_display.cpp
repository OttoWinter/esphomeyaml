#include "addressable_light_display.h"
#include "esphome/core/log.h"

namespace esphome {
namespace addressable_light_display {

static const char* TAG = "display.addressable_light_display";

int AddressableLightDisplay::get_width_internal() { return this->width_; }
int AddressableLightDisplay::get_height_internal() { return this->height_; }

void AddressableLightDisplay::setup() { this->addressable_light_buffer_.resize(this->width_ * this->height_, {0, 0, 0, 0}); };

void AddressableLightDisplay::update() {
  if (!this->enabled_)
    return;

  this->do_update_();
  this->display();
};

void AddressableLightDisplay::display() {
  bool dirty = false;
  uint8_t old_r, old_g, old_b, old_w;
  Color* c;

  for (uint32_t offset = 0; offset < this->addressable_light_buffer_.size(); offset++) {
    c = &(this->addressable_light_buffer_[offset]);

    light::ESPColorView pixel = (*this->light_)[offset];

    // Track the original values for the pixel view. If it has changed updating, then
    // we trigger a redraw. Avoiding redraws == avoiding flicker!
    old_r = pixel.get_red();
    old_g = pixel.get_green();
    old_b = pixel.get_blue();
    old_w = pixel.get_white();

    pixel.set_rgbw(c->r, c->g, c->b, c->w);

    // If the actual value of the pixel changed, then schedule a redraw.
    if (pixel.get_red() != old_r || pixel.get_green() != old_g || pixel.get_blue() != old_b ||
        pixel.get_white() != old_w) {
      dirty = true;
    }
  }

  if (dirty == true) {
    this->light_->schedule_show();
  }
};

void HOT AddressableLightDisplay::draw_absolute_pixel_internal(int x, int y, Color color) {
  if (x >= this->get_width_internal() || x < 0 || y >= this->get_height_internal() || y < 0)
    return;

  this->addressable_light_buffer_[y * this->get_width_internal() + x] = color;
}
}  // namespace addressable_light_display
}  // namespace esphome
