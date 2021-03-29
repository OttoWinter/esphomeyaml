#pragma once
#include "buffer_base.h"

namespace esphome {
namespace display {

#ifdef USE_BUFFER_RGB332
class Buffer332 : public display::BufferBase {
 public:
  uint8_t *buffer_{nullptr};

  bool init_buffer(int width, int height) override;
  bool HOT set_buffer(int x, int y, Color color) override;
  void HOT fill_buffer(Color color) override;
  uint16_t HOT get_pixel_to_565(uint32_t pos) override;
  uint32_t HOT get_pixel_to_666(uint32_t pos) override;
  size_t HOT get_buffer_size() override;
  display::BufferType get_buffer_type() override { return this->buffer_type_; }
  uint8_t get_pixel_storage_size() override { return this->pixel_storage_size_; }

 protected:
  display::BufferType buffer_type_ = display::BufferType::BUFFER_TYPE_332;
  uint8_t pixel_storage_size_ = 8;
};  // class Buffer332
#endif
}  // namespace display
}  // namespace esphome
