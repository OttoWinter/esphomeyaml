#include <utility>

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/esphal.h"
#include "esphome/core/automation.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#ifdef ARDUINO_ARCH_ESP32
#include <driver/rmt.h>
#endif

namespace esphome {
namespace remote_base {

class RemoteTransmitData {
 public:
  void mark(uint32_t length) { this->data_.push_back(length); }

  void space(uint32_t length) { this->data_.push_back(-length); }

  void item(uint32_t mark, uint32_t space) {
    this->mark(mark);
    this->space(space);
  }

  void reserve(uint32_t len) { this->data_.reserve(len); }

  void set_carrier_frequency(uint32_t carrier_frequency) { this->carrier_frequency_ = carrier_frequency; }

  uint32_t get_carrier_frequency() const { return this->carrier_frequency_; }

  const std::vector<int32_t> &get_data() const { return this->data_; }

  void set_data(std::vector<int32_t> data) {
    this->data_.clear();
    this->data_.reserve(data.size());
    for (auto dat : data)
      this->data_.push_back(dat);
  }

  void reset() {
    this->data_.clear();
    this->carrier_frequency_ = 0;
  }

  std::vector<int32_t>::iterator begin() { return this->data_.begin(); }

  std::vector<int32_t>::iterator end() { return this->data_.end(); }

 protected:
  std::vector<int32_t> data_{};
  uint32_t carrier_frequency_{0};
};

class RemoteReceiveData {
 public:
  RemoteReceiveData(std::vector<int32_t> *data, uint8_t tolerance) : data_(data), tolerance_(tolerance) {}

  bool peek_mark(uint32_t length, uint32_t offset = 0) {
    if (int32_t(this->index_ + offset) >= this->size())
      return false;
    int32_t value = this->peek(offset);
    const int32_t lo = this->lower_bound_(length);
    const int32_t hi = this->upper_bound_(length);
    return value >= 0 && lo <= value && value <= hi;
  }

  bool peek_space(uint32_t length, uint32_t offset = 0) {
    if (int32_t(this->index_ + offset) >= this->size())
      return false;
    int32_t value = this->peek(offset);
    const int32_t lo = this->lower_bound_(length);
    const int32_t hi = this->upper_bound_(length);
    return value <= 0 && lo <= -value && -value <= hi;
  }

  bool peek_space_at_least(uint32_t length, uint32_t offset = 0) {
    if (int32_t(this->index_ + offset) >= this->size())
      return false;
    int32_t value = this->pos(this->index_ + offset);
    const int32_t lo = this->lower_bound_(length);
    return value <= 0 && lo <= -value;
  }

  bool peek_item(uint32_t mark, uint32_t space, uint32_t offset = 0) {
    return this->peek_mark(mark, offset) && this->peek_space(space, offset + 1);
  }

  int32_t peek(uint32_t offset = 0) { return (*this)[this->index_ + offset]; }

  void advance(uint32_t amount = 1) { this->index_ += amount; }

  bool expect_mark(uint32_t length) {
    return ;
  }

  bool expect_space(uint32_t length) {
    return ;
  }

  bool expect_item(uint32_t mark, uint32_t space) {
    return ;
  }

  void reset() { this->index_ = 0; }

  int32_t pos(uint32_t index) const { return (*this->data_)[index]; }

  int32_t operator[](uint32_t index) const { return this->pos(index); }

  int32_t size() const { return this->data_->size(); }

  std::vector<int32_t> *get_raw_data() { return this->data_; }

 protected:
  int32_t lower_bound_(uint32_t length) { return int32_t(100 - this->tolerance_) * length / 100U; }
  int32_t upper_bound_(uint32_t length) { return int32_t(100 + this->tolerance_) * length / 100U; }

  uint32_t index_{0};
  std::vector<int32_t> *data_;
  uint8_t tolerance_;
};

template<typename T> class RemoteProtocol {
 public:
  virtual void encode(RemoteTransmitData *dst, const T &data) = 0;

  virtual optional<T> decode(RemoteReceiveData src) = 0;

  virtual void dump(const T &data) = 0;
};

class RemoteComponentBase {
 public:
  explicit RemoteComponentBase(GPIOPin *pin);

#ifdef ARDUINO_ARCH_ESP32
  void set_channel(rmt_channel_t channel) { this->channel_ = channel; }
  void set_clock_divider(uint8_t clock_divider) { this->clock_divider_ = clock_divider; }
#endif

 protected:
#ifdef ARDUINO_ARCH_ESP32
  uint32_t from_microseconds(uint32_t us) {
    const uint32_t ticks_per_ten_us = 80000000u / this->clock_divider_ / 100000u;
    return us * ticks_per_ten_us / 10;
  }
  uint32_t to_microseconds(uint32_t ticks) {
    const uint32_t ticks_per_ten_us = 80000000u / this->clock_divider_ / 100000u;
    return (ticks * 10) / ticks_per_ten_us;
  }
#endif

  GPIOPin *pin_;
#ifdef ARDUINO_ARCH_ESP32
  rmt_channel_t channel_{RMT_CHANNEL_0};
  uint8_t clock_divider_{80};
  esp_err_t error_code_{ESP_OK};
#endif
};

class RemoteTransmitterBase : public RemoteComponentBase {
 public:
  RemoteTransmitterBase(GPIOPin *pin) : RemoteComponentBase(pin) {}
  class TransmitCall {
   public:
    explicit TransmitCall(RemoteTransmitterBase *parent) : parent_(parent) {}
    RemoteTransmitData *get_data() { return &this->parent_->temp_; }
    void set_send_times(uint32_t send_times) { send_times_ = send_times; }
    void set_send_wait(uint32_t send_wait) { send_wait_ = send_wait; }

    void perform() { this->parent_->send_(this->send_times_, this->send_wait_); }

   protected:
    RemoteTransmitterBase *parent_;
    uint32_t send_times_{1};
    uint32_t send_wait_{0};
  };

  TransmitCall transmit() {
    this->temp_.reset();
    return TransmitCall(this);
  }

 protected:
  void send_(uint32_t send_times, uint32_t send_wait);
  virtual void send_internal(uint32_t send_times, uint32_t send_wait) = 0;
  void send_single_() { this->send_(1, 0); }

  /// Use same vector for all transmits, avoids many allocations
  RemoteTransmitData temp_;
};

class RemoteReceiverListener {
 public:
  virtual bool on_receive(RemoteReceiveData data) = 0;
};

class RemoteReceiverDumperBase {
 public:
  virtual bool dump(RemoteReceiveData src) = 0;
  virtual bool is_secondary() { return false; }
};

class RemoteReceiverBase : public RemoteComponentBase {
 public:
  RemoteReceiverBase(GPIOPin *pin) : RemoteComponentBase(pin) {}
  void register_listener(RemoteReceiverListener *listener) { this->listeners_.push_back(listener); }
  void register_dumper(RemoteReceiverDumperBase *dumper) {
    if (dumper->is_secondary()) {
      this->secondary_dumpers_.push_back(dumper);
    } else {
      this->dumpers_.push_back(dumper);
    }
  }
  void set_tolerance(uint8_t tolerance) { tolerance_ = tolerance; }

 protected:
  bool call_listeners_() {
    bool success = false;
    for (auto *listener : this->listeners_) {
      auto data = RemoteReceiveData(&this->temp_, this->tolerance_);
      if (listener->on_receive(data))
        success = true;
    }
    return success;
  }
  void call_dumpers_() {
    bool success = false;
    for (auto *dumper : this->dumpers_) {
      auto data = RemoteReceiveData(&this->temp_, this->tolerance_);
      if (dumper->dump(data))
        success = true;
    }
    if (!success) {
      for (auto *dumper : this->secondary_dumpers_) {
        auto data = RemoteReceiveData(&this->temp_, this->tolerance_);
        dumper->dump(data);
      }
    }
  }
  void call_listeners_dumpers_() {
    if (this->call_listeners_())
      return;
    // If a listener handled, then do not dump
    this->call_dumpers_();
  }

  std::vector<RemoteReceiverListener *> listeners_;
  std::vector<RemoteReceiverDumperBase *> dumpers_;
  std::vector<RemoteReceiverDumperBase *> secondary_dumpers_;
  std::vector<int32_t> temp_;
  uint8_t tolerance_{25};
};

class RemoteReceiverBinarySensorBase : public binary_sensor::BinarySensor,
                                       public Component,
                                       public RemoteReceiverListener {
 public:
  explicit RemoteReceiverBinarySensorBase() : BinarySensor() {}
  void dump_config() override;
  virtual bool matches(RemoteReceiveData src) = 0;
  bool on_receive(RemoteReceiveData src) override {
    return this->matches(src);
  }
};

template<typename T, typename D> class RemoteReceiverBinarySensor : public RemoteReceiverBinarySensorBase {
 public:
  RemoteReceiverBinarySensor() : RemoteReceiverBinarySensorBase() {}

 protected:
  bool matches(RemoteReceiveData src) override {
    auto proto = T();
    auto res = proto.decode(src);
    return res.has_value() && *res == this->data_;
  }

 public:
  void set_data(D data) { data_ = data; }

 protected:
  D data_;
};

template<typename T, typename D> class RemoteReceiverTrigger : public Trigger<D>, public RemoteReceiverListener {
 protected:
  bool on_receive(RemoteReceiveData src) override {
    auto proto = T();
    auto res = proto.decode(src);
    if (res.has_value()) {
      this->trigger(*res);
      return true;
    }
    return false;
  }
};

template<typename... Ts> class RemoteTransmitterActionBase : public Action<Ts...> {
 public:
  void set_parent(RemoteTransmitterBase *parent) { this->parent_ = parent; }

  void play(Ts... x) override {
    auto call = this->parent_->transmit();
    this->encode(call.get_data(), x...);
    call.set_send_times(this->send_times_.value_or(x..., 1));
    call.set_send_wait(this->send_wait_.value_or(x..., 0));
    call.perform();
  }

  virtual void encode(RemoteTransmitData *dst, Ts... x) = 0;

  templatable_value(uint32_t, send_times);
  templatable_value(uint32_t, send_wait);

 protected:
  RemoteTransmitterBase *parent_{};
};

template<typename T, typename D> class RemoteReceiverDumper : public RemoteReceiverDumperBase {
 public:
  bool dump(RemoteReceiveData src) override {
    auto proto = T();
    auto decoded = proto.decode(src);
    if (!decoded.has_value())
      return false;
    proto.dump(*decoded);
    return true;
  }
};

#define DECLARE_REMOTE_PROTOCOL_(prefix) \
  using prefix##BinarySensor = RemoteReceiverBinarySensor<prefix##Protocol, prefix##Data>; \
  using prefix##Trigger = RemoteReceiverTrigger<prefix##Protocol, prefix##Data>; \
  using prefix##Dumper = RemoteReceiverDumper<prefix##Protocol, prefix##Data>;
#define DECLARE_REMOTE_PROTOCOL(prefix) DECLARE_REMOTE_PROTOCOL_(prefix)

}  // namespace remote_base
}  // namespace esphome
