#include "bang_bang_climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bang_bang {

static const char *TAG = "bang_bang.climate";

void BangBangClimate::setup() {
  this->sensor_->add_on_state_callback([this](float state) {
    this->current_temperature = state;
    // control may have changed, recompute
    this->compute_state_();
    // current temperature changed, publish state
    this->publish_state();
  });
  this->current_temperature = this->sensor_->state;
  // restore set points
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->to_call(this).perform();
  } else {
    // restore from defaults, change_away handles those for us
    this->mode = climate::CLIMATE_MODE_AUTO;
    this->change_away_(false);
  }
}
void BangBangClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value())
    this->mode = *call.get_mode();
  if (call.get_target_temperature_low().has_value())
    this->target_temperature_low = *call.get_target_temperature_low();
  if (call.get_target_temperature_high().has_value())
    this->target_temperature_high = *call.get_target_temperature_high();
  if (call.get_away().has_value())
    this->change_away_(*call.get_away());
  if (call.get_boost().has_value())
    this->change_boost_(*call.get_boost());
  if (call.get_eco().has_value())
    this->change_eco_(*call.get_eco());
  if (call.get_night().has_value())
    this->change_night_(*call.get_night());
  this->compute_state_();
  this->publish_state();
}
climate::ClimateTraits BangBangClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_supports_current_temperature(true);
  traits.set_supports_auto_mode(true);
  traits.set_supports_cool_mode(this->supports_cool_);
  traits.set_supports_heat_mode(this->supports_heat_);
  traits.set_supports_two_point_target_temperature(true);
  traits.set_supports_away(this->supports_away_);
  traits.set_supports_boost(this->supports_boost_);
  traits.set_supports_night(this->supports_night_);
  traits.set_supports_eco(this->supports_eco_);
  traits.set_supports_action(true);
  return traits;
}
void BangBangClimate::compute_state_() {
  if (this->mode != climate::CLIMATE_MODE_AUTO) {
    // in non-auto mode, switch directly to appropriate action
    //  - HEAT mode -> HEATING action
    //  - COOL mode -> COOLING action
    //  - OFF mode -> OFF action (not IDLE!)
    this->switch_to_action_(static_cast<climate::ClimateAction>(this->mode));
    return;
  }
  if (isnan(this->current_temperature) || isnan(this->target_temperature_low) || isnan(this->target_temperature_high)) {
    // if any control parameters are nan, go to OFF action (not IDLE!)
    this->switch_to_action_(climate::CLIMATE_ACTION_OFF);
    return;
  }
  const bool too_cold = this->current_temperature < this->target_temperature_low;
  const bool too_hot = this->current_temperature > this->target_temperature_high;

  climate::ClimateAction target_action;
  if (too_cold) {
    // too cold -> enable heating if possible, else idle
    if (this->supports_heat_)
      target_action = climate::CLIMATE_ACTION_HEATING;
    else
      target_action = climate::CLIMATE_ACTION_IDLE;
  } else if (too_hot) {
    // too hot -> enable cooling if possible, else idle
    if (this->supports_cool_)
      target_action = climate::CLIMATE_ACTION_COOLING;
    else
      target_action = climate::CLIMATE_ACTION_IDLE;
  } else {
    // neither too hot nor too cold -> in range
    if (this->supports_cool_ && this->supports_heat_) {
      // if supports both ends, go to idle action
      target_action = climate::CLIMATE_ACTION_IDLE;
    } else {
      // else use current mode and don't change (hysteresis)
      target_action = this->action;
    }
  }

  this->switch_to_action_(target_action);
}
void BangBangClimate::switch_to_action_(climate::ClimateAction action) {
  if (action == this->action)
    // already in target mode
    return;

  if ((action == climate::CLIMATE_ACTION_OFF && this->action == climate::CLIMATE_ACTION_IDLE) ||
      (action == climate::CLIMATE_ACTION_IDLE && this->action == climate::CLIMATE_ACTION_OFF)) {
    // switching from OFF to IDLE or vice-versa
    // these only have visual difference. OFF means user manually disabled,
    // IDLE means it's in auto mode but value is in target range.
    this->action = action;
    this->publish_state();
    return;
  }

  if (this->prev_trigger_ != nullptr) {
    this->prev_trigger_->stop_action();
    this->prev_trigger_ = nullptr;
  }
  Trigger<> *trig;
  switch (action) {
    case climate::CLIMATE_ACTION_OFF:
    case climate::CLIMATE_ACTION_IDLE:
      trig = this->idle_trigger_;
      break;
    case climate::CLIMATE_ACTION_COOLING:
      trig = this->cool_trigger_;
      break;
    case climate::CLIMATE_ACTION_HEATING:
      trig = this->heat_trigger_;
      break;
    default:
      trig = nullptr;
  }
  assert(trig != nullptr);
  trig->trigger();
  this->action = action;
  this->prev_trigger_ = trig;
  this->publish_state();
}
void BangBangClimate::change_away_(bool away) {
  if (!away) {
    this->target_temperature_low = this->normal_config_.default_temperature_low;
    this->target_temperature_high = this->normal_config_.default_temperature_high;
  } else {
    this->target_temperature_low = this->away_config_.default_temperature_low;
    this->target_temperature_high = this->away_config_.default_temperature_high;
  }
  this->away = away;
}

void BangBangClimate::change_boost_(bool boost) {
  // TO DO
  ESP_LOGCONFIG(TAG, "  Boost: %s", YESNO(this->boost));
  this->boost = boost;
}

void BangBangClimate::change_eco_(bool eco) {
  // TO DO
  ESP_LOGCONFIG(TAG, "  Eco: %s", YESNO(this->eco));
  this->eco = eco;
}

void BangBangClimate::change_night_(bool night) {
  // TO DO
  ESP_LOGCONFIG(TAG, "  Night: %s", YESNO(this->night));
  this->night = night;
}

void BangBangClimate::set_normal_config(const BangBangClimateTargetTempConfig &normal_config) {
  this->normal_config_ = normal_config;
}
void BangBangClimate::set_away_config(const BangBangClimateTargetTempConfig &away_config) {
  this->supports_away_ = true;
  this->away_config_ = away_config;
}

BangBangClimate::BangBangClimate()
    : idle_trigger_(new Trigger<>()), cool_trigger_(new Trigger<>()), heat_trigger_(new Trigger<>()) {}
void BangBangClimate::set_sensor(sensor::Sensor *sensor) { this->sensor_ = sensor; }
Trigger<> *BangBangClimate::get_idle_trigger() const { return this->idle_trigger_; }
Trigger<> *BangBangClimate::get_cool_trigger() const { return this->cool_trigger_; }
void BangBangClimate::set_supports_cool(bool supports_cool) { this->supports_cool_ = supports_cool; }
Trigger<> *BangBangClimate::get_heat_trigger() const { return this->heat_trigger_; }
void BangBangClimate::set_supports_heat(bool supports_heat) { this->supports_heat_ = supports_heat; }
void BangBangClimate::set_supports_boost(bool supports_boost) { this->supports_boost_ = supports_boost; }
void BangBangClimate::set_supports_night(bool supports_night) { this->supports_night_ = supports_night; }
void BangBangClimate::set_supports_eco(bool supports_eco) { this->supports_eco_ = supports_eco; }

void BangBangClimate::dump_config() {
  LOG_CLIMATE("", "Bang Bang Climate", this);
  ESP_LOGCONFIG(TAG, "  Supports HEAT: %s", YESNO(this->supports_heat_));
  ESP_LOGCONFIG(TAG, "  Supports COOL: %s", YESNO(this->supports_cool_));
  ESP_LOGCONFIG(TAG, "  Supports AWAY mode: %s", YESNO(this->supports_away_));
  ESP_LOGCONFIG(TAG, "  Supports BOOST mode: %s", YESNO(this->supports_boost_));
  ESP_LOGCONFIG(TAG, "  Supports NIGHT mode: %s", YESNO(this->supports_night_));
  ESP_LOGCONFIG(TAG, "  Supports ECO mode: %s", YESNO(this->supports_eco_));
  ESP_LOGCONFIG(TAG, "  Default Target Temperature Low: %.1f°C", this->normal_config_.default_temperature_low);
  ESP_LOGCONFIG(TAG, "  Default Target Temperature High: %.1f°C", this->normal_config_.default_temperature_high);
}

BangBangClimateTargetTempConfig::BangBangClimateTargetTempConfig() = default;
BangBangClimateTargetTempConfig::BangBangClimateTargetTempConfig(float default_temperature_low,
                                                                 float default_temperature_high)
    : default_temperature_low(default_temperature_low), default_temperature_high(default_temperature_high) {}

}  // namespace bang_bang
}  // namespace esphome
