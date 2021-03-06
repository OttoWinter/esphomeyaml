#pragma once

namespace esphome {
namespace fan {

class FanTraits {
 public:
  FanTraits() = default;
  FanTraits(bool oscillation, bool speed, bool direction, int speed_levels)
      : oscillation_(oscillation), speed_(speed), direction_(direction), speed_levels_(speed_levels) {}

  /// Return if this fan supports oscillation.
  bool supports_oscillation() const { return this->oscillation_; }
  /// Set whether this fan supports oscillation.
  void set_oscillation(bool oscillation) { this->oscillation_ = oscillation; }
  /// Return if this fan supports speed modes.
  bool supports_speed() const { return this->speed_; }
  /// Set whether this fan supports speed levels.
  void set_speed(bool speed) { this->speed_ = speed; }
  /// Return how many speed levels the fan has
  int supported_speed_levels() const { return this->speed_levels_; }
  /// Set how many speed levels this fan has.
  void set_supported_speed_levels(bool speed_levels) { this->speed_levels_ = speed_levels; }
  /// Return if this fan supports changing direction
  bool supports_direction() const { return this->direction_; }
  /// Set whether this fan supports changing direction
  void set_direction(bool direction) { this->direction_ = direction; }

 protected:
  bool oscillation_{false};
  bool speed_{false};
  bool direction_{false};
  int speed_levels_{};
};

}  // namespace fan
}  // namespace esphome
