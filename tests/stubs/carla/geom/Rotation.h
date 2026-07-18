#pragma once

namespace carla {
namespace geom {

struct Rotation {
  float pitch{0.0F};
  float yaw{0.0F};
  float roll{0.0F};

  Rotation() = default;
  Rotation(float pitch_value, float yaw_value, float roll_value)
      : pitch(pitch_value), yaw(yaw_value), roll(roll_value) {}
};

}  // namespace geom
}  // namespace carla
