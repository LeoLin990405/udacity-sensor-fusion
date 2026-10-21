#pragma once

namespace carla {
namespace geom {

struct Vector3D {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};

  Vector3D() = default;
  Vector3D(float x_value, float y_value, float z_value)
      : x(x_value), y(y_value), z(z_value) {}
};

}  // namespace geom
}  // namespace carla
