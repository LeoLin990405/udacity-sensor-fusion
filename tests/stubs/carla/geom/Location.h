#pragma once

#include "carla/geom/Vector3D.h"

namespace carla {
namespace geom {

struct Location : public Vector3D {
  Location() = default;
  Location(float x_value, float y_value, float z_value)
      : Vector3D(x_value, y_value, z_value) {}

  Vector3D operator-(const Location& other) const {
    return {x - other.x, y - other.y, z - other.z};
  }
};

}  // namespace geom
}  // namespace carla
