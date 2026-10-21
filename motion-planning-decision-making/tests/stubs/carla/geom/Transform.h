#pragma once

#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"

namespace carla {
namespace geom {

struct Transform {
  Location location;
  Rotation rotation;
};

}  // namespace geom
}  // namespace carla
