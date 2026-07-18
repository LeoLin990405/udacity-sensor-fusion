#pragma once

#include <vector>

#include "carla/client/Client.h"
#include "carla/geom/Transform.h"

namespace carla {
namespace road {
using JuncId = int;
}  // namespace road

namespace client {

class Waypoint {
 public:
  geom::Transform GetTransform() const { return {}; }
  std::vector<boost::shared_ptr<Waypoint>> GetNext(float) const { return {}; }
  bool IsJunction() const { return false; }
  road::JuncId GetJunctionId() const { return -1; }
};

class Map {
 public:
  boost::shared_ptr<Waypoint> GetWaypoint(const geom::Location&) const {
    return boost::shared_ptr<Waypoint>(new Waypoint());
  }
};

}  // namespace client
}  // namespace carla
