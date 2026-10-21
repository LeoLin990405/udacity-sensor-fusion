#pragma once

#include <memory>

namespace boost {
template <typename T>
using shared_ptr = std::shared_ptr<T>;
}  // namespace boost

namespace carla {
namespace client {

class Client {};
class Map;
class Waypoint;

}  // namespace client
}  // namespace carla
