#pragma once

#include <cstddef>

namespace uWS {

constexpr int SERVER = 0;

enum class OpCode { TEXT };

template <int Mode>
class WebSocket {
 public:
  void send(const char*, std::size_t, OpCode) {}
  void close() {}
};

class HttpRequest {};

class Hub {
 public:
  template <typename Callback>
  void onMessage(Callback) {}

  template <typename Callback>
  void onConnection(Callback) {}

  template <typename Callback>
  void onDisconnection(Callback) {}

  bool listen(const char*, int) { return true; }
  void run() {}
};

}  // namespace uWS
