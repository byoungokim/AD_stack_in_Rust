#ifndef LIMO_CORE_BASE_NODE_HPP_
#define LIMO_CORE_BASE_NODE_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include <zmq.hpp>
#include <spdlog/spdlog.h>

#include "zmq_transport.hpp"

namespace limo::core {

// Base class for all C++ process nodes.
// Provides ZMQ context, heartbeat, peer monitoring, signal handling.
class BaseNode {
 public:
  // Heartbeat thresholds in seconds
  static constexpr double kWarnTimeout = 0.2;
  static constexpr double kDegradedTimeout = 0.5;
  static constexpr double kDeadTimeout = 1.0;

  explicit BaseNode(const std::string& name);
  virtual ~BaseNode();

  // Non-copyable, non-movable
  BaseNode(const BaseNode&) = delete;
  BaseNode& operator=(const BaseNode&) = delete;

  void start();
  void stop();
  bool is_running() const { return running_.load(); }
  const std::string& name() const { return name_; }
  zmq::context_t& zmq_ctx() { return zmq_ctx_; }

 protected:
  // Lifecycle hooks — override in subclasses
  virtual void on_start() {}
  virtual void run() = 0;
  virtual void on_stop() {}

  // Fault tolerance hooks
  virtual void on_peer_dead(const std::string& peer, double age_sec) {
    spdlog::error("[{}] Peer '{}' DEAD (no heartbeat for {:.1f}s)",
                  name_, peer, age_sec);
  }
  virtual void on_peer_degraded(const std::string& peer, double age_sec) {
    spdlog::warn("[{}] Peer '{}' DEGRADED ({:.0f}ms)",
                 name_, peer, age_sec * 1000);
  }

  std::atomic<bool> running_{false};

 private:
  void setup_heartbeat();
  void heartbeat_loop();
  void monitor_loop();

  std::string name_;
  zmq::context_t zmq_ctx_;

  std::unique_ptr<ZmqPublisher> hb_pub_;
  std::vector<std::unique_ptr<ZmqSubscriber>> hb_subs_;
  std::unordered_map<std::string, double> peer_last_seen_;
  std::mutex peer_mutex_;
  uint32_t hb_seq_ = 0;

  std::thread heartbeat_thread_;
  std::thread monitor_thread_;
};

}  // namespace limo::core

#endif  // LIMO_CORE_BASE_NODE_HPP_
