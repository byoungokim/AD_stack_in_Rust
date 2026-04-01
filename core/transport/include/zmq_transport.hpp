#ifndef LIMO_CORE_ZMQ_TRANSPORT_HPP_
#define LIMO_CORE_ZMQ_TRANSPORT_HPP_

#include <zmq.hpp>
#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <vector>
#include <google/protobuf/message.h>

namespace limo::core {

// ZMQ Publisher: serializes Protobuf messages and publishes on a topic.
class ZmqPublisher {
 public:
  // endpoint: e.g., "tcp://*:5551"
  ZmqPublisher(zmq::context_t& ctx, const std::string& endpoint);
  ~ZmqPublisher();

  // Publish a Protobuf message with a topic prefix.
  // Wire format: [topic bytes][serialized protobuf bytes]
  bool publish(const std::string& topic,
               const google::protobuf::Message& msg);

 private:
  zmq::socket_t socket_;
};

// ZMQ Subscriber: receives messages and delivers via callback.
class ZmqSubscriber {
 public:
  using Callback = std::function<void(const std::string& topic,
                                      const void* data, size_t size)>;

  // endpoint: e.g., "tcp://localhost:5551"
  ZmqSubscriber(zmq::context_t& ctx, const std::string& endpoint,
                const std::string& topic_filter);
  ~ZmqSubscriber();

  // Start receiving in a background thread. Calls cb on each message.
  void start(Callback cb);

  // Stop the background receive thread.
  void stop();

 private:
  void receive_loop();

  zmq::socket_t socket_;
  std::string topic_filter_;
  Callback callback_;
  std::thread thread_;
  std::atomic<bool> running_{false};
};

}  // namespace limo::core

#endif  // LIMO_CORE_ZMQ_TRANSPORT_HPP_
