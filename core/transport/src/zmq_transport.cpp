#include "zmq_transport.hpp"
#include <spdlog/spdlog.h>

namespace limo::core {

// --- ZmqPublisher ---

ZmqPublisher::ZmqPublisher(zmq::context_t& ctx, const std::string& endpoint)
    : socket_(ctx, zmq::socket_type::pub) {
  socket_.bind(endpoint);
  spdlog::info("ZmqPublisher bound to {}", endpoint);
}

ZmqPublisher::~ZmqPublisher() {
  socket_.close();
}

bool ZmqPublisher::publish(const std::string& topic,
                           const google::protobuf::Message& msg) {
  std::string serialized;
  if (!msg.SerializeToString(&serialized)) {
    spdlog::error("Failed to serialize message for topic '{}'", topic);
    return false;
  }

  // Send topic frame
  zmq::message_t topic_msg(topic.data(), topic.size());
  socket_.send(topic_msg, zmq::send_flags::sndmore);

  // Send data frame
  zmq::message_t data_msg(serialized.data(), serialized.size());
  socket_.send(data_msg, zmq::send_flags::none);
  return true;
}

// --- ZmqSubscriber ---

ZmqSubscriber::ZmqSubscriber(zmq::context_t& ctx,
                             const std::string& endpoint,
                             const std::string& topic_filter)
    : socket_(ctx, zmq::socket_type::sub), topic_filter_(topic_filter) {
  socket_.connect(endpoint);
  socket_.set(zmq::sockopt::subscribe, topic_filter);
  // Set receive timeout so the loop can check running_ flag
  socket_.set(zmq::sockopt::rcvtimeo, 100);  // 100ms
  spdlog::info("ZmqSubscriber connected to {} [filter='{}']", endpoint,
               topic_filter);
}

ZmqSubscriber::~ZmqSubscriber() {
  stop();
  socket_.close();
}

void ZmqSubscriber::start(Callback cb) {
  if (running_.load()) return;
  callback_ = std::move(cb);
  running_.store(true);
  thread_ = std::thread(&ZmqSubscriber::receive_loop, this);
}

void ZmqSubscriber::stop() {
  running_.store(false);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void ZmqSubscriber::receive_loop() {
  while (running_.load()) {
    zmq::message_t topic_msg;
    auto topic_result = socket_.recv(topic_msg, zmq::recv_flags::none);
    if (!topic_result) continue;  // timeout or error

    zmq::message_t data_msg;
    auto data_result = socket_.recv(data_msg, zmq::recv_flags::none);
    if (!data_result) continue;

    std::string topic(static_cast<const char*>(topic_msg.data()),
                      topic_msg.size());

    if (callback_) {
      callback_(topic, data_msg.data(), data_msg.size());
    }
  }
}

}  // namespace limo::core
