#ifndef LIMO_CORE_SPSC_RING_BUFFER_HPP_
#define LIMO_CORE_SPSC_RING_BUFFER_HPP_

#include <atomic>
#include <array>
#include <cstddef>
#include <optional>

namespace limo::core {

// Single-Producer Single-Consumer lock-free ring buffer.
// Used for intra-process streaming data (driver → processor).
// T must be default-constructible and move-assignable.
template <typename T, std::size_t Capacity>
class SPSCRingBuffer {
  static_assert(Capacity > 0, "Capacity must be > 0");
  static_assert((Capacity & (Capacity - 1)) == 0,
                "Capacity must be a power of 2 for efficient modulo");

 public:
  SPSCRingBuffer() : head_(0), tail_(0) {}

  // Producer: try to push an item. Returns false if full.
  bool try_push(T&& item) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) & (Capacity - 1);
    if (next == tail_.load(std::memory_order_acquire)) {
      return false;  // full
    }
    buffer_[head] = std::move(item);
    head_.store(next, std::memory_order_release);
    return true;
  }

  bool try_push(const T& item) {
    T copy = item;
    return try_push(std::move(copy));
  }

  // Consumer: try to pop an item. Returns nullopt if empty.
  std::optional<T> try_pop() {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
      return std::nullopt;  // empty
    }
    T item = std::move(buffer_[tail]);
    tail_.store((tail + 1) & (Capacity - 1), std::memory_order_release);
    return item;
  }

  // Consumer: peek at the latest item without removing.
  // Drains all but the last item and returns it.
  std::optional<T> try_pop_latest() {
    std::optional<T> latest;
    while (auto item = try_pop()) {
      latest = std::move(item);
    }
    return latest;
  }

  bool empty() const {
    return head_.load(std::memory_order_acquire) ==
           tail_.load(std::memory_order_acquire);
  }

  std::size_t capacity() const { return Capacity; }

 private:
  std::array<T, Capacity> buffer_;
  alignas(64) std::atomic<std::size_t> head_;
  alignas(64) std::atomic<std::size_t> tail_;
};

}  // namespace limo::core

#endif  // LIMO_CORE_SPSC_RING_BUFFER_HPP_
