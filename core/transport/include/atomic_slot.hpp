#ifndef LIMO_CORE_ATOMIC_SLOT_HPP_
#define LIMO_CORE_ATOMIC_SLOT_HPP_

#include <atomic>
#include <memory>

namespace limo::core {

// Atomic latest-value slot for intra-process "mailbox" communication.
// Producer atomically swaps in a new immutable snapshot.
// Consumers read the latest value without blocking.
// T should be an immutable data snapshot.
template <typename T>
class AtomicSlot {
 public:
  AtomicSlot() : ptr_(nullptr) {}

  // Producer: store a new value (thread-safe).
  void store(std::shared_ptr<const T> value) {
    std::atomic_store_explicit(&ptr_, std::move(value),
                               std::memory_order_release);
  }

  // Convenience: construct in-place and store.
  template <typename... Args>
  void emplace(Args&&... args) {
    store(std::make_shared<const T>(std::forward<Args>(args)...));
  }

  // Consumer: load the latest value (thread-safe). May return nullptr.
  std::shared_ptr<const T> load() const {
    return std::atomic_load_explicit(&ptr_, std::memory_order_acquire);
  }

  bool has_value() const { return load() != nullptr; }

 private:
  std::shared_ptr<const T> ptr_;
};

}  // namespace limo::core

#endif  // LIMO_CORE_ATOMIC_SLOT_HPP_
