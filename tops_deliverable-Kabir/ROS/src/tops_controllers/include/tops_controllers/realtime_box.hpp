// Copyright 2026 Kabir Dhingra. MIT licence.
//
// Lock-free single-writer / single-reader handoff between a ROS callback
// thread and the 1 kHz control loop.
//
// ---------------------------------------------------------------------------
// Why this exists instead of realtime_tools::RealtimeBuffer
// ---------------------------------------------------------------------------
// Two reasons, in order of importance:
//
//   1. Self-containment. This workspace is meant to build from a clean
//      Ubuntu + apt ROS 2 install and keep building. realtime_tools' box
//      API has churned (RealtimeBuffer -> RealtimeThreadSafeBox) and this
//      is forty lines of well-understood code that will not churn.
//   2. It is worth being able to explain, in the report, exactly what
//      mechanism keeps the control loop from ever blocking. "A seqlock
//      with bounded retries" is an explanation. "A library class" is not.
//
// ---------------------------------------------------------------------------
// The mechanism: a seqlock
// ---------------------------------------------------------------------------
// A counter is incremented before a write and again after. Odd means "a
// write is in progress". The reader samples the counter, copies the data,
// samples the counter again, and accepts the copy only if both samples are
// equal and even. If not, it retries -- but only a bounded number of times,
// and then gives up and reports failure.
//
// That bound is the whole point. A mutex would make the control loop wait
// for a callback thread it does not control, which is a priority inversion
// and a missed deadline. This never waits: a failed read means the caller
// keeps last cycle's command for one more millisecond, which is completely
// harmless and vastly preferable.
//
// The stored type must be trivially copyable -- no std::string, no vector,
// no shared_ptr. That restriction is a feature: it forces the command and
// diagnostic structures to be fixed-size PODs, which is exactly what you
// want crossing into a real-time context anyway.

#ifndef TOPS_CONTROLLERS__REALTIME_BOX_HPP_
#define TOPS_CONTROLLERS__REALTIME_BOX_HPP_

#include <atomic>
#include <cstdint>
#include <type_traits>

namespace tops_controllers
{

template<typename T>
class RealtimeBox
{
  static_assert(
    std::is_trivially_copyable<T>::value,
    "RealtimeBox<T> requires a trivially copyable T. If this fails you are "
    "trying to pass something heap-backed into the control loop, which is "
    "the bug this static_assert exists to catch.");

public:
  RealtimeBox() = default;
  explicit RealtimeBox(const T & initial) : data_(initial) {}

  /// Single writer only. Safe to call from a ROS subscription callback.
  void write(const T & value)
  {
    const std::uint32_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_release);        // mark: write in progress
    std::atomic_thread_fence(std::memory_order_release);
    data_ = value;
    std::atomic_thread_fence(std::memory_order_release);
    seq_.store(s + 2, std::memory_order_release);        // mark: write complete
  }

  /// Wait-free within `kMaxRetries`. Returns false if the writer was active
  /// throughout; `out` is then untouched and the caller should keep whatever
  /// it had. Never blocks, never allocates, never throws.
  bool read(T & out) const
  {
    for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
      const std::uint32_t before = seq_.load(std::memory_order_acquire);
      if (before & 1u) {continue;}                        // writer mid-update
      std::atomic_thread_fence(std::memory_order_acquire);
      T candidate = data_;
      std::atomic_thread_fence(std::memory_order_acquire);
      const std::uint32_t after = seq_.load(std::memory_order_acquire);
      if (before == after) {
        out = candidate;
        return true;
      }
    }
    return false;
  }

  /// Number of completed writes. Used by the watchdog to distinguish
  /// "the same command again" from "no command at all".
  std::uint32_t generation() const { return seq_.load(std::memory_order_acquire) >> 1; }

private:
  static constexpr int kMaxRetries = 4;

  mutable std::atomic<std::uint32_t> seq_{0};
  T data_{};
};

}  // namespace tops_controllers

#endif  // TOPS_CONTROLLERS__REALTIME_BOX_HPP_
