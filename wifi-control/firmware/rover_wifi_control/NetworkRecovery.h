#pragma once
#include <atomic>
#include <stdint.h>

namespace Rover {
enum class NetworkEvent { Start, Associated, GotIP, Disconnected, LostIP, Stop };

// All intervals are below 2^31 ms; subtraction remains valid across millis wrap.
inline uint32_t elapsedMs(uint32_t now, uint32_t then) { return uint32_t(now - then); }

// Callback signals survive a fast disconnect/reconnect between motor-task ticks.
// The consumed generation belongs to the caller's existing control mutex.
class DisconnectLatch {
  std::atomic<uint32_t> generation_{0};
 public:
  void signal() { generation_.fetch_add(1); }
  uint32_t generation() const { return generation_.load(); }
  bool consume(uint32_t &consumed) const {
    const uint32_t current = generation();
    if (current == consumed) return false;
    consumed = current;
    return true;
  }
};

struct RecoveryAttempt {
  bool due = false;
  bool startStation = false;
  uint32_t generation = 0;
};

// Pure policy. The owner serializes event(), reserve() and valid() with a short
// metadata lock. No network operation is performed under that lock.
class NetworkRecovery {
 public:
  static constexpr uint32_t GRACE_MS = 30000;
  static constexpr uint32_t MAX_BACKOFF_MS = 120000;
 private:
  uint32_t progressAt_ = 0, orderingAt_ = 0, lastAttemptAt_ = 0;
  uint32_t generation_ = 0, backoffMs_ = GRACE_MS;
  bool online_ = false;
 public:
  explicit NetworkRecovery(uint32_t now = 0)
      : progressAt_(now), orderingAt_(now), lastAttemptAt_(now) {}
  bool event(NetworkEvent event, uint32_t now) {
    // A delayed older snapshot must not undo newer progress, including at wrap.
    if (int32_t(now - orderingAt_) < 0) return false;
    orderingAt_ = now;
    ++generation_;
    if (event == NetworkEvent::GotIP) {
      online_ = true;
      progressAt_ = lastAttemptAt_ = now;
      backoffMs_ = GRACE_MS;
    } else if (event == NetworkEvent::Start || event == NetworkEvent::Associated) {
      progressAt_ = now;
    } else {
      // Repeated association failures are not progress and cannot postpone
      // recovery forever. A newly lost working connection gets a full grace.
      if (online_) progressAt_ = lastAttemptAt_ = now;
      online_ = false;
    }
    return true;
  }
  RecoveryAttempt reserve(uint32_t now, bool hasIP, bool associated, bool stationStarted) {
    // The owner polls at 250 ms. Refresh ordering even while online so a quiet
    // connection lasting >2^31 ms does not make its next real event look old.
    if (int32_t(now - orderingAt_) < 0) return {};
    orderingAt_ = now;
    if (hasIP) {
      if (!online_) event(NetworkEvent::GotIP, now);
      return {};
    }
    if (online_) event(NetworkEvent::LostIP, now);
    // Association includes the entire DHCP wait, even beyond the grace period.
    if (associated || elapsedMs(now, progressAt_) < GRACE_MS
        || elapsedMs(now, lastAttemptAt_) < backoffMs_) return {};
    lastAttemptAt_ = now;
    const RecoveryAttempt attempt{true, !stationStarted, generation_};
    backoffMs_ = backoffMs_ >= MAX_BACKOFF_MS / 2 ? MAX_BACKOFF_MS : backoffMs_ * 2;
    return attempt;
  }
  bool valid(const RecoveryAttempt &attempt, uint32_t now, bool hasIP, bool associated) const {
    return attempt.due && attempt.generation == generation_ && !online_ && !hasIP
        && !associated && elapsedMs(now, progressAt_) >= GRACE_MS;
  }
  uint32_t nextBackoffMs() const { return backoffMs_; }
};

// mDNS starts once per link generation; partial failures are explicitly freed
// by the caller before retrying, so service registrations cannot accumulate.
class MdnsRetry {
  uint32_t lastAttemptAt_ = 0, backoffMs_ = 5000;
  bool attempted_ = false, active_ = false;
 public:
  void reset() { attempted_ = active_ = false; backoffMs_ = 5000; }
  bool due(uint32_t now) const {
    return !active_ && (!attempted_ || elapsedMs(now, lastAttemptAt_) >= backoffMs_);
  }
  void result(uint32_t now, bool success) {
    if (attempted_ && !success) backoffMs_ = backoffMs_ >= 30000 ? 60000 : backoffMs_ * 2;
    lastAttemptAt_ = now;
    attempted_ = true;
    active_ = success;
  }
  bool active() const { return active_; }
};
}
