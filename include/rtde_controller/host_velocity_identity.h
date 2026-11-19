#pragma once

#include <arspain_interfaces/msg/identified_joint_velocity.hpp>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

// Host-only command ownership. This class never creates robot acknowledgements.
// A route is selected by its first valid command, and reset only when the
// driver's explicit velocity gate is reopened after queued messages are drained.
class HostVelocityIdentity {
public:
  explicit HostVelocityIdentity(std::string clock_id) : clock_id_(std::move(clock_id)) {}

  void reopen(std::uint64_t now) {
    minimum_publication_ns_ = now;
    owner_ = Owner::None;
    session_.clear();
    sequence_ = cycle_ = epoch_ = 0;
    mode_.clear();
  }

  bool legacy() {
    if (owner_ == Owner::Identified) return false;
    owner_ = Owner::Legacy;
    return true;
  }

  bool identified() const { return owner_ == Owner::Identified; }

  const char *accept(const arspain_interfaces::msg::IdentifiedJointVelocity &message,
                     std::uint64_t now, std::uint64_t maximum_age_ns) {
    if (owner_ == Owner::Legacy) return "legacy_velocity_owner";
    if (message.schema_version != 1 || message.clock_id != clock_id_ ||
        message.controller_session.empty() || message.controller_session.size() >= 256 ||
        !message.sequence || !message.cycle_id ||
        (message.mode != "FREE" && message.mode != "ON_PATH" && message.mode != "RETURNING"))
      return "invalid_host_identity";
    if (!message.publication_begin_ns || message.publication_begin_ns <= minimum_publication_ns_ ||
        message.publication_begin_ns > now || now - message.publication_begin_ns >= maximum_age_ns)
      return "stale_host_command";
    for (double velocity : message.velocity)
      if (!std::isfinite(velocity)) return "nonfinite_velocity";
    if (owner_ == Owner::Identified &&
        (message.controller_session != session_ || message.sequence <= sequence_ ||
         message.cycle_id <= cycle_ || message.mode_epoch < epoch_ ||
         (message.mode_epoch == epoch_ && message.mode != mode_)))
      return "stale_or_conflicting_identity";
    owner_ = Owner::Identified;
    session_ = message.controller_session;
    sequence_ = message.sequence;
    cycle_ = message.cycle_id;
    epoch_ = message.mode_epoch;
    mode_ = message.mode;
    return nullptr;
  }

private:
  enum class Owner { None, Legacy, Identified };
  Owner owner_{Owner::None};
  std::string clock_id_, session_, mode_;
  std::uint64_t minimum_publication_ns_{0}, sequence_{0}, cycle_{0}, epoch_{0};
};
