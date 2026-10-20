// Copyright (c) 2021, Davide
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>

// Receiving-side timing policy, independent of ROS and robot hardware. A
// successful speed command arms it; idle/position-mode gaps do not arm it.
class VelocityCommandWatchdog {
public:
  using Clock = std::chrono::steady_clock;
  explicit VelocityCommandWatchdog(double timeout = 0.05, double rate = 500.0)
  : timeout_(timeout) {
        if (!std::isfinite(rate) || rate <= 0 || !std::isfinite(timeout) ||
      timeout <= 0 || timeout > 1.0 || timeout < 1.0 / rate)
    {
      throw std::invalid_argument(
        "velocity_command_timeout must be finite, in [1/rate, 1] seconds; "
        "rate must be finite and positive");
}
  }
  void reset() {last_.reset();}
  void successfulCommand(Clock::time_point now) {last_ = now;}
  bool expired(Clock::time_point now, bool velocity_mode) const
  {
    return velocity_mode && last_ &&
           now - *last_ >= std::chrono::duration < double > (timeout_);
  }
  bool armed() const {return last_.has_value();}
  double timeout() const {return timeout_;}

private:
  double timeout_;
  std::optional < Clock::time_point > last_;
};
