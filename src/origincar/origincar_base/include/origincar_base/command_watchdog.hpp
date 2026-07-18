#ifndef ORIGINCAR_BASE__COMMAND_WATCHDOG_HPP_
#define ORIGINCAR_BASE__COMMAND_WATCHDOG_HPP_

#include <cmath>
#include <stdexcept>

class CommandWatchdog
{
public:
  explicit CommandWatchdog(double timeout_sec)
  : timeout_sec_(timeout_sec), last_command_sec_(0.0), armed_(false)
  {
    if (!std::isfinite(timeout_sec_) || timeout_sec_ <= 0.0) {
      throw std::invalid_argument("command watchdog timeout must be positive");
    }
  }

  void mark_command(double now_sec)
  {
    last_command_sec_ = now_sec;
    armed_ = true;
  }

  bool consume_stop(double now_sec)
  {
    if (!armed_ || now_sec - last_command_sec_ < timeout_sec_) {
      return false;
    }

    armed_ = false;
    return true;
  }

private:
  double timeout_sec_;
  double last_command_sec_;
  bool armed_;
};

#endif  // ORIGINCAR_BASE__COMMAND_WATCHDOG_HPP_
