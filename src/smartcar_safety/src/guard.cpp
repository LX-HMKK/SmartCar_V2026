#include "smartcar_safety/guard.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace smartcar_safety {

SafetyGuard::SafetyGuard(double cmd_timeout, double scan_timeout,
                         double odom_timeout, double raw_odom_timeout,
                         double min_voltage, bool require_scan,
                         bool require_odom, bool require_raw_odom)
    : cmd_timeout_(cmd_timeout),
      scan_timeout_(scan_timeout),
      odom_timeout_(odom_timeout),
      raw_odom_timeout_(raw_odom_timeout),
      min_voltage_(min_voltage),
      require_scan_(require_scan),
      require_odom_(require_odom),
      require_raw_odom_(require_raw_odom),
      cmd_at_(NAN),
      scan_at_(NAN),
      odom_at_(NAN),
      raw_odom_at_(NAN),
      voltage_at_(NAN),
      voltage_(NAN),
      cmd_invalid_(false),
      estop_(false),
      cmd_at_set_(false),
      scan_at_set_(false),
      odom_at_set_(false),
      raw_odom_at_set_(false),
      voltage_at_set_(false) {
  if (!std::isfinite(cmd_timeout) || cmd_timeout <= 0.0) {
    throw std::invalid_argument("command_timeout_sec must be positive finite");
  }
  if (!std::isfinite(scan_timeout) || scan_timeout <= 0.0) {
    throw std::invalid_argument("scan_timeout_sec must be positive finite");
  }
  if (!std::isfinite(odom_timeout) || odom_timeout <= 0.0) {
    throw std::invalid_argument("odom_timeout_sec must be positive finite");
  }
  if (!std::isfinite(raw_odom_timeout) || raw_odom_timeout <= 0.0) {
    throw std::invalid_argument(
        "raw_odom_timeout_sec must be positive finite");
  }
  if (!std::isfinite(min_voltage) || min_voltage < 0.0) {
    throw std::invalid_argument("minimum_voltage must be nonnegative finite");
  }
}

void SafetyGuard::mark_command(double now_sec) {
  cmd_at_ = now_sec;
  cmd_at_set_ = true;
  cmd_invalid_ = false;
}

void SafetyGuard::mark_command_invalid() { cmd_invalid_ = true; }

void SafetyGuard::mark_scan(double now_sec) {
  scan_at_ = now_sec;
  scan_at_set_ = true;
}

void SafetyGuard::mark_odom(double now_sec) {
  odom_at_ = now_sec;
  odom_at_set_ = true;
}

void SafetyGuard::mark_raw_odom(double now_sec) {
  raw_odom_at_ = now_sec;
  raw_odom_at_set_ = true;
}

void SafetyGuard::mark_voltage(float voltage, double now_sec) {
  voltage_ = voltage;
  voltage_at_ = now_sec;
  voltage_at_set_ = true;
}

void SafetyGuard::set_emergency_stop(bool enabled) { estop_ = enabled; }

bool SafetyGuard::command_is_fresh(double now_sec) const {
  return fresh(cmd_at_, now_sec, cmd_timeout_);
}

SafetyVerdict SafetyGuard::evaluate(double now_sec) const {
  if (estop_) {
    return result(false, "emergency_stop");
  }

  if (cmd_invalid_) {
    return result(false, "command_invalid");
  }

  if (!cmd_at_set_) {
    return result(false, "command_missing");
  }
  if (!fresh(cmd_at_, now_sec, cmd_timeout_)) {
    return result(false, "command_stale");
  }

  if (require_scan_) {
    if (!scan_at_set_) {
      return result(false, "scan_missing");
    }
    if (!fresh(scan_at_, now_sec, scan_timeout_)) {
      return result(false, "scan_stale");
    }
  }

  if (require_odom_) {
    if (!odom_at_set_) {
      return result(false, "odom_missing");
    }
    if (!fresh(odom_at_, now_sec, odom_timeout_)) {
      return result(false, "odom_stale");
    }
  }

  if (require_raw_odom_) {
    if (!raw_odom_at_set_) {
      return result(false, "raw_odom_missing");
    }
    if (!fresh(raw_odom_at_, now_sec, raw_odom_timeout_)) {
      return result(false, "raw_odom_stale");
    }
  }

  if (min_voltage_ > 0.0) {
    if (!voltage_at_set_) {
      return result(false, "voltage_missing");
    }
    if (!std::isfinite(voltage_)) {
      return result(false, "voltage_invalid");
    }
    if (voltage_ < min_voltage_) {
      return result(false, "voltage_low");
    }
  }

  return result(true, "ok");
}

bool SafetyGuard::fresh(double arrival, double now, double timeout) {
  if (std::isnan(arrival)) {
    return false;
  }
  double age = now - arrival;
  return age >= 0.0 && age <= timeout;
}

SafetyVerdict SafetyGuard::result(bool allowed, const std::string &reason) {
  return SafetyVerdict{allowed, reason};
}

}  // namespace smartcar_safety
