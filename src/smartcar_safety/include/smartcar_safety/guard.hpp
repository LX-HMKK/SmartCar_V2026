#pragma once

#include <cmath>
#include <string>

namespace smartcar_safety {

struct SafetyVerdict {
  bool allowed;
  std::string reason;
};

class SafetyGuard {
public:
  SafetyGuard(double cmd_timeout, double scan_timeout, double odom_timeout,
              double raw_odom_timeout, double min_voltage,
              bool require_scan, bool require_odom, bool require_raw_odom);

  void mark_command(double now_sec);
  void mark_command_invalid();
  void mark_scan(double now_sec);
  void mark_odom(double now_sec);
  void mark_raw_odom(double now_sec);
  void mark_voltage(float voltage, double now_sec);
  void set_emergency_stop(bool enabled);
  bool command_is_fresh(double now_sec) const;
  SafetyVerdict evaluate(double now_sec) const;

private:
  double cmd_timeout_, scan_timeout_, odom_timeout_, raw_odom_timeout_,
      min_voltage_;
  bool require_scan_, require_odom_, require_raw_odom_;
  double cmd_at_, scan_at_, odom_at_, raw_odom_at_, voltage_at_;
  float voltage_;
  bool cmd_invalid_, estop_;
  bool cmd_at_set_, scan_at_set_, odom_at_set_, raw_odom_at_set_,
      voltage_at_set_;

  static bool fresh(double arrival, double now, double timeout);
  static SafetyVerdict result(bool allowed, const std::string &reason);
};

}  // namespace smartcar_safety
