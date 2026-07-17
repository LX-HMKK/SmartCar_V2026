#ifndef ORIGINCAR_BASE__COMMAND_MODE_HPP_
#define ORIGINCAR_BASE__COMMAND_MODE_HPP_

#include <stdexcept>
#include <string>
#include <utility>

enum class CommandMode
{
  kTwist,
  kAckermann,
};

enum class CommandType
{
  kTwist,
  kAckermann,
};

inline CommandMode command_mode_from_string(const std::string & value)
{
  if (value == "twist") {
    return CommandMode::kTwist;
  }
  if (value == "ackermann") {
    return CommandMode::kAckermann;
  }
  throw std::invalid_argument("command_mode must be 'twist' or 'ackermann'");
}

inline bool accepts_command(CommandMode mode, CommandType type)
{
  return
    (mode == CommandMode::kTwist && type == CommandType::kTwist) ||
    (mode == CommandMode::kAckermann && type == CommandType::kAckermann);
}

template<typename SideEffect, typename Writer>
inline bool dispatch_command(
  CommandMode mode,
  CommandType type,
  SideEffect && side_effect,
  Writer && writer)
{
  if (!accepts_command(mode, type)) {
    return false;
  }

  std::forward<SideEffect>(side_effect)();
  std::forward<Writer>(writer)();
  return true;
}

#endif  // ORIGINCAR_BASE__COMMAND_MODE_HPP_
