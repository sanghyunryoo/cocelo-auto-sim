// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from core:msg/CommandUser.idl
// generated code does not contain a copyright notice

#ifndef CORE__MSG__DETAIL__COMMAND_USER__TRAITS_HPP_
#define CORE__MSG__DETAIL__COMMAND_USER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "core/msg/detail/command_user__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'odom'
#include "nav_msgs/msg/detail/odometry__traits.hpp"
// Member 'event'
#include "core/msg/detail/event_user__traits.hpp"

namespace core
{

namespace msg
{

inline void to_flow_style_yaml(
  const CommandUser & msg,
  std::ostream & out)
{
  out << "{";
  // member: odom
  {
    out << "odom: ";
    to_flow_style_yaml(msg.odom, out);
    out << ", ";
  }

  // member: event
  {
    out << "event: ";
    to_flow_style_yaml(msg.event, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CommandUser & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: odom
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "odom:\n";
    to_block_style_yaml(msg.odom, out, indentation + 2);
  }

  // member: event
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "event:\n";
    to_block_style_yaml(msg.event, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CommandUser & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace core

namespace rosidl_generator_traits
{

[[deprecated("use core::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const core::msg::CommandUser & msg,
  std::ostream & out, size_t indentation = 0)
{
  core::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use core::msg::to_yaml() instead")]]
inline std::string to_yaml(const core::msg::CommandUser & msg)
{
  return core::msg::to_yaml(msg);
}

template<>
inline const char * data_type<core::msg::CommandUser>()
{
  return "core::msg::CommandUser";
}

template<>
inline const char * name<core::msg::CommandUser>()
{
  return "core/msg/CommandUser";
}

template<>
struct has_fixed_size<core::msg::CommandUser>
  : std::integral_constant<bool, has_fixed_size<core::msg::EventUser>::value && has_fixed_size<nav_msgs::msg::Odometry>::value> {};

template<>
struct has_bounded_size<core::msg::CommandUser>
  : std::integral_constant<bool, has_bounded_size<core::msg::EventUser>::value && has_bounded_size<nav_msgs::msg::Odometry>::value> {};

template<>
struct is_message<core::msg::CommandUser>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CORE__MSG__DETAIL__COMMAND_USER__TRAITS_HPP_
