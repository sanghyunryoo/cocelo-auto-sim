// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from core:msg/EventUser.idl
// generated code does not contain a copyright notice

#ifndef CORE__MSG__DETAIL__EVENT_USER__TRAITS_HPP_
#define CORE__MSG__DETAIL__EVENT_USER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "core/msg/detail/event_user__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace core
{

namespace msg
{

inline void to_flow_style_yaml(
  const EventUser & msg,
  std::ostream & out)
{
  out << "{";
  // member: estop
  {
    out << "estop: ";
    rosidl_generator_traits::value_to_yaml(msg.estop, out);
    out << ", ";
  }

  // member: wake
  {
    out << "wake: ";
    rosidl_generator_traits::value_to_yaml(msg.wake, out);
    out << ", ";
  }

  // member: sleep
  {
    out << "sleep: ";
    rosidl_generator_traits::value_to_yaml(msg.sleep, out);
    out << ", ";
  }

  // member: rough_drive_toggle
  {
    out << "rough_drive_toggle: ";
    rosidl_generator_traits::value_to_yaml(msg.rough_drive_toggle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EventUser & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: estop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estop: ";
    rosidl_generator_traits::value_to_yaml(msg.estop, out);
    out << "\n";
  }

  // member: wake
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wake: ";
    rosidl_generator_traits::value_to_yaml(msg.wake, out);
    out << "\n";
  }

  // member: sleep
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sleep: ";
    rosidl_generator_traits::value_to_yaml(msg.sleep, out);
    out << "\n";
  }

  // member: rough_drive_toggle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rough_drive_toggle: ";
    rosidl_generator_traits::value_to_yaml(msg.rough_drive_toggle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EventUser & msg, bool use_flow_style = false)
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
  const core::msg::EventUser & msg,
  std::ostream & out, size_t indentation = 0)
{
  core::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use core::msg::to_yaml() instead")]]
inline std::string to_yaml(const core::msg::EventUser & msg)
{
  return core::msg::to_yaml(msg);
}

template<>
inline const char * data_type<core::msg::EventUser>()
{
  return "core::msg::EventUser";
}

template<>
inline const char * name<core::msg::EventUser>()
{
  return "core/msg/EventUser";
}

template<>
struct has_fixed_size<core::msg::EventUser>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<core::msg::EventUser>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<core::msg::EventUser>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CORE__MSG__DETAIL__EVENT_USER__TRAITS_HPP_
