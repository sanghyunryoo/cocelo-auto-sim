// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from core:msg/EventUser.idl
// generated code does not contain a copyright notice

#ifndef CORE__MSG__DETAIL__EVENT_USER__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define CORE__MSG__DETAIL__EVENT_USER__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "core/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "core/msg/detail/event_user__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace core
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_core
cdr_serialize(
  const core::msg::EventUser & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_core
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  core::msg::EventUser & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_core
get_serialized_size(
  const core::msg::EventUser & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_core
max_serialized_size_EventUser(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace core

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_core
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core, msg, EventUser)();

#ifdef __cplusplus
}
#endif

#endif  // CORE__MSG__DETAIL__EVENT_USER__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
