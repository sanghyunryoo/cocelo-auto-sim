// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from core:msg/EventUser.idl
// generated code does not contain a copyright notice
#include "core/msg/detail/event_user__rosidl_typesupport_fastrtps_cpp.hpp"
#include "core/msg/detail/event_user__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

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
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: estop
  cdr << (ros_message.estop ? true : false);
  // Member: wake
  cdr << (ros_message.wake ? true : false);
  // Member: sleep
  cdr << (ros_message.sleep ? true : false);
  // Member: rough_drive_toggle
  cdr << (ros_message.rough_drive_toggle ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_core
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  core::msg::EventUser & ros_message)
{
  // Member: estop
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.estop = tmp ? true : false;
  }

  // Member: wake
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.wake = tmp ? true : false;
  }

  // Member: sleep
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.sleep = tmp ? true : false;
  }

  // Member: rough_drive_toggle
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.rough_drive_toggle = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_core
get_serialized_size(
  const core::msg::EventUser & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: estop
  {
    size_t item_size = sizeof(ros_message.estop);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: wake
  {
    size_t item_size = sizeof(ros_message.wake);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sleep
  {
    size_t item_size = sizeof(ros_message.sleep);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rough_drive_toggle
  {
    size_t item_size = sizeof(ros_message.rough_drive_toggle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_core
max_serialized_size_EventUser(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: estop
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: wake
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: sleep
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: rough_drive_toggle
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = core::msg::EventUser;
    is_plain =
      (
      offsetof(DataType, rough_drive_toggle) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _EventUser__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const core::msg::EventUser *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _EventUser__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<core::msg::EventUser *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _EventUser__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const core::msg::EventUser *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _EventUser__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_EventUser(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _EventUser__callbacks = {
  "core::msg",
  "EventUser",
  _EventUser__cdr_serialize,
  _EventUser__cdr_deserialize,
  _EventUser__get_serialized_size,
  _EventUser__max_serialized_size
};

static rosidl_message_type_support_t _EventUser__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_EventUser__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace core

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_core
const rosidl_message_type_support_t *
get_message_type_support_handle<core::msg::EventUser>()
{
  return &core::msg::typesupport_fastrtps_cpp::_EventUser__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, core, msg, EventUser)() {
  return &core::msg::typesupport_fastrtps_cpp::_EventUser__handle;
}

#ifdef __cplusplus
}
#endif
