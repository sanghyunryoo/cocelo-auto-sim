// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from core:msg/CommandUser.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "core/msg/detail/command_user__rosidl_typesupport_introspection_c.h"
#include "core/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "core/msg/detail/command_user__functions.h"
#include "core/msg/detail/command_user__struct.h"


// Include directives for member types
// Member `odom`
#include "nav_msgs/msg/odometry.h"
// Member `odom`
#include "nav_msgs/msg/detail/odometry__rosidl_typesupport_introspection_c.h"
// Member `event`
#include "core/msg/event_user.h"
// Member `event`
#include "core/msg/detail/event_user__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void core__msg__CommandUser__rosidl_typesupport_introspection_c__CommandUser_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  core__msg__CommandUser__init(message_memory);
}

void core__msg__CommandUser__rosidl_typesupport_introspection_c__CommandUser_fini_function(void * message_memory)
{
  core__msg__CommandUser__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember core__msg__CommandUser__rosidl_typesupport_introspection_c__CommandUser_message_member_array[2] = {
  {
    "odom",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(core__msg__CommandUser, odom),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "event",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(core__msg__CommandUser, event),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers core__msg__CommandUser__rosidl_typesupport_introspection_c__CommandUser_message_members = {
  "core__msg",  // message namespace
  "CommandUser",  // message name
  2,  // number of fields
  sizeof(core__msg__CommandUser),
  core__msg__CommandUser__rosidl_typesupport_introspection_c__CommandUser_message_member_array,  // message members
  core__msg__CommandUser__rosidl_typesupport_introspection_c__CommandUser_init_function,  // function to initialize message memory (memory has to be allocated)
  core__msg__CommandUser__rosidl_typesupport_introspection_c__CommandUser_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t core__msg__CommandUser__rosidl_typesupport_introspection_c__CommandUser_message_type_support_handle = {
  0,
  &core__msg__CommandUser__rosidl_typesupport_introspection_c__CommandUser_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_core
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, core, msg, CommandUser)() {
  core__msg__CommandUser__rosidl_typesupport_introspection_c__CommandUser_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav_msgs, msg, Odometry)();
  core__msg__CommandUser__rosidl_typesupport_introspection_c__CommandUser_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, core, msg, EventUser)();
  if (!core__msg__CommandUser__rosidl_typesupport_introspection_c__CommandUser_message_type_support_handle.typesupport_identifier) {
    core__msg__CommandUser__rosidl_typesupport_introspection_c__CommandUser_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &core__msg__CommandUser__rosidl_typesupport_introspection_c__CommandUser_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
