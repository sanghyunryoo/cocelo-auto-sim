// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from core:msg/CommandUser.idl
// generated code does not contain a copyright notice

#ifndef CORE__MSG__DETAIL__COMMAND_USER__STRUCT_H_
#define CORE__MSG__DETAIL__COMMAND_USER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'odom'
#include "nav_msgs/msg/detail/odometry__struct.h"
// Member 'event'
#include "core/msg/detail/event_user__struct.h"

/// Struct defined in msg/CommandUser in the package core.
typedef struct core__msg__CommandUser
{
  nav_msgs__msg__Odometry odom;
  core__msg__EventUser event;
} core__msg__CommandUser;

// Struct for a sequence of core__msg__CommandUser.
typedef struct core__msg__CommandUser__Sequence
{
  core__msg__CommandUser * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core__msg__CommandUser__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CORE__MSG__DETAIL__COMMAND_USER__STRUCT_H_
