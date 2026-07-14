// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from core:msg/EventUser.idl
// generated code does not contain a copyright notice

#ifndef CORE__MSG__DETAIL__EVENT_USER__STRUCT_H_
#define CORE__MSG__DETAIL__EVENT_USER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/EventUser in the package core.
typedef struct core__msg__EventUser
{
  bool estop;
  bool wake;
  bool sleep;
  bool rough_drive_toggle;
} core__msg__EventUser;

// Struct for a sequence of core__msg__EventUser.
typedef struct core__msg__EventUser__Sequence
{
  core__msg__EventUser * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} core__msg__EventUser__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CORE__MSG__DETAIL__EVENT_USER__STRUCT_H_
