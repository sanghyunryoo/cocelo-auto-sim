// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from core:msg/EventUser.idl
// generated code does not contain a copyright notice

#ifndef CORE__MSG__DETAIL__EVENT_USER__FUNCTIONS_H_
#define CORE__MSG__DETAIL__EVENT_USER__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "core/msg/rosidl_generator_c__visibility_control.h"

#include "core/msg/detail/event_user__struct.h"

/// Initialize msg/EventUser message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * core__msg__EventUser
 * )) before or use
 * core__msg__EventUser__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_core
bool
core__msg__EventUser__init(core__msg__EventUser * msg);

/// Finalize msg/EventUser message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_core
void
core__msg__EventUser__fini(core__msg__EventUser * msg);

/// Create msg/EventUser message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * core__msg__EventUser__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_core
core__msg__EventUser *
core__msg__EventUser__create();

/// Destroy msg/EventUser message.
/**
 * It calls
 * core__msg__EventUser__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_core
void
core__msg__EventUser__destroy(core__msg__EventUser * msg);

/// Check for msg/EventUser message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_core
bool
core__msg__EventUser__are_equal(const core__msg__EventUser * lhs, const core__msg__EventUser * rhs);

/// Copy a msg/EventUser message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_core
bool
core__msg__EventUser__copy(
  const core__msg__EventUser * input,
  core__msg__EventUser * output);

/// Initialize array of msg/EventUser messages.
/**
 * It allocates the memory for the number of elements and calls
 * core__msg__EventUser__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_core
bool
core__msg__EventUser__Sequence__init(core__msg__EventUser__Sequence * array, size_t size);

/// Finalize array of msg/EventUser messages.
/**
 * It calls
 * core__msg__EventUser__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_core
void
core__msg__EventUser__Sequence__fini(core__msg__EventUser__Sequence * array);

/// Create array of msg/EventUser messages.
/**
 * It allocates the memory for the array and calls
 * core__msg__EventUser__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_core
core__msg__EventUser__Sequence *
core__msg__EventUser__Sequence__create(size_t size);

/// Destroy array of msg/EventUser messages.
/**
 * It calls
 * core__msg__EventUser__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_core
void
core__msg__EventUser__Sequence__destroy(core__msg__EventUser__Sequence * array);

/// Check for msg/EventUser message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_core
bool
core__msg__EventUser__Sequence__are_equal(const core__msg__EventUser__Sequence * lhs, const core__msg__EventUser__Sequence * rhs);

/// Copy an array of msg/EventUser messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_core
bool
core__msg__EventUser__Sequence__copy(
  const core__msg__EventUser__Sequence * input,
  core__msg__EventUser__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CORE__MSG__DETAIL__EVENT_USER__FUNCTIONS_H_
