// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from core:msg/CommandUser.idl
// generated code does not contain a copyright notice
#include "core/msg/detail/command_user__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `odom`
#include "nav_msgs/msg/detail/odometry__functions.h"
// Member `event`
#include "core/msg/detail/event_user__functions.h"

bool
core__msg__CommandUser__init(core__msg__CommandUser * msg)
{
  if (!msg) {
    return false;
  }
  // odom
  if (!nav_msgs__msg__Odometry__init(&msg->odom)) {
    core__msg__CommandUser__fini(msg);
    return false;
  }
  // event
  if (!core__msg__EventUser__init(&msg->event)) {
    core__msg__CommandUser__fini(msg);
    return false;
  }
  return true;
}

void
core__msg__CommandUser__fini(core__msg__CommandUser * msg)
{
  if (!msg) {
    return;
  }
  // odom
  nav_msgs__msg__Odometry__fini(&msg->odom);
  // event
  core__msg__EventUser__fini(&msg->event);
}

bool
core__msg__CommandUser__are_equal(const core__msg__CommandUser * lhs, const core__msg__CommandUser * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // odom
  if (!nav_msgs__msg__Odometry__are_equal(
      &(lhs->odom), &(rhs->odom)))
  {
    return false;
  }
  // event
  if (!core__msg__EventUser__are_equal(
      &(lhs->event), &(rhs->event)))
  {
    return false;
  }
  return true;
}

bool
core__msg__CommandUser__copy(
  const core__msg__CommandUser * input,
  core__msg__CommandUser * output)
{
  if (!input || !output) {
    return false;
  }
  // odom
  if (!nav_msgs__msg__Odometry__copy(
      &(input->odom), &(output->odom)))
  {
    return false;
  }
  // event
  if (!core__msg__EventUser__copy(
      &(input->event), &(output->event)))
  {
    return false;
  }
  return true;
}

core__msg__CommandUser *
core__msg__CommandUser__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core__msg__CommandUser * msg = (core__msg__CommandUser *)allocator.allocate(sizeof(core__msg__CommandUser), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core__msg__CommandUser));
  bool success = core__msg__CommandUser__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core__msg__CommandUser__destroy(core__msg__CommandUser * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core__msg__CommandUser__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core__msg__CommandUser__Sequence__init(core__msg__CommandUser__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core__msg__CommandUser * data = NULL;

  if (size) {
    data = (core__msg__CommandUser *)allocator.zero_allocate(size, sizeof(core__msg__CommandUser), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core__msg__CommandUser__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core__msg__CommandUser__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
core__msg__CommandUser__Sequence__fini(core__msg__CommandUser__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      core__msg__CommandUser__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

core__msg__CommandUser__Sequence *
core__msg__CommandUser__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core__msg__CommandUser__Sequence * array = (core__msg__CommandUser__Sequence *)allocator.allocate(sizeof(core__msg__CommandUser__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core__msg__CommandUser__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core__msg__CommandUser__Sequence__destroy(core__msg__CommandUser__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core__msg__CommandUser__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core__msg__CommandUser__Sequence__are_equal(const core__msg__CommandUser__Sequence * lhs, const core__msg__CommandUser__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core__msg__CommandUser__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core__msg__CommandUser__Sequence__copy(
  const core__msg__CommandUser__Sequence * input,
  core__msg__CommandUser__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core__msg__CommandUser);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core__msg__CommandUser * data =
      (core__msg__CommandUser *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core__msg__CommandUser__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core__msg__CommandUser__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core__msg__CommandUser__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
