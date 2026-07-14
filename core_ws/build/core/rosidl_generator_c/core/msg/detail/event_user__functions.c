// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from core:msg/EventUser.idl
// generated code does not contain a copyright notice
#include "core/msg/detail/event_user__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
core__msg__EventUser__init(core__msg__EventUser * msg)
{
  if (!msg) {
    return false;
  }
  // estop
  // wake
  // sleep
  // rough_drive_toggle
  return true;
}

void
core__msg__EventUser__fini(core__msg__EventUser * msg)
{
  if (!msg) {
    return;
  }
  // estop
  // wake
  // sleep
  // rough_drive_toggle
}

bool
core__msg__EventUser__are_equal(const core__msg__EventUser * lhs, const core__msg__EventUser * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // estop
  if (lhs->estop != rhs->estop) {
    return false;
  }
  // wake
  if (lhs->wake != rhs->wake) {
    return false;
  }
  // sleep
  if (lhs->sleep != rhs->sleep) {
    return false;
  }
  // rough_drive_toggle
  if (lhs->rough_drive_toggle != rhs->rough_drive_toggle) {
    return false;
  }
  return true;
}

bool
core__msg__EventUser__copy(
  const core__msg__EventUser * input,
  core__msg__EventUser * output)
{
  if (!input || !output) {
    return false;
  }
  // estop
  output->estop = input->estop;
  // wake
  output->wake = input->wake;
  // sleep
  output->sleep = input->sleep;
  // rough_drive_toggle
  output->rough_drive_toggle = input->rough_drive_toggle;
  return true;
}

core__msg__EventUser *
core__msg__EventUser__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core__msg__EventUser * msg = (core__msg__EventUser *)allocator.allocate(sizeof(core__msg__EventUser), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(core__msg__EventUser));
  bool success = core__msg__EventUser__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
core__msg__EventUser__destroy(core__msg__EventUser * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    core__msg__EventUser__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
core__msg__EventUser__Sequence__init(core__msg__EventUser__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core__msg__EventUser * data = NULL;

  if (size) {
    data = (core__msg__EventUser *)allocator.zero_allocate(size, sizeof(core__msg__EventUser), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = core__msg__EventUser__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        core__msg__EventUser__fini(&data[i - 1]);
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
core__msg__EventUser__Sequence__fini(core__msg__EventUser__Sequence * array)
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
      core__msg__EventUser__fini(&array->data[i]);
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

core__msg__EventUser__Sequence *
core__msg__EventUser__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  core__msg__EventUser__Sequence * array = (core__msg__EventUser__Sequence *)allocator.allocate(sizeof(core__msg__EventUser__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = core__msg__EventUser__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
core__msg__EventUser__Sequence__destroy(core__msg__EventUser__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    core__msg__EventUser__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
core__msg__EventUser__Sequence__are_equal(const core__msg__EventUser__Sequence * lhs, const core__msg__EventUser__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!core__msg__EventUser__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
core__msg__EventUser__Sequence__copy(
  const core__msg__EventUser__Sequence * input,
  core__msg__EventUser__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(core__msg__EventUser);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    core__msg__EventUser * data =
      (core__msg__EventUser *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!core__msg__EventUser__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          core__msg__EventUser__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!core__msg__EventUser__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
