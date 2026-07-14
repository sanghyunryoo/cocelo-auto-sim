// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from core:msg/EventUser.idl
// generated code does not contain a copyright notice

#ifndef CORE__MSG__DETAIL__EVENT_USER__STRUCT_HPP_
#define CORE__MSG__DETAIL__EVENT_USER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__core__msg__EventUser __attribute__((deprecated))
#else
# define DEPRECATED__core__msg__EventUser __declspec(deprecated)
#endif

namespace core
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EventUser_
{
  using Type = EventUser_<ContainerAllocator>;

  explicit EventUser_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->estop = false;
      this->wake = false;
      this->sleep = false;
      this->rough_drive_toggle = false;
    }
  }

  explicit EventUser_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->estop = false;
      this->wake = false;
      this->sleep = false;
      this->rough_drive_toggle = false;
    }
  }

  // field types and members
  using _estop_type =
    bool;
  _estop_type estop;
  using _wake_type =
    bool;
  _wake_type wake;
  using _sleep_type =
    bool;
  _sleep_type sleep;
  using _rough_drive_toggle_type =
    bool;
  _rough_drive_toggle_type rough_drive_toggle;

  // setters for named parameter idiom
  Type & set__estop(
    const bool & _arg)
  {
    this->estop = _arg;
    return *this;
  }
  Type & set__wake(
    const bool & _arg)
  {
    this->wake = _arg;
    return *this;
  }
  Type & set__sleep(
    const bool & _arg)
  {
    this->sleep = _arg;
    return *this;
  }
  Type & set__rough_drive_toggle(
    const bool & _arg)
  {
    this->rough_drive_toggle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    core::msg::EventUser_<ContainerAllocator> *;
  using ConstRawPtr =
    const core::msg::EventUser_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<core::msg::EventUser_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<core::msg::EventUser_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      core::msg::EventUser_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<core::msg::EventUser_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      core::msg::EventUser_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<core::msg::EventUser_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<core::msg::EventUser_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<core::msg::EventUser_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__core__msg__EventUser
    std::shared_ptr<core::msg::EventUser_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__core__msg__EventUser
    std::shared_ptr<core::msg::EventUser_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EventUser_ & other) const
  {
    if (this->estop != other.estop) {
      return false;
    }
    if (this->wake != other.wake) {
      return false;
    }
    if (this->sleep != other.sleep) {
      return false;
    }
    if (this->rough_drive_toggle != other.rough_drive_toggle) {
      return false;
    }
    return true;
  }
  bool operator!=(const EventUser_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EventUser_

// alias to use template instance with default allocator
using EventUser =
  core::msg::EventUser_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace core

#endif  // CORE__MSG__DETAIL__EVENT_USER__STRUCT_HPP_
