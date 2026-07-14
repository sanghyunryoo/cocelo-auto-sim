// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from core:msg/CommandUser.idl
// generated code does not contain a copyright notice

#ifndef CORE__MSG__DETAIL__COMMAND_USER__STRUCT_HPP_
#define CORE__MSG__DETAIL__COMMAND_USER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'odom'
#include "nav_msgs/msg/detail/odometry__struct.hpp"
// Member 'event'
#include "core/msg/detail/event_user__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__core__msg__CommandUser __attribute__((deprecated))
#else
# define DEPRECATED__core__msg__CommandUser __declspec(deprecated)
#endif

namespace core
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CommandUser_
{
  using Type = CommandUser_<ContainerAllocator>;

  explicit CommandUser_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : odom(_init),
    event(_init)
  {
    (void)_init;
  }

  explicit CommandUser_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : odom(_alloc, _init),
    event(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _odom_type =
    nav_msgs::msg::Odometry_<ContainerAllocator>;
  _odom_type odom;
  using _event_type =
    core::msg::EventUser_<ContainerAllocator>;
  _event_type event;

  // setters for named parameter idiom
  Type & set__odom(
    const nav_msgs::msg::Odometry_<ContainerAllocator> & _arg)
  {
    this->odom = _arg;
    return *this;
  }
  Type & set__event(
    const core::msg::EventUser_<ContainerAllocator> & _arg)
  {
    this->event = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    core::msg::CommandUser_<ContainerAllocator> *;
  using ConstRawPtr =
    const core::msg::CommandUser_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<core::msg::CommandUser_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<core::msg::CommandUser_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      core::msg::CommandUser_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<core::msg::CommandUser_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      core::msg::CommandUser_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<core::msg::CommandUser_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<core::msg::CommandUser_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<core::msg::CommandUser_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__core__msg__CommandUser
    std::shared_ptr<core::msg::CommandUser_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__core__msg__CommandUser
    std::shared_ptr<core::msg::CommandUser_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CommandUser_ & other) const
  {
    if (this->odom != other.odom) {
      return false;
    }
    if (this->event != other.event) {
      return false;
    }
    return true;
  }
  bool operator!=(const CommandUser_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CommandUser_

// alias to use template instance with default allocator
using CommandUser =
  core::msg::CommandUser_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace core

#endif  // CORE__MSG__DETAIL__COMMAND_USER__STRUCT_HPP_
