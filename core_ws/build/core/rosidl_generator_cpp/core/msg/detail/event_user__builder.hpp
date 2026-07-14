// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from core:msg/EventUser.idl
// generated code does not contain a copyright notice

#ifndef CORE__MSG__DETAIL__EVENT_USER__BUILDER_HPP_
#define CORE__MSG__DETAIL__EVENT_USER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "core/msg/detail/event_user__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace core
{

namespace msg
{

namespace builder
{

class Init_EventUser_rough_drive_toggle
{
public:
  explicit Init_EventUser_rough_drive_toggle(::core::msg::EventUser & msg)
  : msg_(msg)
  {}
  ::core::msg::EventUser rough_drive_toggle(::core::msg::EventUser::_rough_drive_toggle_type arg)
  {
    msg_.rough_drive_toggle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core::msg::EventUser msg_;
};

class Init_EventUser_sleep
{
public:
  explicit Init_EventUser_sleep(::core::msg::EventUser & msg)
  : msg_(msg)
  {}
  Init_EventUser_rough_drive_toggle sleep(::core::msg::EventUser::_sleep_type arg)
  {
    msg_.sleep = std::move(arg);
    return Init_EventUser_rough_drive_toggle(msg_);
  }

private:
  ::core::msg::EventUser msg_;
};

class Init_EventUser_wake
{
public:
  explicit Init_EventUser_wake(::core::msg::EventUser & msg)
  : msg_(msg)
  {}
  Init_EventUser_sleep wake(::core::msg::EventUser::_wake_type arg)
  {
    msg_.wake = std::move(arg);
    return Init_EventUser_sleep(msg_);
  }

private:
  ::core::msg::EventUser msg_;
};

class Init_EventUser_estop
{
public:
  Init_EventUser_estop()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EventUser_wake estop(::core::msg::EventUser::_estop_type arg)
  {
    msg_.estop = std::move(arg);
    return Init_EventUser_wake(msg_);
  }

private:
  ::core::msg::EventUser msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::core::msg::EventUser>()
{
  return core::msg::builder::Init_EventUser_estop();
}

}  // namespace core

#endif  // CORE__MSG__DETAIL__EVENT_USER__BUILDER_HPP_
