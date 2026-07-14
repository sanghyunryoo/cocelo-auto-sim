// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from core:msg/CommandUser.idl
// generated code does not contain a copyright notice

#ifndef CORE__MSG__DETAIL__COMMAND_USER__BUILDER_HPP_
#define CORE__MSG__DETAIL__COMMAND_USER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "core/msg/detail/command_user__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace core
{

namespace msg
{

namespace builder
{

class Init_CommandUser_event
{
public:
  explicit Init_CommandUser_event(::core::msg::CommandUser & msg)
  : msg_(msg)
  {}
  ::core::msg::CommandUser event(::core::msg::CommandUser::_event_type arg)
  {
    msg_.event = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core::msg::CommandUser msg_;
};

class Init_CommandUser_odom
{
public:
  Init_CommandUser_odom()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandUser_event odom(::core::msg::CommandUser::_odom_type arg)
  {
    msg_.odom = std::move(arg);
    return Init_CommandUser_event(msg_);
  }

private:
  ::core::msg::CommandUser msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::core::msg::CommandUser>()
{
  return core::msg::builder::Init_CommandUser_odom();
}

}  // namespace core

#endif  // CORE__MSG__DETAIL__COMMAND_USER__BUILDER_HPP_
