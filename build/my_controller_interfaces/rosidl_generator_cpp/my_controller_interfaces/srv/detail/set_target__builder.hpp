// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_controller_interfaces:srv/SetTarget.idl
// generated code does not contain a copyright notice

#ifndef MY_CONTROLLER_INTERFACES__SRV__DETAIL__SET_TARGET__BUILDER_HPP_
#define MY_CONTROLLER_INTERFACES__SRV__DETAIL__SET_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_controller_interfaces/srv/detail/set_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_controller_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetTarget_Request_target
{
public:
  Init_SetTarget_Request_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_controller_interfaces::srv::SetTarget_Request target(::my_controller_interfaces::srv::SetTarget_Request::_target_type arg)
  {
    msg_.target = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_controller_interfaces::srv::SetTarget_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_controller_interfaces::srv::SetTarget_Request>()
{
  return my_controller_interfaces::srv::builder::Init_SetTarget_Request_target();
}

}  // namespace my_controller_interfaces


namespace my_controller_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetTarget_Response_result
{
public:
  Init_SetTarget_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_controller_interfaces::srv::SetTarget_Response result(::my_controller_interfaces::srv::SetTarget_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_controller_interfaces::srv::SetTarget_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_controller_interfaces::srv::SetTarget_Response>()
{
  return my_controller_interfaces::srv::builder::Init_SetTarget_Response_result();
}

}  // namespace my_controller_interfaces

#endif  // MY_CONTROLLER_INTERFACES__SRV__DETAIL__SET_TARGET__BUILDER_HPP_
