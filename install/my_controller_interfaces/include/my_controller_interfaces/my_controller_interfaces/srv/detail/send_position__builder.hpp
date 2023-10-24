// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_controller_interfaces:srv/SendPosition.idl
// generated code does not contain a copyright notice

#ifndef MY_CONTROLLER_INTERFACES__SRV__DETAIL__SEND_POSITION__BUILDER_HPP_
#define MY_CONTROLLER_INTERFACES__SRV__DETAIL__SEND_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_controller_interfaces/srv/detail/send_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_controller_interfaces
{

namespace srv
{

namespace builder
{

class Init_SendPosition_Request_position
{
public:
  Init_SendPosition_Request_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_controller_interfaces::srv::SendPosition_Request position(::my_controller_interfaces::srv::SendPosition_Request::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_controller_interfaces::srv::SendPosition_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_controller_interfaces::srv::SendPosition_Request>()
{
  return my_controller_interfaces::srv::builder::Init_SendPosition_Request_position();
}

}  // namespace my_controller_interfaces


namespace my_controller_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_controller_interfaces::srv::SendPosition_Response>()
{
  return ::my_controller_interfaces::srv::SendPosition_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace my_controller_interfaces

#endif  // MY_CONTROLLER_INTERFACES__SRV__DETAIL__SEND_POSITION__BUILDER_HPP_
