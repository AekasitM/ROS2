// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlesim_interfaces:srv/SendPosition.idl
// generated code does not contain a copyright notice

#ifndef TURTLESIM_INTERFACES__SRV__DETAIL__SEND_POSITION__TRAITS_HPP_
#define TURTLESIM_INTERFACES__SRV__DETAIL__SEND_POSITION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtlesim_interfaces/srv/detail/send_position__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace turtlesim_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SendPosition_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SendPosition_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SendPosition_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtlesim_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtlesim_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlesim_interfaces::srv::SendPosition_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlesim_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlesim_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlesim_interfaces::srv::SendPosition_Request & msg)
{
  return turtlesim_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlesim_interfaces::srv::SendPosition_Request>()
{
  return "turtlesim_interfaces::srv::SendPosition_Request";
}

template<>
inline const char * name<turtlesim_interfaces::srv::SendPosition_Request>()
{
  return "turtlesim_interfaces/srv/SendPosition_Request";
}

template<>
struct has_fixed_size<turtlesim_interfaces::srv::SendPosition_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<turtlesim_interfaces::srv::SendPosition_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<turtlesim_interfaces::srv::SendPosition_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace turtlesim_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SendPosition_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SendPosition_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SendPosition_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtlesim_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtlesim_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlesim_interfaces::srv::SendPosition_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlesim_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlesim_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlesim_interfaces::srv::SendPosition_Response & msg)
{
  return turtlesim_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlesim_interfaces::srv::SendPosition_Response>()
{
  return "turtlesim_interfaces::srv::SendPosition_Response";
}

template<>
inline const char * name<turtlesim_interfaces::srv::SendPosition_Response>()
{
  return "turtlesim_interfaces/srv/SendPosition_Response";
}

template<>
struct has_fixed_size<turtlesim_interfaces::srv::SendPosition_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlesim_interfaces::srv::SendPosition_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlesim_interfaces::srv::SendPosition_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<turtlesim_interfaces::srv::SendPosition>()
{
  return "turtlesim_interfaces::srv::SendPosition";
}

template<>
inline const char * name<turtlesim_interfaces::srv::SendPosition>()
{
  return "turtlesim_interfaces/srv/SendPosition";
}

template<>
struct has_fixed_size<turtlesim_interfaces::srv::SendPosition>
  : std::integral_constant<
    bool,
    has_fixed_size<turtlesim_interfaces::srv::SendPosition_Request>::value &&
    has_fixed_size<turtlesim_interfaces::srv::SendPosition_Response>::value
  >
{
};

template<>
struct has_bounded_size<turtlesim_interfaces::srv::SendPosition>
  : std::integral_constant<
    bool,
    has_bounded_size<turtlesim_interfaces::srv::SendPosition_Request>::value &&
    has_bounded_size<turtlesim_interfaces::srv::SendPosition_Response>::value
  >
{
};

template<>
struct is_service<turtlesim_interfaces::srv::SendPosition>
  : std::true_type
{
};

template<>
struct is_service_request<turtlesim_interfaces::srv::SendPosition_Request>
  : std::true_type
{
};

template<>
struct is_service_response<turtlesim_interfaces::srv::SendPosition_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TURTLESIM_INTERFACES__SRV__DETAIL__SEND_POSITION__TRAITS_HPP_
