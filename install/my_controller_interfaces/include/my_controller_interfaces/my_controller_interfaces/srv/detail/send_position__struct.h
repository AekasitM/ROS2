// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_controller_interfaces:srv/SendPosition.idl
// generated code does not contain a copyright notice

#ifndef MY_CONTROLLER_INTERFACES__SRV__DETAIL__SEND_POSITION__STRUCT_H_
#define MY_CONTROLLER_INTERFACES__SRV__DETAIL__SEND_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in srv/SendPosition in the package my_controller_interfaces.
typedef struct my_controller_interfaces__srv__SendPosition_Request
{
  geometry_msgs__msg__Point position;
} my_controller_interfaces__srv__SendPosition_Request;

// Struct for a sequence of my_controller_interfaces__srv__SendPosition_Request.
typedef struct my_controller_interfaces__srv__SendPosition_Request__Sequence
{
  my_controller_interfaces__srv__SendPosition_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_controller_interfaces__srv__SendPosition_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SendPosition in the package my_controller_interfaces.
typedef struct my_controller_interfaces__srv__SendPosition_Response
{
  uint8_t structure_needs_at_least_one_member;
} my_controller_interfaces__srv__SendPosition_Response;

// Struct for a sequence of my_controller_interfaces__srv__SendPosition_Response.
typedef struct my_controller_interfaces__srv__SendPosition_Response__Sequence
{
  my_controller_interfaces__srv__SendPosition_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_controller_interfaces__srv__SendPosition_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_CONTROLLER_INTERFACES__SRV__DETAIL__SEND_POSITION__STRUCT_H_
