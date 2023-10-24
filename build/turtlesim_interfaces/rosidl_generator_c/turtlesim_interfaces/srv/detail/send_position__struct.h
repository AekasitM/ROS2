// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlesim_interfaces:srv/SendPosition.idl
// generated code does not contain a copyright notice

#ifndef TURTLESIM_INTERFACES__SRV__DETAIL__SEND_POSITION__STRUCT_H_
#define TURTLESIM_INTERFACES__SRV__DETAIL__SEND_POSITION__STRUCT_H_

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

/// Struct defined in srv/SendPosition in the package turtlesim_interfaces.
typedef struct turtlesim_interfaces__srv__SendPosition_Request
{
  geometry_msgs__msg__Point position;
} turtlesim_interfaces__srv__SendPosition_Request;

// Struct for a sequence of turtlesim_interfaces__srv__SendPosition_Request.
typedef struct turtlesim_interfaces__srv__SendPosition_Request__Sequence
{
  turtlesim_interfaces__srv__SendPosition_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlesim_interfaces__srv__SendPosition_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SendPosition in the package turtlesim_interfaces.
typedef struct turtlesim_interfaces__srv__SendPosition_Response
{
  uint8_t structure_needs_at_least_one_member;
} turtlesim_interfaces__srv__SendPosition_Response;

// Struct for a sequence of turtlesim_interfaces__srv__SendPosition_Response.
typedef struct turtlesim_interfaces__srv__SendPosition_Response__Sequence
{
  turtlesim_interfaces__srv__SendPosition_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlesim_interfaces__srv__SendPosition_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLESIM_INTERFACES__SRV__DETAIL__SEND_POSITION__STRUCT_H_
