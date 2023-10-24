// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_controller_interfaces:srv/SetTarget.idl
// generated code does not contain a copyright notice

#ifndef MY_CONTROLLER_INTERFACES__SRV__DETAIL__SET_TARGET__STRUCT_HPP_
#define MY_CONTROLLER_INTERFACES__SRV__DETAIL__SET_TARGET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'target'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__my_controller_interfaces__srv__SetTarget_Request __attribute__((deprecated))
#else
# define DEPRECATED__my_controller_interfaces__srv__SetTarget_Request __declspec(deprecated)
#endif

namespace my_controller_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetTarget_Request_
{
  using Type = SetTarget_Request_<ContainerAllocator>;

  explicit SetTarget_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target(_init)
  {
    (void)_init;
  }

  explicit SetTarget_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _target_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _target_type target;

  // setters for named parameter idiom
  Type & set__target(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->target = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_controller_interfaces::srv::SetTarget_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_controller_interfaces::srv::SetTarget_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_controller_interfaces::srv::SetTarget_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_controller_interfaces::srv::SetTarget_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_controller_interfaces::srv::SetTarget_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_controller_interfaces::srv::SetTarget_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_controller_interfaces::srv::SetTarget_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_controller_interfaces::srv::SetTarget_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_controller_interfaces::srv::SetTarget_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_controller_interfaces::srv::SetTarget_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_controller_interfaces__srv__SetTarget_Request
    std::shared_ptr<my_controller_interfaces::srv::SetTarget_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_controller_interfaces__srv__SetTarget_Request
    std::shared_ptr<my_controller_interfaces::srv::SetTarget_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetTarget_Request_ & other) const
  {
    if (this->target != other.target) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetTarget_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetTarget_Request_

// alias to use template instance with default allocator
using SetTarget_Request =
  my_controller_interfaces::srv::SetTarget_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace my_controller_interfaces


#ifndef _WIN32
# define DEPRECATED__my_controller_interfaces__srv__SetTarget_Response __attribute__((deprecated))
#else
# define DEPRECATED__my_controller_interfaces__srv__SetTarget_Response __declspec(deprecated)
#endif

namespace my_controller_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetTarget_Response_
{
  using Type = SetTarget_Response_<ContainerAllocator>;

  explicit SetTarget_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  explicit SetTarget_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  // field types and members
  using _result_type =
    bool;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const bool & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_controller_interfaces::srv::SetTarget_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_controller_interfaces::srv::SetTarget_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_controller_interfaces::srv::SetTarget_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_controller_interfaces::srv::SetTarget_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_controller_interfaces::srv::SetTarget_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_controller_interfaces::srv::SetTarget_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_controller_interfaces::srv::SetTarget_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_controller_interfaces::srv::SetTarget_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_controller_interfaces::srv::SetTarget_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_controller_interfaces::srv::SetTarget_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_controller_interfaces__srv__SetTarget_Response
    std::shared_ptr<my_controller_interfaces::srv::SetTarget_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_controller_interfaces__srv__SetTarget_Response
    std::shared_ptr<my_controller_interfaces::srv::SetTarget_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetTarget_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetTarget_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetTarget_Response_

// alias to use template instance with default allocator
using SetTarget_Response =
  my_controller_interfaces::srv::SetTarget_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace my_controller_interfaces

namespace my_controller_interfaces
{

namespace srv
{

struct SetTarget
{
  using Request = my_controller_interfaces::srv::SetTarget_Request;
  using Response = my_controller_interfaces::srv::SetTarget_Response;
};

}  // namespace srv

}  // namespace my_controller_interfaces

#endif  // MY_CONTROLLER_INTERFACES__SRV__DETAIL__SET_TARGET__STRUCT_HPP_
