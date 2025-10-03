// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rmv_task04:msg/ObjectArray.idl
// generated code does not contain a copyright notice

#ifndef RMV_TASK04__MSG__DETAIL__OBJECT_ARRAY__STRUCT_HPP_
#define RMV_TASK04__MSG__DETAIL__OBJECT_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'objects'
#include "rmv_task04/msg/detail/object__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rmv_task04__msg__ObjectArray __attribute__((deprecated))
#else
# define DEPRECATED__rmv_task04__msg__ObjectArray __declspec(deprecated)
#endif

namespace rmv_task04
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectArray_
{
  using Type = ObjectArray_<ContainerAllocator>;

  explicit ObjectArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit ObjectArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _objects_type =
    std::vector<rmv_task04::msg::Object_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rmv_task04::msg::Object_<ContainerAllocator>>>;
  _objects_type objects;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__objects(
    const std::vector<rmv_task04::msg::Object_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rmv_task04::msg::Object_<ContainerAllocator>>> & _arg)
  {
    this->objects = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rmv_task04::msg::ObjectArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const rmv_task04::msg::ObjectArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rmv_task04::msg::ObjectArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rmv_task04::msg::ObjectArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rmv_task04::msg::ObjectArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rmv_task04::msg::ObjectArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rmv_task04::msg::ObjectArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rmv_task04::msg::ObjectArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rmv_task04::msg::ObjectArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rmv_task04::msg::ObjectArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rmv_task04__msg__ObjectArray
    std::shared_ptr<rmv_task04::msg::ObjectArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rmv_task04__msg__ObjectArray
    std::shared_ptr<rmv_task04::msg::ObjectArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->objects != other.objects) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectArray_

// alias to use template instance with default allocator
using ObjectArray =
  rmv_task04::msg::ObjectArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rmv_task04

#endif  // RMV_TASK04__MSG__DETAIL__OBJECT_ARRAY__STRUCT_HPP_
