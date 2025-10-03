// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rmv_task04:msg/ObjectArray.idl
// generated code does not contain a copyright notice

#ifndef RMV_TASK04__MSG__DETAIL__OBJECT_ARRAY__BUILDER_HPP_
#define RMV_TASK04__MSG__DETAIL__OBJECT_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rmv_task04/msg/detail/object_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rmv_task04
{

namespace msg
{

namespace builder
{

class Init_ObjectArray_objects
{
public:
  explicit Init_ObjectArray_objects(::rmv_task04::msg::ObjectArray & msg)
  : msg_(msg)
  {}
  ::rmv_task04::msg::ObjectArray objects(::rmv_task04::msg::ObjectArray::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rmv_task04::msg::ObjectArray msg_;
};

class Init_ObjectArray_header
{
public:
  Init_ObjectArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectArray_objects header(::rmv_task04::msg::ObjectArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObjectArray_objects(msg_);
  }

private:
  ::rmv_task04::msg::ObjectArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rmv_task04::msg::ObjectArray>()
{
  return rmv_task04::msg::builder::Init_ObjectArray_header();
}

}  // namespace rmv_task04

#endif  // RMV_TASK04__MSG__DETAIL__OBJECT_ARRAY__BUILDER_HPP_
