// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rmv_task04:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef RMV_TASK04__MSG__DETAIL__OBJECT__BUILDER_HPP_
#define RMV_TASK04__MSG__DETAIL__OBJECT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rmv_task04/msg/detail/object__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rmv_task04
{

namespace msg
{

namespace builder
{

class Init_Object_score
{
public:
  explicit Init_Object_score(::rmv_task04::msg::Object & msg)
  : msg_(msg)
  {}
  ::rmv_task04::msg::Object score(::rmv_task04::msg::Object::_score_type arg)
  {
    msg_.score = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rmv_task04::msg::Object msg_;
};

class Init_Object_height
{
public:
  explicit Init_Object_height(::rmv_task04::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_score height(::rmv_task04::msg::Object::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_Object_score(msg_);
  }

private:
  ::rmv_task04::msg::Object msg_;
};

class Init_Object_width
{
public:
  explicit Init_Object_width(::rmv_task04::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_height width(::rmv_task04::msg::Object::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_Object_height(msg_);
  }

private:
  ::rmv_task04::msg::Object msg_;
};

class Init_Object_y
{
public:
  explicit Init_Object_y(::rmv_task04::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_width y(::rmv_task04::msg::Object::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Object_width(msg_);
  }

private:
  ::rmv_task04::msg::Object msg_;
};

class Init_Object_x
{
public:
  explicit Init_Object_x(::rmv_task04::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_y x(::rmv_task04::msg::Object::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Object_y(msg_);
  }

private:
  ::rmv_task04::msg::Object msg_;
};

class Init_Object_class_name
{
public:
  explicit Init_Object_class_name(::rmv_task04::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_x class_name(::rmv_task04::msg::Object::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_Object_x(msg_);
  }

private:
  ::rmv_task04::msg::Object msg_;
};

class Init_Object_header
{
public:
  Init_Object_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Object_class_name header(::rmv_task04::msg::Object::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Object_class_name(msg_);
  }

private:
  ::rmv_task04::msg::Object msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rmv_task04::msg::Object>()
{
  return rmv_task04::msg::builder::Init_Object_header();
}

}  // namespace rmv_task04

#endif  // RMV_TASK04__MSG__DETAIL__OBJECT__BUILDER_HPP_
