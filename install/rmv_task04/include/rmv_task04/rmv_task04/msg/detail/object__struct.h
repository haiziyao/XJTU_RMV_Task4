// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rmv_task04:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef RMV_TASK04__MSG__DETAIL__OBJECT__STRUCT_H_
#define RMV_TASK04__MSG__DETAIL__OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'class_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Object in the package rmv_task04.
/**
  * msg/Object.msg
 */
typedef struct rmv_task04__msg__Object
{
  /// 时间戳和坐标系
  std_msgs__msg__Header header;
  /// 物体类别
  rosidl_runtime_c__String class_name;
  /// 中心x坐标（像素）
  float x;
  /// 中心y坐标（像素）
  float y;
  /// 宽度（像素）
  float width;
  /// 高度（像素）
  float height;
  /// 置信度（0~1）
  float score;
} rmv_task04__msg__Object;

// Struct for a sequence of rmv_task04__msg__Object.
typedef struct rmv_task04__msg__Object__Sequence
{
  rmv_task04__msg__Object * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rmv_task04__msg__Object__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RMV_TASK04__MSG__DETAIL__OBJECT__STRUCT_H_
