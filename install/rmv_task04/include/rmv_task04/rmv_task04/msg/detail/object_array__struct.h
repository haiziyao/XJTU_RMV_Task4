// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rmv_task04:msg/ObjectArray.idl
// generated code does not contain a copyright notice

#ifndef RMV_TASK04__MSG__DETAIL__OBJECT_ARRAY__STRUCT_H_
#define RMV_TASK04__MSG__DETAIL__OBJECT_ARRAY__STRUCT_H_

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
// Member 'objects'
#include "rmv_task04/msg/detail/object__struct.h"

/// Struct defined in msg/ObjectArray in the package rmv_task04.
/**
  * msg/ObjectArray.msg
 */
typedef struct rmv_task04__msg__ObjectArray
{
  /// 与图像同步的时间戳
  std_msgs__msg__Header header;
  /// 物体列表
  rmv_task04__msg__Object__Sequence objects;
} rmv_task04__msg__ObjectArray;

// Struct for a sequence of rmv_task04__msg__ObjectArray.
typedef struct rmv_task04__msg__ObjectArray__Sequence
{
  rmv_task04__msg__ObjectArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rmv_task04__msg__ObjectArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RMV_TASK04__MSG__DETAIL__OBJECT_ARRAY__STRUCT_H_
