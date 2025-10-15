cmake_minimum_required(VERSION 3.8)
project(rmv_task04)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(HIKVISION_SDK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(HIKVISION_SDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/64)
# 关键：指定 ROS2 Humble 自带的 OpenCV（确保与 cv_bridge 版本一致）
set(OpenCV_DIR "/opt/ros/humble/lib/cmake/opencv4")

# 查找依赖（按功能分类，避免遗漏）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # 图像消息依赖
find_package(cv_bridge REQUIRED)    # OpenCV-ROS 图像转换依赖
find_package(OpenCV REQUIRED)       # 基于指定路径查找 OpenCV（4.5d 版本）
find_package(Ceres REQUIRED)        # Ceres 优化库依赖
find_package(rosidl_default_generators REQUIRED)  # 自定义消息生成依赖

# 查找 ONNX Runtime 头文件和库
find_path(ONNX_RUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS /usr/include/onnxruntime /usr/local/include/onnxruntime
)
find_library(ONNX_RUNTIME_LIB onnxruntime
  HINTS /usr/lib /usr/local/lib
)
# 未找到则报错
if(NOT ONNX_RUNTIME_INCLUDE_DIR OR NOT ONNX_RUNTIME_LIB)
  message(FATAL_ERROR "ONNX Runtime 未找到！请安装或指定路径。")
endif()


# 链接海康 SDK 库路径（相机节点需要）
link_directories(${HIKVISION_SDK_LIB_DIR})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # 跳过版权检查（根据需求选择）
  set(ament_cmake_cpplint_FOUND TRUE)    # 跳过代码风格检查（根据需求选择）
  ament_lint_auto_find_test_dependencies()
endif()


# 1. 生成自定义消息（必须在节点编译前执行，确保消息头文件生成）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Object.msg"
  "msg/ObjectArray.msg"
  DEPENDENCIES std_msgs  # 依赖标准消息包（Object.msg 中用了 std_msgs/Header）
)


# 2. 编译相机节点（ImgPublisher）
add_executable(ImgPublisher src/img_publisher.cpp)
# 依赖配置：仅保留相机节点实际需要的依赖
ament_target_dependencies(ImgPublisher
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  Ceres
)
# 头文件路径：添加海康 SDK、OpenCV、Ceres 的头文件目录
target_include_directories(ImgPublisher PRIVATE
  ${HIKVISION_SDK_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)
# 链接库：相机节点需要海康 SDK、OpenCV、Ceres（无需自定义消息库，因不使用 Object.msg）
target_link_libraries(ImgPublisher
  ${OpenCV_LIBS}          # 正确的 OpenCV 库变量（与 find_package(OpenCV) 对应）
  ${CERES_LIBRARIES}      # Ceres 库
  MvCameraControl         # 海康 SDK 核心库
)



add_executable(ImgDealer src/img_dealer.cpp)

ament_target_dependencies(ImgDealer
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  std_msgs
  
)
target_include_directories(ImgDealer PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${ONNX_RUNTIME_INCLUDE_DIR}  # 新增 ONNX Runtime 头文件路径
)


rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ImgDealer
  ${cpp_typesupport_target}  # 自定义消息的 C++ 类型支持库
  ${OpenCV_LIBS}             # 正确的 OpenCV 库变量（修复原错误 ${OpenCV_LIB}）
  ${ONNX_RUNTIME_INCLUDE_DIR}
)



install(TARGETS
  ImgPublisher
  ImgDealer
  DESTINATION lib/${PROJECT_NAME}  # 安装到 lib/rmv_task04 目录（ROS 标准路径）
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/  # launch 文件安装到 share/rmv_task04/launch
)

# 5. 导出配置（供其他包依赖时使用）
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  rosidl_default_runtime  # 导出自定义消息的运行时依赖
)

ament_package()
