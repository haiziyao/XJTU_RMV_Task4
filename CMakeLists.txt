cmake_minimum_required(VERSION 3.8)
project(rmv_task04)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



set(HIKVISION_SDK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(HIKVISION_SDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/64)
set(OpenCV_DIR "/opt/ros/humble/lib/cmake/opencv4")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# （可选）若节点需要发布/订阅图像，需添加sensor_msgs（图像消息类型）
find_package(sensor_msgs REQUIRED)
# （可选）若用OpenCV处理图像后转ROS消息，需添加cv_bridge（OpenCV与ROS图像转换）
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)


link_directories(${HIKVISION_SDK_LIB_DIR})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



add_executable(ImgPublisher src/img_publisher.cpp)
ament_target_dependencies(ImgPublisher
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  Ceres
)
target_include_directories(ImgPublisher PRIVATE
  ${HIKVISION_SDK_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)
target_link_libraries(ImgPublisher
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  # 海康SDK库（如果需要）
  MvCameraControl
)

# 安装所有可执行文件
install(TARGETS
  ImgPublisher
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs cv_bridge OpenCV)
ament_package()
