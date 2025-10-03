# 清理之前的编译产物
rm -rf build/ install/ log/

# 重新编译
colcon build --packages-select rmv_task04

# 加载环境变量
source install/setup.bash

# 启动节点
ros2 launch rmv_task04 launch.py


# 启动时指定相机IP、曝光时间、像素格式
#ros2 launch rmv_task04 launch.py \
#  camera_ip:=192.168.1.100 \
#  exposure_time:=2000.0 \
#  pixel_format:=rgb8 \
#  frame_rate:=25.0
