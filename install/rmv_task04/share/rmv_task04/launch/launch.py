from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 1. 声明启动参数（支持命令行覆盖）
    declare_camera_ip_arg = DeclareLaunchArgument(
        'camera_ip',
        default_value='',
        description='相机IP地址（空则自动枚举）'
    )
    declare_camera_serial_arg = DeclareLaunchArgument(
        'camera_serial',
        default_value='',
        description='相机序列号（空则自动枚举）'
    )
    declare_image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/image_raw',
        description='图像发布话题名'
    )
    declare_exposure_time_arg = DeclareLaunchArgument(
        'exposure_time',
        default_value='40000.0',
        description='曝光时间（μs）'
    )
    declare_gain_arg = DeclareLaunchArgument(
        'gain',
        default_value='5.0',
        description='增益'
    )
    declare_frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='20.0',
        description='帧率（fps）'
    )
    declare_pixel_format_arg = DeclareLaunchArgument(
        'pixel_format',
        default_value='rgb8',
        description='像素格式（mono8/rgb8）'
    )

    # 2. 配置节点参数（从启动参数或配置文件读取）
    config_file_path = PathJoinSubstitution(
        [FindPackageShare('rmv_task04'), 'config', 'camera_params.yaml']
    )

    hk_camera_node = Node(
        package='rmv_task04',
        executable='ImgPublisher',
        name='ImgPublisher',
        output='screen',
        parameters=[
            {
                'camera_ip': LaunchConfiguration('camera_ip'),
                'camera_serial': LaunchConfiguration('camera_serial'),
                'image_topic': LaunchConfiguration('image_topic'),
                'exposure_time': LaunchConfiguration('exposure_time'),
                'gain': LaunchConfiguration('gain'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'pixel_format': LaunchConfiguration('pixel_format')
            }
        ]
    )

    # 3. 组装Launch描述
    return LaunchDescription([
        declare_camera_ip_arg,
        declare_camera_serial_arg,
        declare_image_topic_arg,
        declare_exposure_time_arg,
        declare_gain_arg,
        declare_frame_rate_arg,
        declare_pixel_format_arg,
        hk_camera_node
    ])
