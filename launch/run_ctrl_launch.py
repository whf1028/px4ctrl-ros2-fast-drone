#-*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('px4ctrl_ros2_fast_drone').find('px4ctrl_ros2_fast_drone')
    # 获取配置文件路径
    config_path = os.path.join(pkg_share, 'config', 'ctrl_param_fpv.yaml')
    # 创建节点
    px4ctrl_node = Node(
        package='px4ctrl_ros2_fast_drone',
        executable='px4ctrl_node',
        name='px4ctrl_node',
        output='screen',
        parameters=[config_path],
        remappings=[
            ('odom', '/imu_propagate'),
            ('cmd', '/position_cmd')
        ]
    )
    # 返回启动描述
    return LaunchDescription([
        px4ctrl_node
    ])