"""
@copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026.
           Licensed under the Apache License, Version 2.0 (see LICENSE).

@file      jetson_v4l2_camera.launch.py
@brief     Launch one jetson_v4l2_camera_node with a parameter file and per-camera overrides

@date      2026-10-26 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('jetson_v4l2_camera'), 'config', 'jetson_v4l2_camera.yaml')

    params = [
        LaunchConfiguration('params_file'),
        {
            'camera_name': ParameterValue(LaunchConfiguration('camera_name'), value_type=str),
            'device': ParameterValue(LaunchConfiguration('device'), value_type=str),
            'pipeline': ParameterValue(LaunchConfiguration('pipeline'), value_type=str),
        },
    ]

    def camera_node(i_respawn, i_condition):
        return Node(
            package='jetson_v4l2_camera',
            executable='jetson_v4l2_camera_node',
            name='jetson_v4l2_camera',
            namespace=LaunchConfiguration('namespace'),
            parameters=params,
            # The node exits non-zero on a pipeline error (device unplugged etc.) so it can be restarted
            respawn=i_respawn,
            respawn_delay=1.0,
            output='screen',
            condition=i_condition,
        )

    respawn = LaunchConfiguration('respawn')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params, description='Node parameter file'),
        DeclareLaunchArgument('camera_name', default_value='camera',
                              description='Camera name (camera_info_manager name, default frame_id prefix)'),
        DeclareLaunchArgument('device', default_value='/dev/video0', description='V4L2 device'),
        DeclareLaunchArgument('namespace', default_value='camera', description='Topic namespace'),
        DeclareLaunchArgument('pipeline', default_value='',
                              description='Full GStreamer pipeline override, must end with "appsink name=sink"'),
        DeclareLaunchArgument('respawn', default_value='true', description='Restart the node when it exits'),
        camera_node(True, IfCondition(respawn)),
        camera_node(False, UnlessCondition(respawn)),
    ])
