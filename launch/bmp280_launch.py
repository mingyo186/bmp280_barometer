"""Launch file for bmp280_barometer package."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('bmp280_barometer')
    default_params = os.path.join(pkg_dir, 'config', 'bmp280_params.yaml')

    return LaunchDescription([
        # ── Launch arguments (override on CLI) ────────
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the parameter YAML file',
        ),

        # ── BMP280 Barometer Node ────────────────────
        Node(
            package='bmp280_barometer',
            executable='bmp280_node.py',
            name='bmp280_barometer_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[
                ('bmp280/pressure', 'bmp280/pressure'),
                ('bmp280/temperature', 'bmp280/temperature'),
            ],
        ),
    ])
