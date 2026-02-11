# Copyright 2025 The bmp280_barometer Authors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""Launch file for bmp280_barometer package."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for BMP280 barometer node."""
    pkg_dir = get_package_share_directory('bmp280_barometer')
    default_params = os.path.join(pkg_dir, 'config', 'bmp280_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the parameter YAML file',
        ),

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
