# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# /* Author: Gary Liu */

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    depthimage_to_laserscan_dir = get_package_share_directory('depthimage_to_laserscan')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    depth_image = LaunchConfiguration('depth_image')
    depth_info = LaunchConfiguration('depth_info')
    scan_time = LaunchConfiguration('scan_time')
    range_min = LaunchConfiguration('range_min')
    range_max = LaunchConfiguration('range_max')
    scan_height = LaunchConfiguration('scan_height')
    output_frame = LaunchConfiguration('output_frame')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'scan_time': scan_time,
        'range_min': range_min,
        'range_max': range_max,
        'scan_height': scan_height,
        'output_frame': output_frame}

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(depthimage_to_laserscan_dir, 'cfg', 'params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'depth_image',
            default_value='/camera/depth/image_rect_raw',
            description='Topic providing depth camera images'),

        DeclareLaunchArgument(
            'depth_info',
            default_value='/camera/depth/camera_info',
            description='Topic providing depth camera info'),

        DeclareLaunchArgument(
            'scan_time', default_value='0.033',
            description='Interval to publish laserscan information. Defaults to 0.033 seconds.'),

        DeclareLaunchArgument(
            'range_min', default_value='0.45',
            description='The minimum distance in meters a projected point should be. Points closer than this are discarded. Defaults to 0.45 meters.'),
        
        DeclareLaunchArgument(
            'range_max', default_value='10.0',
            description='The maximum distance in meters a projected point should be. Points further than this are discarded. Defaults to 10.0 meters.'),

        DeclareLaunchArgument(
            'scan_height', default_value='1',
            description='The row from the depth image to use for the laser projection. Defaults to 1.'),

        DeclareLaunchArgument(
            'output_frame', default_value='camera_depth_frame',
            description='The frame id to publish in the LaserScan message. Defaults to "camera_depth_frame".'),

        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            output='screen',
            parameters=[params_file],
            remappings=[('depth', depth_image),
                        ('depth_camera_info', depth_info)])
    ])
