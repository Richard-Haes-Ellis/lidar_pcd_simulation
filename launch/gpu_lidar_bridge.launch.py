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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_path = get_package_share_directory('lidar_pcd_simulator')
    path_to_sdf_xacro = os.path.join(pkg_path, 'worlds', 'lidar_world.sdf.xacro')
    path_to_sdf_processed = os.path.join(pkg_path, 'worlds', 'lidar_world_processed.sdf')
    models_path = os.path.join(pkg_path, 'models')
    
    
    os.system(f'xacro {path_to_sdf_xacro} -o {path_to_sdf_processed}')

    # Set environment variables
    set_env_vars = [
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=models_path),
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            # 'gz_args': '' + '~/RGLGazeboPlugin-0.1.2/test_world/sonoma_with_rgl.sdf'
            # 'gz_args': '-r ' + path_to_sdf
            'gz_args': '-r ' + path_to_sdf_processed

        }.items(),
    )

    # RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_path, 'rviz', 'rviz.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                    'lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                    # Bridge the TF frame
                    '/world/lidar_world/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
                   ],
        output='screen'
    )

    # TF publisher for the lidar
    tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '1.1', '0', '0', '0', 'odom', 'lidar_model/lidar_link/lidar'],
        output='screen'
    )

    return LaunchDescription([
        *set_env_vars,
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        rviz,
        tf,
    ])