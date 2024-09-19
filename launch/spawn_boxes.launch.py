import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('lidar_pcd_simulator').find('lidar_pcd_simulator')
    xacro_path = os.path.join(pkg_share, 'urdf', 'boxes.xacro')
    urdf_path = os.path.join(pkg_share, 'urdf', 'boxes.urdf')

    # Generate nodes for spawning boxes
    spawn_boxes = [
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'box',
                '-file', urdf_path,
                '-x', '0', '-y', '0', '-z', '0.01'
            ],
            output='screen'
        )
    ]

    return LaunchDescription([
        # Convert Xacro to URDF
        ExecuteProcess(
            cmd=[
                'xacro', xacro_path, '-o', urdf_path,
            ],
            output='screen'
        ),
        # Spawn boxes
        *spawn_boxes
    ])
