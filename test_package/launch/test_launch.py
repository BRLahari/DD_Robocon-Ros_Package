from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='test_package').find('test_package')
    sdf_file_path = os.path.join(pkg_share, 'sdf', 'robot_model.sdf')
    
    ignition_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', sdf_file_path],
        output='screen'
    )

    return LaunchDescription([
        ignition_gazebo,
        Node(
            package='ros_ign_gazebo',
            executable='create',
            name='spawn_robot',
            arguments=['-name', 'omniwheel_robot', '-file', sdf_file_path],
            output='screen'
        ),
    ])
