import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

config = os.path.join(get_package_share_directory('apriltag_docking'), 'param', 'neuronbot.yaml') 

def generate_launch_description():
    params_file = LaunchConfiguration('params_file', default = config)

    return LaunchDescription([
                
            Node(
                package='apriltag_docking',
                executable='controller',
                name='autodock_controller',
                output='screen',
                parameters = [params_file])])
