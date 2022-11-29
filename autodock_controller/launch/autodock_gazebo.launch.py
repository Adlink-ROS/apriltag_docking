import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    tag_launch_dir = os.path.join(get_package_share_directory('apriltag_ros'), 'launch')
    config = os.path.join(get_package_share_directory('apriltag_docking'), 'param', 'neuronbot.yaml') 
    rviz_config_dir = os.path.join(get_package_share_directory('apriltag_docking'), 'rviz', 'apriltag.rviz')
    
    open_rviz = LaunchConfiguration('open_rviz', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    image_topic = LaunchConfiguration('image_topic', default="image_raw")
    camera_name = LaunchConfiguration('camera_name', default="/camera_color_frame")

    arg_open_rviz = DeclareLaunchArgument(
        'open_rviz',
        default_value='false',
        description='Launch Rviz?')

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
            
    bringup_cmd_group = GroupAction([
        Node(
            package='apriltag_docking',
            executable='controller',
            name='autodock_controller',
            output='screen',
            parameters = [config]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration("open_rviz"))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tag_launch_dir,'tag_gazebo.launch.py')),
            launch_arguments={'image_topic': image_topic,
                              'camera_name': camera_name}.items())
    ])

    ld = LaunchDescription()
    ld.add_action(arg_open_rviz)
    ld.add_action(arg_use_sim_time)
    ld.add_action(bringup_cmd_group)

    return ld
