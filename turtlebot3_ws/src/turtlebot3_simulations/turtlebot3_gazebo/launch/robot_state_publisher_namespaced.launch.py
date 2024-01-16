#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PythonExpression
from launch import LaunchContext
from launch.substitutions import Command


def generate_launch_description():

    context = LaunchContext()

    # Namespace
    turtlebot_ns = LaunchConfiguration('namespace')
    # Robot model
    robot_model = LaunchConfiguration('robot_model')
    # Robot position
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    # Sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # urdf_file_name = 'turtlebot3_' + robot_model + '.urdf'
    urdf_file_name = 'turtlebot3_' + 'waffle' + '.urdf' #This overwrites the robot_model argument but I get an error otherwise...

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)
    
    robot_desc = Command(['xacro ', urdf, ' frame_prefix:=', turtlebot_ns, ' topic_prefix:=', turtlebot_ns])


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('namespace', default_value='', description='Namespace for robot'),
        DeclareLaunchArgument('robot_model', default_value='burger', description='Robot model type'),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='Robot initial X pose'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Robot initial Y pose'),

        Node(
            namespace=turtlebot_ns,
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': robot_desc,
                         'frame_prefix': PythonExpression(["'", LaunchConfiguration('namespace'), "/'"])}],
            arguments=[urdf]
        ),
    ])
