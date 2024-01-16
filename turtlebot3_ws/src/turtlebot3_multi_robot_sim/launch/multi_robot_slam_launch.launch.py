import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    map_file = os.path.join(get_package_share_directory('turtlebot3_multi_robot_sim'), 'map', 'factory.yaml')
    robot1_yaml = os.path.join(get_package_share_directory('turtlebot3_multi_robot_sim'), 'param', 'waffle1.yaml')
    robot2_yaml = os.path.join(get_package_share_directory('turtlebot3_multi_robot_sim'), 'param', 'waffle2.yaml')
    robot3_yaml = os.path.join(get_package_share_directory('turtlebot3_multi_robot_sim'), 'param', 'waffle3.yaml')
    rviz_config_file = os.path.join(get_package_share_directory('turtlebot3_multi_robot_sim'), 'rviz', 'multi_robot_slam.rviz')

    remappings1 = [('/waffle1/map', '/map')]
    remappings2 = [('/waffle2/map', '/map')]
    remappings3 = [('/waffle3/map', '/map')]


    lifecycle_nodes_nav = [
        'waffle1/planner_server',
        'waffle1/controller_server',
        'waffle1/bt_navigator',
        'waffle1/recoveries_server',
        'waffle2/planner_server',
        'waffle2/controller_server',
        'waffle2/bt_navigator',
        'waffle2/recoveries_server',
        'waffle3/planner_server',
        'waffle3/controller_server',
        'waffle3/bt_navigator',
        'waffle3/recoveries_server'
    ]

    lifecycle_nodes_loc = [
        'map_server', 
        'waffle1/amcl',
        'waffle2/amcl', 
        'waffle3/amcl'
    ]

    return LaunchDescription([
        ## Map server common to all robots
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[
                {'use_sim_time': True},
                {'topic_name':'map'},
                {'frame_id':'map'},
                {'yaml_filename':map_file}
            ],
        ),
        
        ### ---------------------------------------------------------------------
        ## Robot 1
        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace='waffle1',
            parameters=[robot1_yaml],
        ),
        # Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace='waffle1',
            output='screen',
            parameters=[robot1_yaml],
        ),
        # Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace='waffle1',
            parameters=[robot1_yaml],
            remappings=remappings1
        ),
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            namespace='waffle1',
            parameters=[robot1_yaml],
        ),
        # Recoveries Server
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            namespace='waffle1',
            output='screen',
            parameters=[robot1_yaml],
        ),


        ### ---------------------------------------------------------------------
        ## Robot 2
        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace='waffle2',
            parameters=[robot2_yaml],
        ),
        # Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace='waffle2',
            output='screen',
            parameters=[robot2_yaml],
        ),
        # Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace='waffle2',
            parameters=[robot2_yaml],
            remappings=remappings2
        ),
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            namespace='waffle2',
            parameters=[robot2_yaml],
        ),
        # Recoveries Server
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            namespace='waffle2',
            output='screen',
            parameters=[robot2_yaml],
        ),


        ### ---------------------------------------------------------------------
        ## Robot 3
        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace='waffle3',
            parameters=[robot3_yaml],
        ),
        # Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace='waffle3',
            output='screen',
            parameters=[robot3_yaml],
        ),
        # Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace='waffle3',
            parameters=[robot3_yaml],
            remappings=remappings3
        ),
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            namespace='waffle3',
            parameters=[robot3_yaml],
        ),
        # Recoveries Server
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            namespace='waffle3',
            output='screen',
            parameters=[robot3_yaml],
        ),


        ### ---------------------------------------------------------------------
        ## Lifecycle Manager Localization
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes_loc},
                        {'bond_timeout':0.4}
            ]
        ),

        ## Lifecycle Manager Navigation
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes_nav},
                        {'bond_timeout':0.4}
            ]
        ),

        
        ### ---------------------------------------------------------------------
        ## Rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(rviz_config_file)]
        )
    ])