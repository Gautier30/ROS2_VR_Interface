import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    map_file = os.path.join(get_package_share_directory('turtlebot3_multi_robot_sim'), 'map', 'workshop.yaml')
    robot_yaml = os.path.join(get_package_share_directory('turtlebot3_multi_robot_sim'), 'param', 'waffle.yaml')
    bt_xml_file = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    lifecycle_nodes_nav = [
        'planner_server',
        'controller_server',
        'bt_navigator',
        'recoveries_server'
    ]

    lifecycle_nodes_loc = [
        'map_server', 
        'amcl'
    ]
    
    return LaunchDescription([
        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[
                {'use_sim_time': True},
                {'yaml_filename':map_file}
            ],
        ),
        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[robot_yaml],
        ),
        # Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[robot_yaml],
        ),
        # Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[robot_yaml],
        ),
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[robot_yaml,
                        #{'default_bt_xml_filename':bt_xml_file}
            ],
        ),
        # Recoveries Server
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[robot_yaml],
        ),
        # Lifecycle Manager Localization
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

        # Lifecycle Manager Navigation
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

    ])