import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller_sim.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator_sim.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_sim.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recoveries_sim.yaml')
    filters_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'filters_sim.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_config_dir = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'pathplanning.rviz')
    
    return LaunchDescription([     
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', 'diffbot_base_controller/cmd_vel_unstamped')]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml, {'use_sim_time': use_sim_time}]
        ),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', 'diffbot_base_controller/cmd_vel_unstamped')],
            output='screen'
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml]),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator']}]
        ),

        DeclareLaunchArgument(
            "rviz_config_dir", 
            default_value="", 
            description="TRviz"
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])