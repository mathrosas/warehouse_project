import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    config_dir = os.path.join(get_package_share_directory('path_planner_server'), 'config')
    
    controller_yaml = [config_dir, '/', PythonExpression(["'controller_sim.yaml' if '", use_sim_time, "'.lower() == 'true' else 'controller_real.yaml'"])]
    bt_navigator_yaml = [config_dir, '/', PythonExpression(["'bt_navigator_sim.yaml' if '", use_sim_time, "'.lower() == 'true' else 'bt_navigator_real.yaml'"])]
    planner_yaml = [config_dir, '/', PythonExpression(["'planner_sim.yaml' if '", use_sim_time, "'.lower() == 'true' else 'planner_real.yaml'"])]
    recovery_yaml = [config_dir, '/', PythonExpression(["'recoveries_sim.yaml' if '", use_sim_time, "'.lower() == 'true' else 'recoveries_real.yaml'"])]
    filters_yaml = [config_dir, '/', PythonExpression(["'filters_sim.yaml' if '", use_sim_time, "'.lower() == 'true' else 'filters_real.yaml'"])]

    rviz_config_dir = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'pathplanning.rviz')

    cmd_vel = PythonExpression(["'diffbot_base_controller/cmd_vel_unstamped' if '", use_sim_time, "'.lower() == 'true' else '/cmd_vel'"])
    
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
            remappings=[('/cmd_vel', cmd_vel)]
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
            remappings=[('/cmd_vel', cmd_vel)],
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