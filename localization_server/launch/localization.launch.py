import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    map_file_name = LaunchConfiguration('map_file', default='warehouse_map_sim.yaml')
    map_dir = get_package_share_directory('map_server') + '/config'
    map_file = [map_dir, '/', map_file_name]

    amcl_dir = get_package_share_directory('localization_server') + '/config'
    amcl_yaml = PythonExpression(["'amcl_config_sim.yaml' if '", map_file_name,"'.lower().endswith('sim.yaml') else 'amcl_config_real.yaml'"])
    amcl_file = [amcl_dir, '/', amcl_yaml]

    use_sim_time = PythonExpression(["'true' if '", map_file_name, "'.lower().endswith('sim.yaml') else 'false'"])

    attach_service = PythonExpression(["'attach_service_server' if '", map_file_name, "'.lower().endswith('sim.yaml') else 'approach_service_server'"])
    
    rviz_config_dir = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'localization_config.rviz')

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'map_file',
            default_value='warehouse_map_sim.yaml',
            description='Name of the map YAML in <pkg>/config/'
        ),
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename':map_file}]
        ),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_file]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
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
            output='screen'),

        Node(
            package='attach_service',
            executable=attach_service,
            name=attach_service,
            output='screen')
    ])
