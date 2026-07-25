import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
                                                 default=os.path.join(
                                                     get_package_share_directory('turtlebot4_cartographer'),
                                                     'config'))
    
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                default='turtlebot4_2d.lua')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                      '-configuration_basename', configuration_basename]
                      
remappings=[
    ('scan', '/bot/scan'),
    ('odom', '/bot/odom'),
    ('imu', '/bot/imu'),
    ('tf', '/bot/tf'),
    ]
),
        
        # Node(
        #     package='cartographer_ros',
        #     executable='occupancy_grid_node',
        #     name='occupancy_grid_node',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}]),
    ])
