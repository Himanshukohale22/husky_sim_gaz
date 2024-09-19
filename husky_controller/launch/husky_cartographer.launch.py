from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

        gazebo_get_dir= get_package_share_directory('husky_gazebo')
        
        gazebo_husky = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(gazebo_get_dir),'launch','husky_playpen.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
        )

        cartographer_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': False  # Set to True if using Gazebo or a simulation
            }],
            arguments=[
                'get_dir', '/path/to/your/config',
                '', ''
            ],
        ),
        
        cartographer_gz_node = Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{
                'resolution': 0.05  # Resolution of the occupancy grid map
            }],
        ),
        tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),
        # Optionally add RViz for visualization

        rviz_config_node =Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d',],
        ),

        return LaunchDescription(cartographer_node,cartographer_gz_node,tf_node,rviz_config_node,gazebo_husky)