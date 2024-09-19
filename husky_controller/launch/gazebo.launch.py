# This launch file launches gazebo/bot/world/lidar/rviz2
# 
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    #pkg path for mapping and localization, path planning 
    pkg_path = os.path.join(get_package_share_directory('husky_controller'))

    #path for urdf, husky urdf
    urdf_file_path = os.path.join(FindPackageShare("husky_description"),"urdf","husky.urdf.xacro")

    #rviz file path , visualize using urdf_tutorial node and save file in your own file  
    rviz_config_path = os.path.join(FindPackageShare("husky_controller"),"rviz","husky.rviz")


    # world file path 
    world_file_path = LaunchConfiguration('world', default=os.path.join(
        get_package_share_directory('husky_controller'),
        'world', 'small_house.world'
    ))

    gazebo_node = Node(
            package='gazebo_ros', 
            executable='gzserver' 'gzclient',
            name='gzserver' 'gzclient',
            output='screen',
            arguments=[LaunchConfiguration('world')],
            parameters=[{
                'use_sim_time': True
            }]
        )
 
    node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }],
            remappings=[
                ('/robot_description', 'robot_description')  
            ]
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d',urdf_file_path ],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output = 'screen',
        arguments=[
           '-topic','robot_description',
           '-entity','husky'
        ],
        parameters=[{
           'use_sim_time':True
        }]
        
    )

    return LaunchDescription(
        [
            gazebo_node, 
            node_rviz,
            node_robot_state_publisher,
            spawn_entity,
        ]
    )