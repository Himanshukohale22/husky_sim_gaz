import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ballie_bot_description').find('ballie_bot_description')

    default_model_path = os.path.join(pkg_share, 'urdf/ballie_bot.urdf.xacro')

    default_rviz_config_path = os.path.join(pkg_share, 'rviz/ballie.rviz')

    # gazebo_process = launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # arguments=[default_model_path],

        parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))

    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    spawn_entity = launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity','ballie_bot',
                '-topic','robot_description'
            ],
            output='screen'
    )

    gazebo_node =launch_ros.actions.Node(
            package='ballie_bot_description',
            executable='your_robot_node',
            name='ballie_bot',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),

        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        # gazebo_process,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        spawn_entity
    ])

