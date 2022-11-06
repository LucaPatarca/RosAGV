import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_dolly_gazebo = get_package_share_directory('dolly_gazebo')
    pkg_navigation_ros = get_package_share_directory('nav2_bringup')
    pkg_slam_ros = get_package_share_directory('slam_toolbox')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation_ros, 'launch', 'navigation_launch.py')
        )
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_ros, 'launch', 'online_async_launch.py')
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_dolly_gazebo, 'rviz', 'dolly_gazebo.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'dolly', '-topic', 'robot_description'],
        output='screen'
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_dolly_gazebo, 'models/dolly/config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument(name='model', default_value=[os.path.join(pkg_dolly_gazebo, 'models', 'dolly', 'model_no_macro.urdf'), ''],
                                            description='Absolute path to robot urdf file'),
        navigation,
        slam,
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', os.path.join(pkg_dolly_gazebo, 'worlds', 'dolly_city.world')], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        robot_localization_node,
        rviz,
    ])
