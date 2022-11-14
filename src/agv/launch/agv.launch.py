from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    controller_config = LaunchConfiguration('controller')
    object_detection_config = LaunchConfiguration('object_detection')
    gps = LaunchConfiguration('gps')
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='True'),
        DeclareLaunchArgument('object_detection', default_value='True'),
        DeclareLaunchArgument('gps', default_value='True'),
        Node(
            package='gpio', executable='range', output='screen'),
        Node(
            package='gpio', executable='movement', output='screen'),
        Node(
            package='gpio', executable='clamp', output='screen'),
        Node(
            package='gpio', executable='led', output='screen'),
        Node(
            package='controller', executable='controller', output='screen', condition=IfCondition(controller_config)),
        Node(
            package='raspicam2', executable='raspicam2_node', output='screen',
            parameters=[{'fps':10}]),
        Node(
            package='object_detection', executable='object_detection', output='screen', condition=IfCondition(object_detection_config)),
        Node(
            package='gps', executable='gps', output='screen', condition=IfCondition(gps)),
    ])