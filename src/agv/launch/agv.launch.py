from launch import LaunchDescription
import launch.actions
import launch.substitutions
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agv', executable='range', output='screen'),
        Node(
            package='agv', executable='movement', output='screen'),
        Node(
            package='agv', executable='clamp', output='screen'),
        Node(
            package='agv', executable='led', output='screen'),
        Node(
            package='controller', executable='controller', output='screen'),
        Node(
            package='raspicam2', executable='raspicam2_node', output='screen',
            parameters=[{'fps':20}]),
        Node(
            package='object_detection', executable='object_detection', output='screen'),
        Node(
            package='gps', executable='gps', output='screen'),
    ])