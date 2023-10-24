import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#launch argument:
#declare_arg1 = DeclareLaunchArgument('param1', default_value='default_value')

vision_sender_node = Node(
    package='mini_project',
    executable='vision_sender_node',
    name='vision_sender_node',
)

vision_receiver_node = Node(
    package='mini_project',
    executable='vision_receiver_node',
    name='vision_receiver_node',
    output='screen'
)

vision_receiver_node = Node(
    package='mini_project',
    executable='vision_receiver_node',
    name='vision_receiver_node',
    output='screen'
)
