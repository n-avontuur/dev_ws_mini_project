# all_nodes_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    display_node = Node(
            package='mini_project',
            executable='display',
            output='screen'
        )

    vision_node =    Node(
            package='mini_project',
            executable='object_detection',
            output='screen'
        )

    planner_node =   Node(
            package='mini_project',
            executable='plan_move',
            name='plan_move',
        )
    
    ld.add_action(display_node)
    ld.add_action(vision_node)
    ld.add_action(planner_node)
    return ld
