# all_nodes_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_normallaunch_description():
    run = LaunchDescription()
    
    display_node = Node(
            package='mini_project',
            executable='n_display',
            output='screen'
        )

    vision_node =    Node(
            package='mini_project',
            executable='n_object_detection',
            output='screen'
        )

    planner_node =   Node(
            package='mini_project',
            executable='n_plan_move',
            name='plan_move',
        )
    
    run.add_action(display_node)
    run.add_action(vision_node)
    run.add_action(planner_node)

    return run
