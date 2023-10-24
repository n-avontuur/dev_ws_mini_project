# all_nodes_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    publisher_node = Node(
            package='mini_project',
            executable='objectdetection_sender_node',
            name='objectdetection_sender_node',
            output='screen'
        )

    subscriber_node =    Node(
            package='mini_project',
            executable='objectdetection_reciever_node',
            name='objectdetection_reciever_node',
            output='screen'
        )

    movement_node =   Node(
            package='mini_project',
            executable='movement_node',
            name='movement_node',
            output='screen'
        )

    planner_node =   Node(
            package='mini_project',
            executable='planner_node',
            name='planner_node',
            output='screen'
        )
    
    ld.add_action(publisher_node)
    ld.add_action(subscriber_node)
    ld.add_action(movement_node)
    ld.add_action(planner_node)

    return ld
