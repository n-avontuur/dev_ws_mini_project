# all_nodes_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    publisher_node = Node(
            package='mini_project',
            executable='objectdetection_reciever_node',
            name='objectdetection_reciever_node',
            output='screen'
        )

    ld.add_action(planner_node)

    return ld
