from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    goto_target_node = Node(
        package    = "project_03",
        executable = "goto_target_node"
    )

    target_sequence_node = Node(
        package    = "project_03",
        executable = "target_sequence_node"
    )

    ld.add_action(goto_target_node)
    ld.add_action(target_sequence_node)

    return ld
