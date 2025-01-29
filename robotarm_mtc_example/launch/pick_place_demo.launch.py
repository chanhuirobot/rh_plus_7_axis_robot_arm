from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rh_plus_7_axis_robot_arm").to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        package="robotarm_mtc_example",
        executable="robotarm_mtc_example",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])
