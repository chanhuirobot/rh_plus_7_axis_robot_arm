from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rh_plus_7_axis_robot_arm", package_name="rh_plus_7_axis_robot_arm_moveit_config").to_moveit_configs()
    return generate_move_group_launch(moveit_config)
