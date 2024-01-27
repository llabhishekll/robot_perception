from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # launch parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # moveit config
    moveit_sim_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()
    moveit_real_config = MoveItConfigsBuilder("name", package_name="real_moveit_config").to_moveit_configs()

    # return launch
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_sim_time", default_value="True"),
            Node(
                package="simple_grasping",
                executable="basic_grasping_perception_node",
                name="basic_grasping_perception_node",
                output="screen",
                parameters=[{'debug_topics': True}],
                condition=IfCondition(use_sim_time),
            ),
            Node(
                package="simple_grasping",
                executable="basic_grasping_perception_node",
                name="basic_grasping_perception_node",
                output="screen",
                remappings=[('/wrist_rgbd_depth_sensor/points', '/camera/depth/color/points')],
                parameters=[{'debug_topics': True}],
                condition=UnlessCondition(use_sim_time),
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="moveit2_scripts",
                        executable="pick_and_place_sim",
                        name="pick_and_place_sim_node",
                        output="screen",
                        parameters=[
                            moveit_sim_config.to_dict(),
                            {"trajectory_execution.allowed_execution_duration_scaling": 2.0},
                            {"publish_robot_description_semantic": True},
                            {"use_sim_time": True},
                        ],
                        condition=IfCondition(use_sim_time),
                    ),
                    Node(
                        package="moveit2_scripts",
                        executable="pick_and_place_real",
                        name="pick_and_place_real_node",
                        output="screen",
                        parameters=[
                            moveit_real_config.to_dict(),
                            {"trajectory_execution.allowed_execution_duration_scaling": 5.0},
                            {"publish_robot_description_semantic": True},
                            {"use_sim_time": False},
                        ],
                        condition=UnlessCondition(use_sim_time),
                    )
                ]
            )
        ]
    )
