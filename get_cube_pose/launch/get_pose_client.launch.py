from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # launch parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # package launch description
    path_root = Path(get_package_share_directory("get_cube_pose"))
    path_rviz = path_root / "rviz" / "perception.config.rviz"

    # return launch
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_sim_time", default_value="True"),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=["-d", path_rviz.as_posix()],
            ),
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
                        package="get_cube_pose",
                        executable="get_cube_pose_node",
                        name="get_cube_pose_node",
                        output="screen",
                    ),
                ]
            )
        ]
    )
