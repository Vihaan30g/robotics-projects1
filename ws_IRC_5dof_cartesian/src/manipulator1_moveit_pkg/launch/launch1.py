
# THIS FILE WAS SNE BY SHUBH ANNA, NOT COMPLETE













from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path

def generate_launch_description():

    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")


    #
    moveit_config = (
    MoveItConfigsBuilder("manipulator1", package_name="manipulator1_moveit_pkg")
        .robot_description(file_path="config/manipulator1.urdf.xacro")
        .robot_description_semantic(file_path="config/manipulator1.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description = False,
            publish_robot_description_semantic = True,
            publish_planning_scene = True,
        )
        .pilz_cartesian_limits(Path("config") / "pilz_cartesian_limits.yaml")
        .joint_limits(Path("config") / "joint_limits.yaml")
        .trajectory_execution(Path("config") / "moveit_controllers.yaml")
        .to_moveit_configs()
    )



    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2_moveit",
        output = "log",
        arguments = ["-d", rviz_config],
        parameters = [
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time" : use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            #default_value = PathJoinSubstitution([FindPackageShare("manipulator1_moveit_pkg"), "rviz", "moveit.rviz"]),
            description = "Rviz config file (absolute path) to use when launching rviz."
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value = "false",
            description = "Using or not time from simulation"
        ),
        RegisterEventHandler(
            OnProcessExit(
                #target_action = wait_robot_description,
                on_exit = [
                    rviz_node
                ]
            )
        ),
        rviz_node,
        # wait_robot_description
    ])