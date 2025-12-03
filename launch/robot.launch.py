from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    pkg_path = get_package_share_directory("warehouse_bot")
    world_path = os.path.join(pkg_path, "worlds", "arena.world")
    model_path = os.path.expanduser("/home/aech/ros2_ws/src/warehouse_bot/models/my_robot/model.sdf")

    return LaunchDescription([

        DeclareLaunchArgument("x", default_value="2.7"),
        DeclareLaunchArgument("y", default_value="1.2"),
        DeclareLaunchArgument("z", default_value="0.1"),

        ExecuteProcess(
            cmd=[
                "gazebo",
                "--verbose",
                world_path,
                "-s", "libgazebo_ros_init.so",
                "-s", "libgazebo_ros_factory.so",
                "-s", "libgazebo_ros_state.so"
            ],
            output="screen"
        ),

        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            output="screen",
            arguments=[
        "-entity", "my_robot",
        "-file", model_path,
        "-x", LaunchConfiguration("x"),
        "-y", LaunchConfiguration("y"),
        "-z", LaunchConfiguration("z"),
        "-R", "0.0",  
        "-P", "0.0",      
        "-Y", "-1.5708"       
    ],
        ),
    ])
