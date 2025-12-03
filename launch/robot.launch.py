from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    model_path = os.path.expanduser("/home/aech/ros2_ws/src/warehouse_bot/models/my_robot/model.sdf")

    return LaunchDescription([

        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.1"),

        # Start Gazebo Classic + IMPORTANT ROS PLUGINS
        ExecuteProcess(
            cmd=[
                "gazebo",
                "--verbose",
                "/usr/share/gazebo-11/worlds/empty.world",
                "-s", "libgazebo_ros_init.so",
                "-s", "libgazebo_ros_factory.so",
                "-s", "libgazebo_ros_state.so"
            ],
            output="screen"
        ),

        # ROS2: Spawn robot using spawn_entity.py
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            output="screen",
            arguments=[
                "-entity", "my_robot",
                "-file", model_path,
                "-x", LaunchConfiguration("x"),
                "-y", LaunchConfiguration("y"),
                "-z", LaunchConfiguration("z")
            ],
        ),
    ])
