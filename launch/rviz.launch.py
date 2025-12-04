from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('warehouse_bot')

    urdf_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'display.rviz')

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
               "robot_description": os.popen(f"xacro {urdf_path}").read()
            }],
            output='screen'
        ),

        Node(
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui"
),


        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
