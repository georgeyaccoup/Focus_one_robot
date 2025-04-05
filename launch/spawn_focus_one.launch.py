from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    urdf_file_path = os.path.join(
        "/home/george/new_ws/src/focus_one_robot/urdf", "focus_one.urdf"
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                "/opt/ros/humble/share/gazebo_ros/launch", "gazebo.launch.py"
            )
        )
    )

    return LaunchDescription([
        gazebo_launch,  # Start Gazebo first

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": open(urdf_file_path).read()}],
            output="screen"
        ),

        ExecuteProcess(
            cmd=[
                "ros2", "run", "gazebo_ros", "spawn_entity.py",
                "-entity", "focus_one_robot",
                "-topic", "/robot_description"
            ],
            output="screen"
        )
    ])

