import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joint Controllers (Motor Nodes)
        Node(
            package='focus_one_robot',
            executable='joint_1_controller',
            name='joint_1_controller_node',
            output='screen'
        ),
        Node(
            package='focus_one_robot',
            executable='joint_2_controller',
            name='joint_2_controller_node',
            output='screen'
        ),
        Node(
            package='focus_one_robot',
            executable='joint_3_controller',
            name='joint_3_controller_node',
            output='screen'
        ),
        Node(
            package='focus_one_robot',
            executable='joint_4_controller',
            name='joint_4_controller_node',
            output='screen'
        ),
        Node(
            package='focus_one_robot',
            executable='joint_5_controller',
            name='joint_5_controller_node',
            output='screen'
        ),
        Node(
            package='focus_one_robot',
            executable='joint_6_controller',
            name='joint_6_controller_node',
            output='screen'
        ),

        # Feedback Monitor Node
        Node(
            package='focus_one_robot',
            executable='feedback_monitor',
            name='feedback_monitor_node',
            output='screen'
        ),

        # MPC Controller Node
        Node(
            package='focus_one_robot',
            executable='mpc_controller',
            name='mpc_controller_node',
            output='screen'
        ),

        # Manipulator Controller Node
        Node(
            package='focus_one_robot',
            executable='manipulator_controller',
            name='manipulator_controller_node',
            output='screen'
        ),

        # Kinematics Controller Node
        Node(
            package='focus_one_robot',
            executable='kinematics_controller',
            name='kinematics_controller_node',
            output='screen'
        ),

        # Piston Controller Node
        Node(
            package='focus_one_robot',
            executable='piston_controller',
            name='piston_controller_node',
            output='screen'
        ),
    ])
