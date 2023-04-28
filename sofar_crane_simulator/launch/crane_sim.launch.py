from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sofar_crane_simulator',
            executable='crane_sim_node',
            name='crane_sim_node',
        ),
        Node(
            package='sofar_crane_simulator',
            executable='robot_logic_node',
            name='robot_logic_node',
        ),
        Node(
            package='sofar_crane_simulator',
            executable='motor_x_node',
            name='motor_x_node',
        ),
        Node(
            package='sofar_crane_simulator',
            executable='motor_y_node',
            name='motor_y_node',
        ),
    ])