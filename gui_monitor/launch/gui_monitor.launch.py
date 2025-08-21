from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui_monitor',
            executable='gui_monitor',
            name='gui_monitor',
            output='screen',
            emulate_tty=True,
            parameters=[{}],
            arguments=[],
        )
    ])
