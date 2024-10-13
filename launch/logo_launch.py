from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            parameters=[{'background_b': 255, 'background_g': 255, 'background_r': 255}],
            output='screen'
        ),

        # Launch the draw_logo node
        Node(
            package='nov_fij_draw_logo',  # Your package name
            executable='draw_logo',       # Your executable name
            name='draw_logo_node',        # Optional node name
            output='screen'
        ),
    ])
