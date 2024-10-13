from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            parameters=[{'background_b': 255, 'background_g': 255, 'background_r': 255}],
            output='screen'
        ),
        Node(
            package='nov_fij_draw_logo',  
            executable='draw_logo', 
            name='draw_logo',
            output='screen'
        ),
    ])
