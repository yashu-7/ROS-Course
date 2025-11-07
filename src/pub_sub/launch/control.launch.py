from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtle_simulator',
        output='screen'
    )

    turtle_control = Node(
        package='pub_sub',
        executable='control',
        name='controller',
        output='screen'
    )

    return LaunchDescription([
        turtlesim_node,
        turtle_control
    ])