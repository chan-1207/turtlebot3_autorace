from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the mission type argument
    mission_arg = DeclareLaunchArgument(
        'mission',
        default_value='intersection',
        description='Mission type [intersection, construction, parking, level_crossing, tunnel]'
    )

    # Use the mission argument to determine the node executable
    mission = LaunchConfiguration('mission')

    # Define the node for detecting traffic signs
    detect_sign_node = Node(
        package='turtlebot3_autorace_detect',
        executable=['detect_', mission, '_sign.py'],
        name=['detect_', mission, '_sign'],
        output='screen',
        remappings=[
            ('/detect/image_input', '/camera/image_compensated'), # image_compensated
            ('/detect/image_input/compressed', '/camera/image_compensated/compressed'),
            ('/detect/image_output', '/detect/image_traffic_sign'),
            ('/detect/image_output/compressed', '/detect/image_traffic_sign/compressed'),
        ]
    )

    # Launch description including arguments and the node
    return LaunchDescription([
        mission_arg,
        detect_sign_node,
    ])
