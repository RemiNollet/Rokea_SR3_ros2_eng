from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare configurable parameters
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.2.160',
            description='Robot Controller IP Address'
        ),
        DeclareLaunchArgument(
            'local_ip',
            default_value='192.168.2.100',
            description='Local computer IP address'
        ),
        
        # Startrokae_driverNode
        Node(
            package='rokae_hardware',
            executable='rokae_driver',
            name='rokae_driver',
            output='screen',
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'local_ip': LaunchConfiguration('local_ip'),
            }],
            # You can set remapping and so on
            remappings=[
                # If you need to remap topics, you can add them here
                # ('/rokae_driver/joint_states', '/joint_states'),
            ],
            # You can set the node namespace
            # namespace='robot1',
        )
    ])