from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'base_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser1_link',
            arguments=['0.628', '-0.401', '0.12', '-0.76794487088', '0.0', '3.14159', 'base_link', 'front_scan'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser2_link',
            arguments=['-0.628', '0.401', '0.12', '2.3561944902', '0.0', '3.14159', 'base_link', 'back_scan'],
        ),

        
        # Uncomment if needed
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_camera_link',
        #     arguments=['0.45', '0.0', '0.12', '0.0', '0.0', '0.0', 'base_link', 'cam1_link'],
        # ),
    ])
