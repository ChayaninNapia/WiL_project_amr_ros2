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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera_link',
            arguments=['0.628', '0.0', '0.16', '1.572', '-0.008', '1.355', 'base_link', 'camera_link'],
        ),       
        # Node(
                #     package='tf2_ros',
                #     executable='static_transform_publisher',
                #     name='base_link_to_camera_link',
                #     arguments=['0.0', '0.0', '0.0', '0.008149', '0.000881', '-0.215383', 'camera_imu_frame', 'camera_imu_tilt'],
                # ),
                
        # old publish without tilt angle
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_camera_link',
        #     arguments=['0.628', '0.0', '0.16', '1.57079633', '0.0', '1.57079633', 'base_link', 'camera_link'],
        # ),                       
    ])

'''
base_link -> camera_tilt
At time 0.0
- Translation: [0.628, 0.000, 0.160]
- Rotation: in Quaternion [0.445, 0.441, 0.553, 0.549]
- Rotation: in RPY (radian) [1.355, -0.008, 1.572]
- Rotation: in RPY (degree) [77.659, -0.467, 90.050]
- Matrix:
 -0.001 -0.214  0.977  0.628
  1.000 -0.008 -0.001  0.000
  0.008  0.977  0.214  0.160
  0.000  0.000  0.000  1.000

'''