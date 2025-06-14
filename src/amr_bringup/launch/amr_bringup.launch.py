import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node                                # ← เพิ่มตรงนี้
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # หา path ของแพ็กเกจ bringup และ sick_safetyscanners2
    pkg_bringup = get_package_share_directory('amr_bringup')
    pkg_sick   = get_package_share_directory('sick_safetyscanners2')
    pkg_merger  = get_package_share_directory('ira_laser_tools')

    # 1) ประกาศ IncludeLaunchDescription สำหรับสอง launch เดิม
    lidar_tf_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sick, 'launch', 'lidar_tf.launch.py')
        )
    )
    front_back_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sick, 'launch', 'front_back_scan.launch.py')
        )
    )
    
    merger_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_merger, 'launch', 'laserscan_multi_merger_launch.py')
        )
    )

    # 2) ประกาศ Node สำหรับ RViz
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'lidar.rviz')
    rviz_action = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )
    
    prototype_driver_node = Node(
    package='roboteq_ros2_driver',      # ชื่อแพ็กเกจ ตาม CMakeLists.txt
    executable='prototype_driver',      # ชื่อ executable ที่คุณสร้าง
    name='prototype_driver',            # node name ในระบบ
    output='screen',
    # ถ้ามีพารามิเตอร์ให้ส่ง ก็ใส่แบบนี้
    # parameters=[{'your_param': 'value'}],
    # ถ้าต้อง remap topic ก็ส่งแบบนี้
    # remappings=[('old_topic', 'new_topic')],
    )  

    # 3) สร้าง LaunchDescription แล้ว add action ตามลำดับ
    ld = LaunchDescription()
    ld.add_action(lidar_tf_action)
    ld.add_action(merger_action)
    ld.add_action(front_back_action)
    ld.add_action(rviz_action)
    ld.add_action(prototype_driver_node)  # เพิ่ม Node ของคุณเข้าไป

    return ld
