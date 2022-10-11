from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ld.add_action(vision_node)

    finger_rig_desc_dir = get_package_share_directory('finger_rig_description')
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(finger_rig_desc_dir, 'rviz', 'finger_rig.rviz')]),
        Node(package="finger_rig_control", executable="vision")
    ])
