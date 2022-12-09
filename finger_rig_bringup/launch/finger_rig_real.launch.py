import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            Shutdown)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

def generate_launch_description():
    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
                                     'panda_arm.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=true',
         ' robot_ip:=', "panda0.robot", ' use_fake_hardware:=', "false",
         ' fake_sensor_commands:=', "false"])

    robot_description = {'robot_description': robot_description_config}

    franka_semantic_xacro_file = os.path.join(get_package_share_directory('franka_moveit_config'),
                                              'srdf',
                                              'panda_arm.srdf.xacro')
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=true']
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    # RViz
    rviz_base = os.path.join(get_package_share_directory('finger_rig_description'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'finger_rig.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    # Static transform broadcaster
    static_tf_node = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = ["0", "0", "0", "0", "0", "0", "world", "panda_link0"]
    )

    camera_launch_file = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("avt_vimba_camera"),
                "launch/mono_camera.launch.xml",
            )
        )
    )

    # Nodes
    environment_node = Node(
        package="finger_rig_control",
        executable="environment",
        parameters=[
            robot_description,
            robot_description_semantic,
        ]
    )

    motion_control_node = Node(
        package="finger_rig_control",
        executable="motion_control",
        output="log",
        parameters=[
            robot_description,
            robot_description_semantic,
        ]
    )

    vision_node = Node(
        package="finger_rig_control",
        executable="vision"
    )

    return LaunchDescription(
        [rviz_node,
         static_tf_node,
         camera_launch_file,
        #  environment_node,
         motion_control_node,
         vision_node
         ]
    )