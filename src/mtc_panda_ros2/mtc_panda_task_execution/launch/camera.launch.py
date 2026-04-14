import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
        launch_arguments={"pointcloud.enable": "true",
                          "rgb_camera.profile": "640x480x30",
                          "depth_module.profile": "640x480x30"}.items()
    )

    camera_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="camera_tf_publisher", output="log",
        arguments=['0.10503262', '0.07446604', '0.07179419', '0.4953364', '0.20476633', '0.79364474', '-0.28782048', 'panda_link8', 'camera_link'],)

    return LaunchDescription([camera_node, camera_tf])
