from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="robot_tf_publisher",
            output="log",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="cell_tf_publisher",
            output="log",
            arguments=['0.52326', '0.070703', '0.0', '0.0', '0.0', '-0.38269', '0.923877', "panda_link0", "cell"],
        ),
    ])