from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_base_tf_publisher",
            output="log",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_robot_tf_publisher",
            output="log",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "base", "fr3_link0"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="cell_tf_publisher",
            output="log",
            arguments=['0.52326', '0.070703', '0.0', '0.0', '0.0', '-0.38269', '0.923877', "fr3_link0", "cell"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_tf_publisher",
            output="log",
            arguments=['0.05289', '-0.01869', '-0.06185', '0.71497', '-0.0157', '0.6989', '0.01039', 'fr3_hand_tcp', 'camera_link'],
        ),
    ])
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name="robot_tf_publisher",
        #     output="log",
        #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
        # ),
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name="cell_tf_publisher",
        #     output="log",
        #     arguments=['0.52326', '0.070703', '0.0', '0.0', '0.0', '-0.38269', '0.923877', "panda_link0", "cell"],
        # ),
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name="robot_tf_publisher",
        #     output="log",
        #     arguments=['0.05289', '-0.01869', '-0.06185', '0.71497', '-0.0157', '0.6989', '0.01039', 'panda_hand_tcp', 'camera_link'],
        # ),
    # ])