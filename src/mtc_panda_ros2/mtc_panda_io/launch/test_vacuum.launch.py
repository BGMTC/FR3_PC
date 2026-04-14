from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    modbus_server = Node(
        package='mtc_panda_io',
        executable='modbus_io'
    )

    test_client = Node(
        package='mtc_panda_io',
        executable='io_test_client'
    )

    return LaunchDescription(
        [
            modbus_server,
            test_client
        ]
    )