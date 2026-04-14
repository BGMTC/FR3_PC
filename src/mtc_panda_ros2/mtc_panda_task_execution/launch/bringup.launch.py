"""
Launch file for bringing up the full magtec pick-and-place demonstrator
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, Shutdown)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, ThisLaunchFileDir)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    task_executable_parameter_name = 'task_executable'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    task_executable = LaunchConfiguration(task_executable_parameter_name)

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="panda", package_name='mtc_panda_moveit_config'
        )
        .moveit_cpp(
            file_path="config/planners.yaml"
        )
        .to_moveit_configs()
    )

    # Planning_context
    franka_xacro_file = os.path.join(get_package_share_directory('mtc_panda_description'), 'urdf',
                                     'panda.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=true',
         ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands])

    robot_description = {'robot_description': robot_description_config}

    magtec_system = Node(
        name="moveit_py",
        package="mtc_panda_task_execution",
        executable=task_executable,
        output="both",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    magtec_inspect = Node(
        name='magtec_inspect',
        package='magtec_system',
        executable='magtec_inspect'
    )

    # RViz
    rviz_config = os.path.join(get_package_share_directory('magtec_system'), 'config', 'config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.to_dict()
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description],
    )

    static_tf_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=[ThisLaunchFileDir(), '/tf_publisher.launch.py']
        )
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('franka_moveit_config'),
        'config',
        'panda_ros_controllers.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    servo_yaml = load_yaml("magtec_system", "config/servo.yaml")

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_yaml,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    # Load controllers
    load_controllers = []
    for controller in ['panda_arm_controller', 'joint_state_broadcaster']:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner {}'.format(controller)],
                shell=True,
                output='screen',
            )
        ]

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'source_list': ['franka/joint_states', 'panda_gripper/joint_states'], 'rate': 30}],
    )

    franka_robot_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['franka_robot_state_broadcaster'],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )

    modbus_service = Node(
        package='mtc_beckhoff_modbus_io',
        executable='modbus_io'
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=[ThisLaunchFileDir(), '/camera.launch.py']
        )
    )

    robot_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        default_value='172.20.9.185',
        description="Hostname or IP address of the robot.")

    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value='false',
        description="Use fake hardware.")

    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value='false',
        description="Fake sensor commands. Only valid when '{}' is true.".format(
            use_fake_hardware_parameter_name))

    task_executable_arg = DeclareLaunchArgument(
        task_executable_parameter_name,
        default_value='magtec_system',
        description='Name of the main task executable.'
    )

    return LaunchDescription(
        [
            robot_arg,
            use_fake_hardware_arg,
            fake_sensor_commands_arg,
            task_executable_arg,

            static_tf_publishers,
            rviz_node,
            robot_state_publisher,
            ros2_control_node,
            joint_state_publisher,
            franka_robot_state_broadcaster,

            # servo_node,
            magtec_system,
            magtec_inspect,
            modbus_service,
            camera_launch

        ]
        + load_controllers
    )
