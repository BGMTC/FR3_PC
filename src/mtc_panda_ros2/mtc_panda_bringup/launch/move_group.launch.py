import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_param_builder import ParameterBuilder
import yaml


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
    use_servo_parameter_name = 'use_servo'
    ros2_control_parameter_name = 'ros2_control'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_servo = LaunchConfiguration(use_servo_parameter_name)
    ros2_control = LaunchConfiguration(ros2_control_parameter_name)

    # planning_context
    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
                                     'panda_arm.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=true',
         ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands])

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

    kinematics_yaml = load_yaml(
        'franka_moveit_config', 'config/kinematics.yaml'
    )

    joint_limits_yaml = load_yaml(
        'mtc_panda_bringup', 'config/joint_limits.yaml'
    )

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'franka_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    pilz_planning_pipeline_config = {
        "planning_pipelines": ["pilz_industrial_motion_planner"],
        "default_planning_pipeline": "pilz_industrial_motion_planner",
        "pilz_industrial_motion_planner": {},
        "move_group": {},
        "robot_description_planning":{},
    }
    pilz_planning_yaml = load_yaml("mtc_panda_bringup", "config/pilz.yaml")
    pilz_planning_pipeline_config["move_group"].update(pilz_planning_yaml)
    pilz_planning_pipeline_config["pilz_industrial_motion_planner"].update(pilz_planning_yaml)
    pilz_cartesian_limits_yaml = load_yaml("mtc_panda_moveit_config", "config/pilz_cartesian_limits.yaml")
    pilz_planning_pipeline_config["robot_description_planning"].update(pilz_cartesian_limits_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'franka_moveit_config', 'config/panda_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_planning_pipeline_config,
            pilz_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # Launch a standalone Servo node.
    servo_params = {
        "moveit_servo": ParameterBuilder("mtc_panda_bringup")
        .yaml("config/servo_params.yaml")
        .to_dict()
    }
    acceleration_filter_update_period = {"update_period": 0.01}
    move_group_name = {"move_group_name": "panda_arm"}
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            acceleration_filter_update_period,
            move_group_name,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            servo_params
        ],
        output="screen",
        condition=IfCondition(use_servo)
    )

    # ROS2 Control
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory('mtc_panda_bringup'), 'launch/panda_nuc.launch.py']),
        ),
        condition=IfCondition(ros2_control)
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory('franka_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
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
    
    use_servo_arg = DeclareLaunchArgument(
        use_servo_parameter_name,
        default_value='true',
        description="Create a moveit servo node")
    
    ros2_control_arg = DeclareLaunchArgument(
        ros2_control_parameter_name,
        default_value='true',
        description="Launch ros2 control nodes for controlling the robot")
    

    return LaunchDescription(
        [
            robot_arg,
            use_fake_hardware_arg,
            fake_sensor_commands_arg,
            use_servo_arg,
            ros2_control_arg,

            # rviz_node,
            move_group_node,
            servo_node,
            control
         ]
    )
