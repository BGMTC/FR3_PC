"""
Launch file for bringing up the basic task state machine
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
        DeclareLaunchArgument, 
        IncludeLaunchDescription, 
        TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, 
    FindExecutable, 
    LaunchConfiguration,
    PathJoinSubstitution, 
    PythonExpression
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    task_name_parameter_name = 'task_name'
    task_name = LaunchConfiguration(task_name_parameter_name)
    task_name_arg = DeclareLaunchArgument(
        task_name_parameter_name,
        default_value='None',
        description='Name of the main task executable.'
    )

    loop_task_parameter_name = 'loop_task'
    loop_task = LaunchConfiguration(loop_task_parameter_name)
    loop_task_arg = DeclareLaunchArgument(
        loop_task_parameter_name,
        default_value='False',
        description='Toggle looping the task'
    )

    use_vacuum_parameter_name = 'use_vacuum'
    use_vacuum = LaunchConfiguration(use_vacuum_parameter_name)
    use_vacuum_arg = DeclareLaunchArgument(
        use_vacuum_parameter_name,
        default_value='False', 
        description='Toggle whetherFranka Hand or the vacuum gripper is used'
    )

    declared_args = [task_name_arg, loop_task_arg, use_vacuum_arg]

    # Need to invert the 'use_vacuum' for the hand arg in the xacro/urdf
    use_hand = PythonExpression(['not ', '(True if "', use_vacuum, '" == "true" else False)'
    ])

    # planning_context
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )
    
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, 
         ' hand:=true',
         ' robot_ip:=172.16.0.2', 
        #  ' ee_id:=', ee_id, 
         ' use_fake_hardware:=false',
         ' fake_sensor_commands:=false', 
         ' ros2_control:=true'])

    robot_description = {'robot_description': ParameterValue(
        robot_description_config, value_type=str)}

    franka_semantic_xacro_file = os.path.join(
    get_package_share_directory('franka_description'),
    'robots', 'fr3', 'fr3.srdf.xacro'
    )

    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ',
         franka_semantic_xacro_file, ' hand:=true'] #, use_hand] #, ' ee_id:=', ee_id]
    )

    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}


    kinematics_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/kinematics.yaml'
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
        'franka_fr3_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)


#trajectory_execution
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }


# Trajectory Execution Functionality - replacing moveit_cpp
    moveit_simple_controllers_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }


    task_executor = Node(
        name='task_executor',
        package='mtc_panda_task_execution',
        executable='task_executor',
        output='both',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,      
            {'task_name': task_name},
            {'loop_task': loop_task},
            {'use_vacuum': use_vacuum},
        ],
    )

    # Delay the launch of the task_executor node by 5 secondsto allow other components to launch
    delayed_task_executor = TimerAction(
        period=5.0,
        actions=[task_executor]
    )

    inspection_server = Node(
        name='inspection_server',
        package='mtc_panda_visual_inspect',
        executable='inspect'
    )

    modbus_server = Node(
        name='modbus_server',
        package='mtc_panda_io',
        executable='modbus_io',
        condition=IfCondition(use_vacuum)
    )

    static_tf_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [get_package_share_directory('mtc_panda_bringup'), 'launch', 'tf_publisher.launch.py'] 
            )
        )
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [get_package_share_directory('mtc_panda_bringup'), 'launch', 'camera.launch.py']
            )
        )
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory('franka_fr3_moveit_config'),'rviz')
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
            trajectory_execution,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    return LaunchDescription(
        [
            *declared_args,
            delayed_task_executor,
            inspection_server,
            modbus_server,
            camera_launch,
            static_tf_publishers,
            rviz_node

        ]
    )
