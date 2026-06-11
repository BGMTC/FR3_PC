import time
from typing import List
from importlib import import_module

import mtc_panda_task_execution.task_definition as tasks
import rclpy
import rclpy.logging

from franka_msgs.action import Grasp, Move
from geometry_msgs.msg import Pose
# from moveit import MoveItPy
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from mtc_panda_interfaces.srv import SetIO
from mtc_panda_task_execution.collision_objects import add_panda_cell, add_walls
from mtc_panda_task_execution.task import *
from mtc_panda_task_execution.utils import plan_and_execute, tf2_to_homogenous
from rclpy.action import ActionClient
from rclpy.node import Node
from transitions import Machine, State


class TaskExecutor(Node):

    def __init__(self):
        super().__init__('task_executor')

        # Parameters
        self.declare_parameter('use_fake_hardware', False)
        self.declare_parameter('task_name', '')
        self.declare_parameter('use_vacuum', False)
        self.declare_parameter('loop_task', False)
        self.use_sim = self.get_parameter('use_fake_hardware').get_parameter_value().bool_value
        self.task_name = self.get_parameter('task_name').get_parameter_value().string_value
        self.use_vacuum = self.get_parameter('use_vacuum').get_parameter_value().bool_value
        self.loop_task = self.get_parameter('loop_task').get_parameter_value().bool_value

        # Instantiate MoveItPy instance and get planning component
        self.max_velocity_scaling_factor = 0.5
        self.max_acceleration_scaling_factor = 0.3
        self.franka = MoveItPy(node_name="moveit_py_planning_scene")
        self.franka_arm = self.franka.get_planning_component("fr3v2_arm")
        self.get_logger().info("MoveItPy instance created")

        # Populate planning scene
        cell_tf = tf2_to_homogenous(['0.52326', '0.070703', '0.0', '0.0', '0.0', '-0.38269', '0.923877'])
        self.planning_scene_monitor = self.franka.get_planning_scene_monitor()
        add_walls(self.planning_scene_monitor, back_spacing=0.2, left_spacing=0.5, right_spacing=0.05, cell_tf=cell_tf)
        add_panda_cell(self.planning_scene_monitor, cell_tf=cell_tf)
        self.get_logger().info("Planning scene populated")

        # Define state machine
        states = [
            State(name='initializing'),
            State(name='select_task', on_enter=self.select_task_callback),
            State(name='planned_move', on_enter=self.planned_move_callback),
            State(name='servo_move', on_enter=self.servo_move_callback),
            State(name='toggle_vacuum', on_enter=self.toggle_vacuum_callback),
            State(name='gripper', on_enter=self.gripper_callback),
            State(name='call_service', on_enter=self.call_service_callback),
            State(name='error', on_enter=self.error_callback)
        ]
        transitions = [
            # Trigger                      Source                                            Destination
            ['finish_init',               'initializing',                                   'select_task'],
            ['exec_planned_move',         'select_task',                                    'planned_move'],
            ['exec_servo_move',           'select_task',                                    'servo_move'],
            ['exec_toggle_vacuum',        'select_task',                                    'toggle_vacuum'],
            ['exec_gripper',              'select_task',                                    'gripper'],
            ['exec_service',              'select_task',                                    'call_service'],
            ['complete_move',             ['planned_move', 'servo_move'],                   'select_task'],
            ['complete_action',           ['toggle_vacuum', 'gripper', 'call_service'],     'select_task'],
            ['enter_error_state',         '*',                                              'error']
        ]
        self.state_machine = Machine(model=self, states=states, transitions=transitions,
                                     initial='initializing', queued=True, auto_transitions=False)

        # Task sequence management
        self.task_sequence = self.load_task()
        self.task_counter = 0
        self.current_task: Task = None
        self.pose_register: PoseStamped = None

        # Create modbus io client to toggle vacuum
        if self.use_vacuum and not self.use_sim:
            self.vacuum_client = self.create_client(SetIO, 'update_io')
            while not self.vacuum_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Modbus service not available, waiting again...")
            self.get_logger().info("Modbus service connected")

        # Gripper interfaces
        self.gripper_close_client = ActionClient(self, Grasp, '/default_namespace/franka_gripper/grasp')
        while not self.gripper_close_client.wait_for_server(1.0):
            self.get_logger().info(f"Waiting for franka_msgs.action.Grasp action server")
        self.gripper_open_client = ActionClient(self, Move, '/default_namespace/franka_gripper/move')
        while not self.gripper_open_client.wait_for_server(1.0):
            self.get_logger().info(f"Waiting for franka_msgs.action.Move action server")

        # Move to ready state
        time.sleep(1)
        self.finish_init()

    def load_task(self) -> List[Task]:
        """Using the 'task_name' parameter, load in the correct task from mtc_franka_task_execution/task_definition.py"""
        try:
            task_sequence = getattr(tasks, self.task_name)
            task_heading_str = [f"Loaded {self.task_name} with {len(task_sequence)} steps:"]
            task_sequence_str = [f"{index + 1}. {task}" for index, task in enumerate(task_sequence)]
            formatted_task_details = '\n   '.join([''] + task_heading_str + task_sequence_str)
            self.get_logger().info(formatted_task_details)
            return task_sequence
        except AttributeError as ex:
            self.get_logger().error(f"Could not find a task called '{self.task_name}' in mtc_franka_task_execution.task")
            self.enter_error_state()
        

    def select_task_callback(self) -> None:
        """
        Based on the task_counter, selects the next task to be executed and transitions to the appropriate state
        """
        # Return the task counter to 0 if if the sequence has completed
        self.get_logger().info(f"{self.task_sequence = }")
        if self.task_counter >= len(self.task_sequence):
            if self.loop_task:
                self.task_counter = 0
            else:
                self.get_logger().info('Task complete')
                quit()

        # Select the appropriate state for the next task
        self.current_task = self.task_sequence[self.task_counter]
        if isinstance(self.current_task, MotionTask):
            self.exec_planned_move()
        elif isinstance(self.current_task, ModbusTask):
            self.exec_toggle_vacuum()
        elif isinstance(self.current_task, GripperTask):
            self.exec_gripper()
        elif isinstance(self.current_task, ServiceTask):
            self.exec_service()
        else:
            self.get_logger().error(f"Failed to select to suitable state for task of type'{type(self.current_task)}'")
            self.enter_error_state()

    def planned_move_callback(self) -> None:
        """ 
        Moves from the current robot position to the goal position defined by the current_place_position.
        """
        self.get_logger().info(f"Planner = '{self.current_task.planner}'")
        self.get_logger().info(f"Entered state '{self.state}', task = {self.current_task}")
        if not isinstance(self.current_task, MotionTask):
            self.enter_error_state()
            return

        # Plan and execute move
        self.franka_arm.set_start_state_to_current_state()

        if isinstance(self.current_task, MoveConfiguration):
            self.franka_arm.set_goal_state(configuration_name=self.current_task.goal)
        elif isinstance(self.current_task, MovePose):
            self.franka_arm.set_goal_state(pose_stamped_msg=self.current_task.goal, pose_link="fr3v2_hand_tcp")
        elif isinstance(self.current_task, MovePoseRegister):
            if self.pose_register is not None:
                self.franka_arm.set_goal_state(pose_stamped_msg=self.pose_register, pose_link="fr3v2_hand_tcp")
                self.pose_register = None  # Only use the register once
            else:
                self.get_logger().error("self.pose_register has not been populated")
                self.enter_error_state()
                return
        elif isinstance(self.current_task, MoveJoint):
            robot_state = RobotState(self.franka.get_robot_model())
            robot_state.joint_positions = {
                'fr3v2_joint1': self.current_task.goal.q1,
                'fr3v2_joint2': self.current_task.goal.q2,
                'fr3v2_joint3': self.current_task.goal.q3,
                'fr3v2_joint4': self.current_task.goal.q4,
                'fr3v2_joint5': self.current_task.goal.q5,
                'fr3v2_joint6': self.current_task.goal.q6,
                'fr3v2_joint7': self.current_task.goal.q7
            }
            joint_constraint = construct_joint_constraint(
                robot_state=robot_state,
                joint_model_group=self.franka.get_robot_model().get_joint_model_group("fr3v2_arm"),
                tolerance=0.0001
            )
            self.franka_arm.set_goal_state(motion_plan_constraints=[joint_constraint])
        elif isinstance(self.current_task, MoveNull):
            tcp: Pose = self.franka_arm.get_start_state().get_pose('fr3v2_hand_tcp')
            p, o = tcp.position, tcp.orientation
            joints = self.franka_arm.get_start_state().joint_positions.items()

            self.get_logger().info("-" * 80)
            self.get_logger().info(" ~~~ CURRENT POSITION ~~~ ")
            self.get_logger().info(
                f"[x: {round(p.x, 4)}, y: {round(p.y, 4)}, z: {round(p.z, 4)}, "
                f"rx: {round(o.x, 4)}, ry: {round(o.y, 4)}, rz: {round(o.z, 4)}, rw: {round(o.w, 4)}]"
            )
            self.get_logger().info(f"{ {k: f'{v:.5f}' for k, v in joints} }")
            self.get_logger().info("-" * 80)
            self.task_counter += 1
            self.complete_move()
            return

        # # plan_parameters = PlanRequestParameters(self.franka, self.current_task.planner)
        # plan_parameters = PlanRequestParameters(self.franka)
        # plan_parameters.planner_id = self.current_task.planner 
        # plan_parameters.max_velocity_scaling_factor = self.current_task.vel_scale
        # plan_parameters.max_acceleration_scaling_factor = self.current_task.acc_scale
        # plan_parameters.planning_attempts = 3
        # plan_parameters.planning_time = 3.0
        # success = plan_and_execute(
        #     self.franka, self.franka_arm, self.get_logger(), single_plan_parameters=plan_parameters)


        # def configure_planning(planning_component, task):
        #     planning_component.set_max_velocity_scaling_factor(task.vel_scale)
        #     planning_component.set_max_acceleration_scaling_factor(task.acc_scale)

        #     # planner selection (depends on your MoveIt config)
        #     if hasattr(planning_component, "set_planner_id"):
        #         planning_component.set_planner_id(task.planner)

        self.get_logger().info("Planning trajectory")

        # configure_planning(self.franka_arm, self.current_task)

        success = plan_and_execute(
            self.franka,
            self.franka_arm,
            self.get_logger()
        )


        # Progress to next state
        if success:
            self.task_counter += 1
            self.complete_move()
        else:
            self.get_logger().error("move failed")
            self.enter_error_state()

    def servo_move_callback(self) -> None:
        """
        Performs visual servoing to better align the gripper before performing the pick/place action
        """
        time.sleep(0.1)
        current_step = self.task_sequence[self.task_counter]
        self.get_logger().info(f"Entered state '{self.state}', {current_step = }")
        self.servo.start_teleop()
        finish_time = time.time() + 0
        while time.time() < finish_time:
            self.servo.publish_command()
        self.servo.stop_teleop()
        time.sleep(0.1)
        self.finish_servo_move()

    def toggle_vacuum_callback(self) -> None:
        """
        Activates or deactivates the vacuum according to the task target
        """
        self.get_logger().info(f"Entered state '{self.state}'")
        if not isinstance(self.current_task, ModbusTask):
            self.enter_error_state()
            return
        
        if not self.use_vacuum:
            self.get_logger().error(f"Attempted to execute vacuum/modbus task while {self.use_vacuum = }'")
            self.enter_error_state()
        elif self.use_sim:
            self.get_logger().info(f"Using fakehardware, skipping modbus service call")
            pass
        else:
            req = SetIO.Request()
            req.data = list(self.current_task)
            self.vacuum_client.call_async(req)

        time.sleep(1)
        self.task_counter += 1
        self.complete_action()

    # TODO This might be able to be modified into a generic ActionTask handler?
    # def inspect_request_callback(self) -> None:
    #     self.get_logger().info(f"Entered state '{self.state}'")
    #     time.sleep(1)
        
    #     if self.use_sim:
    #         time.sleep(1)
    #         self.complete_action()
    #     else:
    #         inspect_request = InspectMagnets.Goal()
    #         inspect_request.x1 = self.current_task.x1
    #         inspect_request.y1 = self.current_task.y1
    #         inspect_request.x2 = self.current_task.x2
    #         inspect_request.y2 = self.current_task.y2
    #         future = self.inspect_client.send_goal_async(inspect_request)
    #         future.add_done_callback(self.inspect_response_callback)

    # def inspect_response_callback(self, future) -> None:
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().error("Magnet inspect goal rejected")
    #         self.enter_error_state()
    #         return
    #     self.get_logger().info("Magnet inspect goal accepted")
    #     self._get_result_future = goal_handle.get_result_async()
    #     self._get_result_future.add_done_callback(self.inspect_result_callback)

    # def inspect_result_callback(self, future) -> None:
    #     result = future.result().result
    #     self.get_logger().info("Magnet inspect result received")
    #     self.task_counter += 1
    #     self.complete_action()

    def gripper_callback(self) -> None:
        """
        Handles gripper tasks. Will use the 'Grasp' action if effort is specified, otherwise 'Move' is called
        """
        self.get_logger().info(f"Entered state '{self.state}'")
        if not isinstance(self.current_task, GripperTask):
            self.get_logger().error(f"Invalid task type '{type(self.current_task)}', expected ServiceTask")
            self.enter_error_state()
            return

        if self.current_task.effort:  
            goal_msg = Grasp.Goal()
            goal_msg.width = self.current_task.width
            goal_msg.speed = self.current_task.speed
            goal_msg.force = self.current_task.width
            goal_msg.epsilon.inner = 0.001
            goal_msg.epsilon.outer = 0.08
            self.get_logger().info("Sending gripper grasp goal")
            self.gripper_close_client.send_goal_async(goal_msg)
            time.sleep(2)

        else:
            goal_msg = Move.Goal()
            goal_msg.width = self.current_task.width
            goal_msg.speed = self.current_task.speed
            self.get_logger().info("Sending gripper move goal")
            self.gripper_open_client.send_goal_async(goal_msg)
            time.sleep(2)

        self.task_counter += 1
        self.complete_action()

    def call_service_callback(self) -> None:
        """
        A generic callback for calling services
        """
        self.get_logger().info(f"Entered state '{self.state}'")
        if not isinstance(self.current_task, ServiceTask):
            self.get_logger().error(f"Invalid task type '{type(self.current_task)}', expected ServiceTask")
            self.enter_error_state()
            return
        
        # Dynamically import the service type
        try:
            pkg, subpkg, srv_type_name = self.current_task.type.split('/')
            assert(subpkg == 'srv')
            srv_module = import_module(f"{pkg}.{subpkg}")
            srv_type = getattr(srv_module, srv_type_name)
        except ValueError:
            self.get_logger().error(f"Failed to parse and import {self.current_task.type = }")
            self.enter_error_state()
            return
        except ModuleNotFoundError:
            self.get_logger().error(f"Could not find module '{pkg}/{subpkg}'")
            self.enter_error_state()
            return
        except AttributeError:
            self.get_logger().error(f"Could not find service type '{srv_type}' in module '{pkg}/{subpkg}'")
            self.enter_error_state()
            return
        
        # Create client
        self.client = self.create_client(srv_type, self.current_task.name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"{self.current_task.name} service not available, waiting again...")
        self.get_logger().info(f"Successfully created '{self.current_task.name}' client")

        # Populate request
        req = srv_type.Request()
        for field, data in self.current_task.request_data.items():
            if hasattr(req, field):
                field_type = type(getattr(req, field))
                if isinstance(data, field_type):
                    setattr(req, field, data)
                else:
                    self.get_logger().warning(f"Type mismatch: Field '{field}' expects type '{field_type.__name__}', but received type '{type(data).__name__}'")
                    self.enter_error_state()
                    return
            else:
                self.get_logger().warning(f"Request does not have a field '{field}'")
                self.enter_error_state()
                return    
        self.get_logger().info(f"Request populated successfully with data: {self.current_task.request_data}, calling")

        # Call the service asynchronously
        future = self.client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service response received: {response}")
            self.task_counter += 1

            # TODO Need to be smarter about how to process the response when its unknown until runtime, this is rubbish
            # Look for the first PoseStamped in the response and store it to self.pose_register
            for field_name in dir(response):
                field_val = getattr(response, field_name)
                if isinstance(field_val, PoseStamped):
                    self.pose_register = field_val
                    break

            self.complete_action()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.enter_error_state()


    def error_callback(self) -> None:
        """ 
        # TODO: Handles error states
        """
        self.get_logger().info(f"Entered state '{self.state}'")


def main(args=None):
    rclpy.init(args=args)
    task_executor = TaskExecutor()
    rclpy.spin(task_executor)
    task_executor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
