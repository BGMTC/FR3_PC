"""
Defines classes for storing task information
"""
from dataclasses import dataclass
from typing import Any, List, NamedTuple, Optional, Type, Union

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

#########################
#       Motion tasks    #                          
#########################

class JointTarget(NamedTuple):
    """
    Stores the target joint values for a move_joint task
    """
    q1: float
    q2: float
    q3: float
    q4: float
    q5: float
    q6: float
    q7: float


@dataclass
class MotionTask:
    """
    Base class for storing information for moveit planning
    """
    goal: Any
    planner: str
    vel_scale: float = 0.5
    acc_scale: float = 0.3


@dataclass
class MoveConfiguration(MotionTask):
    """
    Stores information for a moveit motion to a named configuration
    """
    goal: str


@dataclass
class MovePose(MotionTask):
    """
    Stores information for a moveit motion to a pose in task space
    """
    goal: PoseStamped

@dataclass
class MovePoseRegister(MotionTask):
    """
    Stores information for a moveit motion to a pose in task space. Rather than using a pre-defined pose, this will
    read from the `self.pose_register` attribute in the task_executor node
    """
    goal: PoseStamped


@dataclass
class MoveJoint(MotionTask):
    """
    Stores information for a moveit motion to a pose in joint space   
    """
    goal: JointTarget

@dataclass
class MoveNull(MotionTask):
    """
    Utility for logging the current ee pose and joint values
    """
    goal: None


#########################
#       Modbus tasks    #
#########################

class ModbusTask(NamedTuple):
    """
    States of each coil of the modbus IO in the MTC Panda Cell
    """
    c1: bool = False
    c2: bool = False
    c3: bool = False
    c4: bool = False
    c5: bool = False
    c6: bool = False
    c7: bool = False
    c8: bool = False
    c9: bool = False
    c10: bool = False
    c11: bool = False
    c12: bool = False


#########################
#       Util tasks      #
#########################

class GripperTask(NamedTuple):
    width: float
    speed: float
    effort: Optional[float] = None

class ServiceTask(NamedTuple):
    type: Any
    name: str
    request_data: dict[str, Any]






#########################
#         Typing        #
#########################

Task = Union[
    MotionTask,
    ModbusTask,
    GripperTask,
    ServiceTask
]



if __name__ == '__main__':
    pass