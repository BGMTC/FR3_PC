from typing import Optional, Sequence, Tuple, Union

import numpy as np
import pyassimp
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from scipy.spatial.transform import Rotation
from shape_msgs.msg import Mesh, MeshTriangle

def list_to_pose(vals: list) -> Pose:
    """Construct a geometry_msgs.msg.PoseStamped from a list in form [x, y, z] or [x, y, z, rx, ry, rz, rw] """
    if not all(isinstance(v, float | int) for v in vals):
        raise ValueError("All list values must be numeric")

    vals = [float(v) for v in vals]

    pose = Pose()
    if len(vals) == 3:
        pose.position.x, pose.position.y, pose.position.z = vals
        return pose
    elif len(vals) == 7:
        pose.position.x, pose.position.y, pose.position.z = vals[:3]
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = vals[3:]
        return pose
    else:
        raise ValueError(f"Could not construct a PoseStamped message from a list with length {len(vals)}")


def list_to_pose_stamped(vals: list, frame_id: Optional[str] = None) -> PoseStamped:
    """ Construct a geometry_msgs.msg.PoseStamped from a list of values and a frame_id"""
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.pose = list_to_pose(vals)
    return pose_stamped


def load_mesh(filename: str, scale: float = 1.0) -> Mesh:
    """ Load an .stl file and build a shape_msgs.msg.Mesh """
    try:
        with pyassimp.load(filename) as scene:
            if not scene.meshes or len(scene.meshes) == 0:
                raise Exception("There are no meshes in the file")
            if len(scene.meshes[0].faces) == 0:
                raise Exception("There are no faces in the mesh")

            mesh = Mesh()
            for face in scene.meshes[0].faces:
                triangle = MeshTriangle()
                indicies = [
                    int(face[0]),
                    int(face[1]),
                    int(face[2])
                ]
                triangle.vertex_indices = indicies
                mesh.triangles.append(triangle)
            for vertex in scene.meshes[0].vertices:
                point = Point()
                point.x, point.y, point.z = [
                    float(vertex[0]) * scale,
                    float(vertex[1]) * scale,
                    float(vertex[2]) * scale
                ]
                mesh.vertices.append(point)
            return mesh

    except Exception as ex:
        print(ex)
        return None


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
):
    """A helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
        return True
    else:
        logger.error("Planning failed")
        return False


def homogenous_to_trans_quat(h: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Convert a 4x4 homogenous transformation matrix to a translation vector and unit quaternion
    """
    t = h[:3, 3].reshape(3)
    q = Rotation.from_matrix(h[:3, :3]).as_quat()
    return t, q


def trans_quat_to_homogenous(t: np.ndarray, q: np.ndarray) -> np.ndarray:
    """
    Convert a translation vector and unit quaternion [x,y,z,w] to a 4x4 homogenous transformation matrix
    """
    tf_mat = np.vstack([np.hstack([Rotation.from_quat(np.asarray(q)).as_matrix(),
                                   np.asarray(t).reshape(3, 1)]), [0, 0, 0, 1]])
    return tf_mat


def homogenous_to_tf2(h: np.ndarray, precision: int = 4, output_string: bool = True) -> Union[str, Tuple[float, ...]]:
    """
    Convert a 4x4 homogenous transformation matrix to a string of values as required for a tf2 transform publisher.
    """
    t, q = homogenous_to_trans_quat(h)
    if output_string:
        tf = (', ').join([f"'{round(n, precision)}'" for n in list(np.hstack([t, q]))])
    else:
        tf = tuple([round(n, precision) for n in list(np.hstack([t, q]))])
    return tf


def tf2_to_homogenous(tf: Sequence) -> np.ndarray:
    """
    Convert an array of values in the order required for a tf2 transform publisher to a 4x4 homogenous transformation matrix
    """

    if all(isinstance(n, str) for n in tf):
        vals = [float(n.replace("'", "").replace('"', '')) for n in tf]
    elif all(str(n).isnumeric for n in tf):  # Catches both floats and integers
        vals = tf
    else:
        raise ValueError("Invalid typing in input sequence")

    if len(vals) == 7:
        tf_mat = trans_quat_to_homogenous(np.array(vals[:3]), np.array(vals[3:]))
        return tf_mat
    else:
        raise ValueError("Sequence must contain seven elements (x, y, z, rx, ry,rz, rw)")


def pose_to_homogenous(p: Pose) -> np.ndarray:
    """
    Convert a geometry_msgs/msg/Pose message to a 4x4 homogenous transform matrix
    """
    T = trans_quat_to_homogenous(
        t=np.array([p.position.x, p.position.y, p.position.z]),
        q=np.asarray([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    )
    return T


def homogenous_to_pose(T: np.ndarray) -> Pose:
    """
    Convert a 4x4 homogenous transform matrix to a geometry_msgs/msg/Pose message
    """
    t, q = homogenous_to_trans_quat(T)
    p = Pose(position=Point(x=t[0], y=t[1], z=t[2]),
             orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
    return p


def transform_pose(p1: Pose, T: np.ndarray, use_pose_frame: bool = False) -> Pose:
    """
    Apply a homogenous transform to a geometry_msgs/msg/Pose message w.r.t the world frame. Set use_pose_frame=true to 
    apply the transform in the original pose's frame, otherwise the transform wil be in the world frame. 
    """
    p1_h = pose_to_homogenous(p1)

    if use_pose_frame:
        p2_h = p1_h @ T
    else:
        p2_h = T @ p1_h

    p2 = homogenous_to_pose(p2_h)
    return p2


def transform_pose_stamped(p1: PoseStamped, T: np.ndarray, use_pose_frame: bool = False) -> PoseStamped:
    """
    Apply a homogenous transform to a geometry_msgs/msg/PoseStamped message w.r.t the world frame. Set 
    use_pose_frame=true to apply the transform in the original pose's frame, otherwise the transform wil be in the world
    frame. 
    """
    p2 = PoseStamped(pose=transform_pose(p1.pose, T, use_pose_frame),
                     header=p1.header)
    return p2


if __name__ == '__main__':
    pass
