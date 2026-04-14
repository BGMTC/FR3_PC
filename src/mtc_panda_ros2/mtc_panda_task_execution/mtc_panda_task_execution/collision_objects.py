"""
A functions for defining the collision objects for the MTC Franka Panda Cell
"""
from typing import Optional

import numpy as np
from mtc_panda_task_execution.utils import list_to_pose, transform_pose
from moveit.planning import PlanningSceneMonitor
from moveit_msgs.msg import CollisionObject, ObjectColor
from shape_msgs.msg import SolidPrimitive

DESK_WIDTH = 1.35
DESK_DEPTH = 1.0
DESK_HEIGHT = 0.7
MONITOR_DEPTH = 0.3
MONITOR_HEIGHT = 0.4
BUTTONS_DEPTH = 0.13
BUTTONS_HEIGHT = 0.13
WALL_THICKNESS = 0.05
WALL_HEIGHT = 2.0


def add_panda_cell(planning_scene_monitor: PlanningSceneMonitor,
                   cell_tf: Optional[np.ndarray] = None) -> None:
    """ 
    Add in collision objects to model the desk, monitor and buttons on the Panda cell. 
    cell_tf is an optional homogenous transform matrix which describes the transform between the world frame and cell frame
    """
    if cell_tf is None:
        cell_tf = np.identity(4)

    with planning_scene_monitor.read_write() as scene:
        cell = CollisionObject()
        cell.header.frame_id = "panda_link0"
        cell.id = "cell"

        # Desk
        desk_mesh = SolidPrimitive()
        desk_mesh.type = SolidPrimitive.BOX
        desk_mesh.dimensions = (DESK_DEPTH, DESK_WIDTH, DESK_HEIGHT)
        cell.primitives.append(desk_mesh)
        cell.primitive_poses.append(
            transform_pose(list_to_pose([0.0, 0.0, - DESK_HEIGHT / 2]), cell_tf))

        # Monitor
        monitor_mesh = SolidPrimitive()
        monitor_mesh.type = SolidPrimitive.BOX
        monitor_mesh.dimensions = (MONITOR_DEPTH, DESK_WIDTH / 2, MONITOR_HEIGHT)
        cell.primitives.append(monitor_mesh)
        cell.primitive_poses.append(
            transform_pose(
                list_to_pose(
                    [(DESK_DEPTH - MONITOR_DEPTH) / 2, DESK_WIDTH / 4, MONITOR_HEIGHT / 2]),
                cell_tf))

        # Buttons
        buttons_mesh = SolidPrimitive()
        buttons_mesh.type = SolidPrimitive.BOX
        buttons_mesh.dimensions = (BUTTONS_DEPTH, DESK_WIDTH / 2, BUTTONS_HEIGHT)
        cell.primitives.append(buttons_mesh)
        cell.primitive_poses.append(
            transform_pose(
                list_to_pose(
                    [(DESK_DEPTH - BUTTONS_DEPTH) / 2, -DESK_WIDTH / 4, BUTTONS_HEIGHT / 2]),
                cell_tf))

        # Apply collision object and update scene
        oc = ObjectColor()
        oc.id = 'cell'
        oc.color.r, oc.color.g, oc.color.b, oc.color.a = 0.2, 0.2, 1.0, 0.2
        scene.apply_collision_object(cell, oc)
        scene.current_state.update()


def add_walls(planning_scene_monitor: PlanningSceneMonitor,
              back_spacing: Optional[float] = None,
              left_spacing: Optional[float] = None,
              right_spacing: Optional[float] = None,
              cell_tf: Optional[np.ndarray] = None) -> None:
    """ 
    Add in collision objects to model the walls. Parameters left_spacing, back_spacing and right_spacing set the
    distance between the desk and wall, or leave blank for no wall.
    cell_tf is an optional homogenous transform matrix which describes the transform between the world frame and cell frame
    """
    if cell_tf is None:
        cell_tf = np.identity(4)

    # Calculate corner positions
    fl = (DESK_DEPTH / 2, - (DESK_WIDTH / 2 + float(left_spacing or 0)))
    fr = (DESK_DEPTH / 2, (DESK_WIDTH / 2 + float(right_spacing or 0)))
    bl = (- (DESK_DEPTH / 2 + float(back_spacing or 0)), - (DESK_WIDTH / 2 + float(left_spacing or 0)))
    br = (- (DESK_DEPTH / 2 + float(back_spacing or 0)), (DESK_WIDTH / 2 + float(right_spacing or 0)))

    # Create collision object and add to planning scene
    with planning_scene_monitor.read_write() as scene:
        walls = CollisionObject()
        walls.header.frame_id = "panda_link0"
        walls.id = "walls"

        # Back wall
        if back_spacing:
            back_wall = SolidPrimitive()
            back_wall.type = SolidPrimitive.BOX
            back_wall.dimensions = (WALL_THICKNESS, (fr[1] - fl[1]), WALL_HEIGHT)
            walls.primitives.append(back_wall)
            walls.primitive_poses.append(
                transform_pose(
                    list_to_pose([
                        bl[0] + WALL_THICKNESS / 2,
                        (br[1] + bl[1]) / 2,
                        WALL_HEIGHT / 2 - DESK_HEIGHT]),
                    cell_tf
                )
            )

        # Left wall
        if left_spacing:
            left_wall_mesh = SolidPrimitive()
            left_wall_mesh.type = SolidPrimitive.BOX
            left_wall_mesh.dimensions = ((fl[0] - bl[0]), WALL_THICKNESS, WALL_HEIGHT)
            walls.primitives.append(left_wall_mesh)
            walls.primitive_poses.append(
                transform_pose(
                    list_to_pose([
                        (fl[0] + bl[0]) / 2,
                        fl[1] - WALL_THICKNESS / 2,
                        WALL_HEIGHT / 2 - DESK_HEIGHT]),
                    cell_tf
                )
            )

        # Right wall
        if right_spacing:
            right_wall_mesh = SolidPrimitive()
            right_wall_mesh.type = SolidPrimitive.BOX
            right_wall_mesh.dimensions = ((fr[0] - br[0]), WALL_THICKNESS, WALL_HEIGHT)
            walls.primitives.append(right_wall_mesh)
            walls.primitive_poses.append(
                transform_pose(
                    list_to_pose([
                        (fr[0] + br[0]) / 2,
                        fr[1] - WALL_THICKNESS / 2,
                        WALL_HEIGHT / 2 - DESK_HEIGHT]),
                    cell_tf
                )
            )

        # Apply collision object and update scene
        oc = ObjectColor()
        oc.id = 'walls'
        oc.color.r, oc.color.g, oc.color.b, oc.color.a = 0.7, 0.7, 0.7, 0.2
        scene.apply_collision_object(walls, oc)
        scene.current_state.update()


if __name__ == '__main__':
    pass
