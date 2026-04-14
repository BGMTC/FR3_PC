"""
Defines instances of tasks to be called by the task executor
"""
from mtc_panda_task_execution.task import *
from mtc_panda_task_execution.utils import (list_to_pose, list_to_pose_stamped,
                                            pose_to_homogenous,
                                            transform_pose_stamped)

# Common
base_frame = 'panda_link0'
capture_pose = MoveNull(None, '')
move_ready = MoveConfiguration('ready', 'ompl_rrtc', 1.0, 0.5)
open_gripper = GripperTask(0.08, 0.1)
close_gripper = GripperTask(0.0, 0.1, 40)


########################################################################################
# CAPTURE_POSE_TASK
########################################################################################

capture_pose_task = [capture_pose]

########################################################################################
# CIRCULAR_FASHION_TASK
########################################################################################

# Above position
above_j = JointTarget(0.54837, 0.03333, -2.84364, -1.25778, -0.01442, 1.25128, -0.67093)
move_above_j = MoveJoint(goal=above_j,
                       planner='ompl_rrtc',
                       vel_scale=1.0,
                       acc_scale=0.5)

# Vision task
vision_task = ServiceTask(type='mtc_panda_interfaces/srv/GetPose',
                          name='/circular_fashion_vision/find_grasp_pose',
                          request_data={'target_frame': base_frame})

# Pre-pick position

pre_pick = list_to_pose_stamped(vals=[-0.3318, -0.3743, 0.1, 0.9121, -0.4098, -0.0138, 0.0069],
                             frame_id=base_frame)
move_pre_pick= MovePose(goal=pre_pick,
                        planner='ompl_rrtc',
                        vel_scale=1.0,
                        acc_scale=0.5)

# pre_pick = JointTarget(0.36471, -0.30622 , -2.61078, -2.18704 , -0.28741 , 2.45322 ,-0.42416)
# move_pre_pick= MoveJoint(goal=pre_pick,
#                        planner='ompl_rrtc',
#                        vel_scale=1.0,
#                        acc_scale=0.5)



# Detected position
move_detected_position = MovePoseRegister(goal=None,
                                        #   planner='pilz_lin',
                                          planner='ompl_rrtc',
                                          vel_scale=1.0,
                                        #   acc_scale=0.002)
                                        acc_scale=0.5)

circular_fashion_task = [
    move_above_j,
    vision_task,
    move_pre_pick,
    move_detected_position,
    move_pre_pick,
    move_above_j,
    move_ready,
]

gripper_test_task = [
    open_gripper,
    move_above_j,
    close_gripper,
    move_ready,
    open_gripper,
]

# ########################################################################################
# # DEMO TASK
# ########################################################################################
# # Poses
# 
# prepick_offset = list_to_pose([0.0, 0.000, -0.02])
# camera_offset = list_to_pose([0, 0.05, -0.02])

# pick_1 = list_to_pose_stamped(
#     [0.435436, 0.154364, 0.053617, -0.382683, 0.92388, 0.0, 0.0], base_frame
# )

# place_1 = list_to_pose_stamped([
#    0.305326, 0.411502, 0.0646, 0.925468, -0.374401, -0.052374, -0.024274
# ], base_frame
# )
# place_2 = list_to_pose_stamped([
#    0.309047, 0.418185, 0.06493, 0.927718, -0.368479, -0.050059, -0.032496
# ], base_frame
# )
# place_3 = list_to_pose_stamped([
#    0.315411, 0.424549, 0.06493, 0.927718, -0.368479, -0.050059, -0.032496
# ], base_frame
# )
# place_4 = list_to_pose_stamped([
#    0.319654, 0.433034, 0.06493, 0.927718, -0.368479, -0.050059, -0.032496
# ], base_frame
# )
# place_5 = list_to_pose_stamped([
#    0.326018, 0.439398, 0.06493, 0.927718, -0.368479, -0.050059, -0.032496
# ], base_frame
# )
# place_6 = list_to_pose_stamped([
#    0.332382, 0.445762, 0.06493, 0.927718, -0.368479, -0.050059, -0.032496
# ], base_frame
# )

# d0 = list_to_pose_stamped(
#         [0.351178, 0.239491, 0.055, -0.379969, 0.924907, 0.004926, 0.012139]
#         , base_frame)
# d1 = list_to_pose_stamped(
#         [0.36882, 0.217578, 0.055, -0.382922, 0.923667, 0.003521, 0.014031]
#         , base_frame)
# d2 = list_to_pose_stamped(
#         [0.372568, 0.209937, 0.055, -0.379142, 0.925235, 0.003146, 0.013464]
#         , base_frame)

# pick_1_camera = transform_pose_stamped(pick_1, pose_to_homogenous(camera_offset), use_pose_frame=True)
# place_1_camera = transform_pose_stamped(place_1, pose_to_homogenous(camera_offset), use_pose_frame=True)

# # Joint targets
# prepick_j = joint_target(0.290408, 0.07405, 0.054128, -2.570065, 0.051662, 2.686061, 0.291621)
# preplace_1_joint = joint_target(1.26201, 0.24937, -0.36175, -2.31085, 0.40534, 2.52319, -1.06352)
# preplace_2_joint = joint_target(0.93928, 0.23899, -0.02365, -2.31638, 0.15775, 2.58776, -0.82357)

# # Steps
# 

# move_d1 = move_pose(d1, 'pilz_lin', 0.01, 0.05)
# move_d2 = move_pose(d2, 'pilz_lin', 0.01, 0.05)

# capture_pose = move_null(None, '')


# pick_5 = modbus_task(c4=True, c5=True)
# place = modbus_task()
# inspect = inspect_task(x1=300, y1=232, x2=470, y2=232)

# def build_pick_place_test(p: PoseStamped,
#                           j = joint_target):
#     move_prepick_test = move_joint(j, 'ompl_rrtc')
#     move_pick_test = move_pose(p, 'pilz_lin', 0.01, 0.05)
#     move_postpick_test = move_joint(j, 'pilz_lin', 0.01, 0.05)
#     return([
#         move_ready, move_prepick_test, capture_pose, move_pick_test, capture_pose, pick_5, move_postpick_test,
#         move_ready, move_prepick_test, capture_pose, move_pick_test, capture_pose, place, move_postpick_test
#     ])

# def build_pick(p: PoseStamped, j = joint_target):
#     move_prepick = move_joint(j, 'ompl_rrtc', 1.0, 0.5)
#     move_pick = move_pose(p, 'pilz_lin', 0.01, 0.05)
#     move_postpick = move_joint(j, 'pilz_lin', 0.01, 0.05)
#     return([
#         move_prepick,  move_pick, pick_5, move_postpick
#     ])

# def build_place(p: PoseStamped, j = joint_target):
#     move_preplace = move_joint(j, 'ompl_rrtc', 1.0, 0.5)
#     move_place = move_pose(p, 'pilz_lin', 0.01, 0.05)
#     move_postplace = move_joint(j, 'pilz_lin', 0.01, 0.05)
#     move_inspect = move_pose(transform_pose_stamped(p, pose_to_homogenous(camera_offset), use_pose_frame=True), 'pilz_lin', 0.1, 0.1)
#     return([
#         move_preplace,  move_place, place, move_postplace, move_inspect
#     ])

# def build_index(p0: PoseStamped, p1: PoseStamped):
#     return([
#         move_pose(transform_pose_stamped(p0, pose_to_homogenous(list_to_pose([0, 0, -0.03])), use_pose_frame=True), 'ompl_rrtc'),
#         move_pose(p0, 'pilz_lin', 0.02, 0.05),
#         move_pose(p1, 'pilz_lin', 0.02, 0.05),
#         move_pose(transform_pose_stamped(p1, pose_to_homogenous(list_to_pose([0, 0.005, -0.03])), use_pose_frame=True), 'ompl_rrtc')
#     ])

# demo_task = [

#     move_ready,
#     *build_pick(pick_1, prepick_j),
#     *build_place(place_1, preplace_1_joint),
#     inspect,
#     *build_index(d0, d1),

#     move_ready, 
#     *build_pick(pick_1, prepick_j),
#     *build_place(place_2, preplace_2_joint),
#     inspect,
#     *build_index(d0, d2),
#     ]



if __name__ == '__main__':
    pass
