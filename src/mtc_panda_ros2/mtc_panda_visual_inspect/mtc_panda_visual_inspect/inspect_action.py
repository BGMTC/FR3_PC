"""
An action client which can be called by other nodes which processes feed from an image stream
using OpenCV

Replace the 'count_magnets' method with your own code
"""
from copy import deepcopy
import os
from typing import Tuple, Union

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from mtc_panda_interfaces.action import InspectMagnets
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


class InspectAction(Node):

    def __init__(self):
        super().__init__('visual_inspection')  # type: ignore

        # Subscribers and publishers
        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic='camera/color/image_raw',
            callback=self.camera_cb,
            qos_profile=qos_profile_system_default)

        self.output_pub = self.create_publisher(
            msg_type=Image,
            topic='~/image_processed',
            qos_profile=qos_profile_system_default)

        # Inspect action server
        self.inspect_server = ActionServer(
            self,
            action_type=InspectMagnets,
            action_name='inspect_magnets',
            execute_callback=self.inspect_action_cb)

        # Other image processing attributes
        self.last_imgmsg: Union[Image, None] = None
        self.cv_bridge = CvBridge()

    def camera_cb(self, msg: Image):
        """
        Stores the most recent image message from the camera
        """
        self.last_imgmsg = msg

    def inspect_action_cb(self, goal_handle):
        """
        Action server callback
        """
        self.get_logger().info('Executing inspect action...')
        result = InspectMagnets.Result()
        save_dir = '/workspaces/mtc_panda_project/inspect_images'
        if self.last_imgmsg is not None:
            image = self.cv_bridge.imgmsg_to_cv2(self.last_imgmsg, desired_encoding='bgr8')
            cv2.imwrite(os.path.join(save_dir, f'{len(os.listdir(save_dir)):03d}.png'), image)
            p1 = goal_handle.request.x1, goal_handle.request.y1
            p2 = goal_handle.request.x2, goal_handle.request.y2
            success, annotated, _ = self.magtec_inspect_line(image, p1, p2)
            self.output_pub.publish(self.cv_bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
            goal_handle.succeed()
            result.success = success
            self.output_pub.publish(self.cv_bridge.cv2_to_imgmsg(annotated))
        else:
            pass

        return result

    @staticmethod    
    def bresenham_line_2d(p1: Tuple[int, int], p2: Tuple[int, int]) -> np.ndarray:
        """Uses Bresenham's line drawing algorithm to return the integer pixel coordinates intersected by a line between points p1 and p2

        Args:
            p1 (Tuple[int, int]): Starting xy coordinate of the line
            p2 (Tuple[int, int]): End xy of the line

        Returns:
            np.ndarray: 2xN array of integer points intersected by line p1->p2
        """

        x, y = x1, y1 = p1
        x2, y2 = p2
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        gradient = dy/float(dx)

        if gradient > 1:
            dx, dy = dy, dx
            x, y = y, x
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        p = 2 * dy - dx

        xcoordinates, ycoordinates = [x], [y]

        for _ in range(2, dx + 2):
            if p > 0:
                y = y + 1 if y < y2 else y - 1
                p = p + 2 * (dy - dx)
            else:
                p = p + 2 * dy

            x = x + 1 if x < x2 else x - 1
            xcoordinates.append(x)
            ycoordinates.append(y)

        return np.vstack([xcoordinates, ycoordinates]).T

    def magtec_inspect_line(self, frame: np.ndarray, p1: Tuple[int, int], p2: Tuple[int, int]) -> Tuple[bool, np.ndarray, np.ndarray]:
        output = deepcopy(frame)
        hsv = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([62,100,100])
        upper_hsv = np.array([75,255,255])
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        masked = cv2.bitwise_and(output, output, mask=mask)
        
        success = True
        for p in self.bresenham_line_2d(p1, p2):
            if mask[p[1], p[0]] == 0:
                cv2.circle(output, p.astype(int), 1, (0, 255, 0), -1)
            else:
                success = False
                cv2.circle(output, p.astype(int), 2, (0, 0, 255), -1)

        return success, output, masked


def main(args=None):
    rclpy.init(args=args)
    inspect_action = InspectAction()
    rclpy.spin(inspect_action)
    inspect_action.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
