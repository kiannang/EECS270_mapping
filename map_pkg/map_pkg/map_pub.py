import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose2D

import cv2
import numpy as np
from cv_bridge import CvBridge
from pupil_apriltags import Detector


class AprilTagPublisher(Node):
    def __init__(self):
        super().__init__('map_pub')

        # Pose2D publisher
        self.pose2d_pub = self.create_publisher(Pose2D, 'map_topic', 10)

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)

        self.caminfo_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10)

        self.bridge = CvBridge()

        self.camera_params = None   # fx, fy, cx, cy
        self.tag_size = 0.1         # meters
        self.detector = Detector(families='tag36h11')

    def camera_info_callback(self, msg: CameraInfo):
        """ Extract fx, fy, cx, cy from camera_info """
        k = msg.k
        fx = k[0]
        fy = k[4]
        cx = k[2]
        cy = k[5]

        self.camera_params = [fx, fy, cx, cy]

    def image_callback(self, msg: Image):
        """ Convert image and run AprilTag when ready """
        if self.camera_params is None:
            self.get_logger().warn("Waiting for camera_info...")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size
        )

        if len(results) == 0:
            print("No tag detected")
            return

        for r in results:
            t = r.pose_t

            # Extract coordinates in camera frame
            tx = float(t[0][0])   # left-right
            tz = float(t[2][0])   # forward
            angle = float(np.degrees(np.arctan2(tx, tz)))

            # Build Pose2D
            pose = Pose2D()
            pose.x = tx
            pose.y = tz
            pose.theta = angle

            # Publish Pose2D
            self.pose2d_pub.publish(pose)

            # Console output only (NOT published)
            print(f"ID:{r.tag_id}  X:{tx:.2f}  Y:{tz:.2f}  Theta:{angle:.1f}")


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
