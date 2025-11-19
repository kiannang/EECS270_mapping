import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

import cv2
import numpy as np
from cv_bridge import CvBridge
from pupil_apriltags import Detector


class AprilTagPublisher(Node):
    def __init__(self):
        super().__init__('map_pub')

        # PoseStamped publisher
        self.pose_pub = self.create_publisher(PoseStamped, 'map_topic', 10)

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
            ty = float(t[1][0])   # up-down
            tz = float(t[2][0])   # forward

            # horizontal yaw only
            yaw = float(np.degrees(np.arctan2(tx, tz)))

            # Build PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "camera_frame"

            pose_msg.pose.position.x = tx
            pose_msg.pose.position.y = ty
            pose_msg.pose.position.z = tz

            # Orientation: yaw only → convert to quaternion
            cyaw = np.cos(np.deg2rad(yaw) / 2.0)
            syaw = np.sin(np.deg2rad(yaw) / 2.0)

            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = syaw
            pose_msg.pose.orientation.w = cyaw

            # Publish PoseStamped
            self.pose_pub.publish(pose_msg)

            # Console debug only
            print(f"ID:{r.tag_id}  X:{tx:.2f}  Y:{ty:.2f}  Z:{tz:.2f}  Yaw:{yaw:.1f}")


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
