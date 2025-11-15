import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import numpy as np
from pupil_apriltags import Detector


class AprilTagPublisher(Node):
    def __init__(self):
        super().__init__('map_pub')

        # ROS publisher
        self.publisher_ = self.create_publisher(String, 'map_topic', 10)

        # Camera init
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            exit()

        # AprilTag detector
        self.detector = Detector(families='tag36h11')

        self.camera_params = [600, 600, 640, 360]  # fx, fy, cx, cy
        self.tag_size = 0.1  # meters

        # Timer runs at 10 Hz
        self.timer = self.create_timer(0.1, self.detect_tags)

    def detect_tags(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to grab frame")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size
        )

        if len(results) == 0:
            msg = String()
            msg.data = "No tag detected"
            self.publisher_.publish(msg)
            self.get_logger().info(msg.data)
            return

        for r in results:
            tag_id = r.tag_id
            t = r.pose_t  # translation vector
            distance = float(np.linalg.norm(t))
            angle = float(np.degrees(np.arctan2(t[0][0], t[2][0])))

            msg_text = f"ID:{tag_id} Distance:{distance:.2f} Angle:{angle:.1f}"

            msg = String()
            msg.data = msg_text

            # publish
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: {msg_text}")

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()