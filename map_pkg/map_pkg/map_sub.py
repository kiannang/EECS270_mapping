import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_sub')
        self.subscription = self.create_subscription(
            PoseStamped,           # <<< change message type
            'map_topic',           # <<< same topic
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation

        self.get_logger().info(
            f"Received PoseStamped:\n"
            f"  Position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}\n"
            f"  Orientation (quat): x={ori.x:.2f}, y={ori.y:.2f}, z={ori.z:.2f}, w={ori.w:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MapSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
