import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_sub')
        self.subscription = self.create_subscription(
            Pose2D,
            'map_topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(
            f"Received Pose2D: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.1f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MapSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
