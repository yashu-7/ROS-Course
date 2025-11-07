import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.get_logger().info("Node started")

    def pose_callback(self, msg):
        self.get_logger().info(f"Turtle pose: x:={msg.x:.3f} y:={msg.y:.3f}, theta:={msg.theta:.3f}")

def main():
    rclpy.init()
    node = PoseSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()