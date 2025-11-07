import rclpy
import random
from rclpy.node import Node
from tutorial_interfaces.msg import RGB

class RGBPublisher(Node):
    def __init__(self):
        super().__init__('rgb_pub')
        self.publisher = self.create_publisher(RGB, '/rgb', 10)
        time_period = 1.0
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.get_logger().info("Node started")
    
    def timer_callback(self):
        msg = RGB()
        msg.r = float(random.randint(0, 255))
        msg.g = float(random.randint(0, 255))
        msg.b = float(random.randint(0, 255))

        self.publisher.publish(msg)
        self.get_logger().info(f"Published RGB:={msg.r},{msg.g},{msg.b}")

def main():
    rclpy.init()
    node = RGBPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()