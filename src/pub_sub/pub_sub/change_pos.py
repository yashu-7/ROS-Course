import sys
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute

class TeleportClient(Node):
    def __init__(self):
        super().__init__('teleport_client')
        self.client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request = TeleportAbsolute.Request()

    def send_request(self, x, y, theta):
        self.request.x = x
        self.request.y = y
        self.request.theta = theta
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = TeleportClient()

    # Check for command-line arguments
    if len(sys.argv) != 3:
        client.get_logger().error('Usage: ros2 run turtle_examples teleport_client <x> <y>')
        rclpy.shutdown()
        return

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        theta = 0.0  # Default theta
    except ValueError:
        client.get_logger().error('Invalid input: x and y must be float values')
        rclpy.shutdown()
        return

    response = client.send_request(x, y, theta)
    client.get_logger().info(f'Result: Teleported to x={x}, y={y}, theta={theta}')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()