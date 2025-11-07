import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import RGBToGrayscale
from tutorial_interfaces.msg import RGB
import sys

class GrayscaleClient(Node):
    def __init__(self):
        super().__init__('grayscale_client')
        self.client = self.create_client(RGBToGrayscale, '/rgb_to_grayscale')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = RGBToGrayscale.Request()

    def send_request(self, r, g, b):
        self.req.rgb = RGB()
        self.req.rgb.r = float(r)
        self.req.rgb.g = float(g)
        self.req.rgb.b = float(b)
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(
                f'RGB ({self.req.rgb.r:.1f}, {self.req.rgb.g:.1f}, {self.req.rgb.b:.1f}) -> Grayscale: {future.result().gray:.2f}'
            )
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 4:
        print('Usage: ros2 run tutorial_py_nodes grayscale_client <r> <g> <b>')
        return
    try:
        r, g, b = map(float, sys.argv[1:4])
        if not all(0 <= x <= 255 for x in [r, g, b]):
            print('RGB values must be between 0 and 255')
            return
    except ValueError:
        print('Invalid input: Please provide three numbers for RGB')
        return
    client = GrayscaleClient()
    client.send_request(r, g, b)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()