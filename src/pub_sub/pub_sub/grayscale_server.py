import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import RGBToGrayscale

class GrayscaleServer(Node):
    def __init__(self):
        super().__init__('grayscale_server')
        self.srv = self.create_service(RGBToGrayscale, '/rgb_to_grayscale', self.rgb_to_grayscale_callback)
        self.get_logger().info('Grayscale Service Server initialized')

    def rgb_to_grayscale_callback(self, request, response):
        # Standard luminance formula: gray = 0.2989 * r + 0.5870 * g + 0.1140 * b
        response.gray = 0.2989 * request.rgb.r + 0.5870 * request.rgb.g + 0.1140 * request.rgb.b
        self.get_logger().info(
            f'Incoming RGB: ({request.rgb.r:.1f}, {request.rgb.g:.1f}, {request.rgb.b:.1f}) -> Grayscale: {response.gray:.2f}'
        )
        return response

def main(args=None):
    rclpy.init(args=args)
    server = GrayscaleServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()