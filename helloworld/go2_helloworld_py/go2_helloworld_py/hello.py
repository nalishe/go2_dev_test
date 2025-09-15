import rclpy
from rclpy.node import Node
from unitree_api.msg import Request


class HelloWorldPy(Node):
    def __init__(self):
        super().__init__('go2_helloworld_py')
        self.pub = self.create_publisher(Request, "/api/sport/request", 10)

        self.timer = self.create_timer(1.0, self.on_timer)
    

    def on_timer(self):
        request = Request()
        request.header.identity.api_id = 1016

        self.pub.publish(request)


def main():
    rclpy.init()
    rclpy.spin(HelloWorldPy())
    rclpy.shutdown()

if __name__ == '__main__':
    main()