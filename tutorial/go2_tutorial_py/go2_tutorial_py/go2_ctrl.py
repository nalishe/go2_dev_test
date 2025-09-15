import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
from .sport_model import ROBOT_SPORT_API_IDS
import json


class Go2Ctrl(Node):
    def __init__(self):
        super().__init__("go2_ctrl")
        self.get_logger().info("Go2 Control Node has been started.")
        self.declare_parameter("sport_api_id", ROBOT_SPORT_API_IDS["BALANCESTAND"])
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)

        self.req_pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.timer = self.create_timer(0.1, self.on_timer)


    def on_timer(self):
        request = Request()
        id = self.get_parameter("sport_api_id").get_parameter_value().integer_value

        request.header.identity.api_id = id

        if id == ROBOT_SPORT_API_IDS["MOVE"]:
            js = {
                "x": self.get_parameter("x").get_parameter_value().double_value,
                "y": self.get_parameter("y").get_parameter_value().double_value,
                "z": self.get_parameter("z").get_parameter_value().double_value,
            }

            request.parameter = json.dumps(js)
        
        self.req_pub.publish(request)


def main():
    rclpy.init()
    rclpy.spin(Go2Ctrl())
    rclpy.shutdown()
if __name__ == "__main__":
    main()