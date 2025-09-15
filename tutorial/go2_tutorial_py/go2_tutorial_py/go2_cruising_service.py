import rclpy
from rclpy.node import Node
from go2_tutorial_inter.srv import Cruising
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from unitree_api.msg import Request
from .sport_model import ROBOT_SPORT_API_IDS
import json

class Go2CruisingService(Node):
    def __init__(self):
        super().__init__("go2_cruising_service")

        self.service =  self.create_service(Cruising, "cruising", self.cru_cb)
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_cb, 10)
        self.req_pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]

        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.5)
        

        self.point = Point()

    def cru_cb(self, request : Cruising.Request, response : Cruising.Response):
        flag = request.flag
        if flag == 0:
            self.api_id = ROBOT_SPORT_API_IDS["STOPMOVE"]
            self.get_logger().info("end cruising")
        else:
            self.api_id = ROBOT_SPORT_API_IDS["MOVE"]
            self.get_logger().info("start cruising")

        response.point = self.point

        
        return response
    
    def odom_cb(self, odom : Odometry):
        self.point = odom.pose.pose.position
        
    def on_timer(self):
        req = Request()
        
        req.header.identity.api_id = self.api_id
        js = {
            "x": self.get_parameter("x").value,
            "y": self.get_parameter("y").value,
            "z": self.get_parameter("z").value
        }
        req.parameter = json.dumps(js)

        self.req_pub.publish(req)


def main():
    rclpy.init()
    rclpy.spin(Go2CruisingService())
    rclpy.shutdown()

if __name__ == "__main__":
    main()