
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse, GoalResponse, ServerGoalHandle
from go2_tutorial_inter.action import Nav
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from unitree_api.msg import Request
from .sport_model import ROBOT_SPORT_API_IDS
import json
import time
import math


class Go2NavServer(Node):
    def __init__(self):
        super().__init__("go2_nav_server")
        self.get_logger().info("Go2 Control Node has been started.")

        self.point = Point()

        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_cb, 10)
        self.declare_parameter("x", 0.1)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)


        self.req_pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]


        self.action_server = ActionServer(
            self,
            Nav,
            'nav',
            self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def execute(self, goal_handle : ServerGoalHandle):
        feedback = Nav.Feedback()



        

        while rclpy.ok():
            time.sleep(0.5)

            dis_x = self.point.x - self.start_point.x
            dis_y = self.point.y - self.start_point.y
            dis = math.sqrt(math.pow(dis_x, 2) + math.pow(dis_y**2))

            feedback.distance = goal_handle.request.goal - dis

            goal_handle.publish_feedback(feedback)

            if feedback.distance <= 0.0:
                self.api_id = ROBOT_SPORT_API_IDS["STOPMOVE"]
                
                break

        goal_handle.succeed()
        result = Nav.Result()
        result.point = self.point

        return result
        

    def goal_callback(self, goal_request : Nav.Goal):

        if goal_request.goal > 0.0:
            self.start_point = self.point
            self.get_logger().info(f"Received goal: {goal_request.goal}")
            self.api_id = ROBOT_SPORT_API_IDS["MOVE"]
            return GoalResponse.ACCEPT
        else:
            self.api_id = ROBOT_SPORT_API_IDS["STOPMOVE"]
            self.get_logger().error("Goal must be positive.")
            return GoalResponse.REJECT


    def cancel_callback(self, goal_handle):
        self.api_id = ROBOT_SPORT_API_IDS["STOPMOVE"]
        return CancelResponse.ACCEPT


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
    
    node = Go2NavServer()

    excutor = MultiThreadedExecutor()
    excutor.add_node(node)

    excutor.spin()

    rclpy.shutdown()
if __name__ == "__main__":
    main()
    