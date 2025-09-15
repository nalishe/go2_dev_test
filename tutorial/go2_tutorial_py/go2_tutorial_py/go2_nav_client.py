
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle
from rclpy.action import ActionClient
from go2_tutorial_inter.action import Nav


import sys

class Go2NavClient(Node):
    def __init__(self):
        super().__init__("go2_nav_client")
        self.get_logger().info("Go2 Control Node has been started.")
        self.client = ActionClient(self, Nav, "nav")

    def connect_server(self):
        while not self.client.wait_for_server(1.0):
            if not rclpy.ok():
                return False
            get_logger("rclpy").error("action server not available, waiting again...")
        self.get_logger().info("action server available")
        return True
    
    def send_goal(self, goal):

        goal_msg = Nav.Goal()

        goal_msg.goal = float(goal)


        future : Future = self.client.send_goal_async(goal_msg, self.feedback_callback)
        future.add_done_callback(self.goal_response)
    
    def feedback_callback(self, fb_msg):

        fb : Nav.Feedback = fb_msg.feedback

        self.get_logger().info(f"distance to goal: {fb.distance:.2f} m")

    def goal_response(self, future : Future):
        goal_handle : ClientGoalHandle = future.result()
        # self.get_logger().info("%s" % goal_handle.__str__())
        if goal_handle.accepted:
            self.get_logger().info("goal accepted")
            result_future : Future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_response)
        else:
            self.get_logger().info("goal rejected")
            rclpy.shutdown()
            
    def result_response(self, future : Future):
        result : Nav.Result = future.result().result
        self.get_logger().info(f"result: x={result.point.x:.2f}, y={result.point.y:.2f}, z={result.point.z:.2f}")

        




def main():

    if len(sys.argv) != 2:
        get_logger("rclpy").error("Usage: ros2 run go2_tutorial_py go2_nav_client.py <distance>")
        return



    rclpy.init()

    go2_nav_client = Go2NavClient()

    flag = go2_nav_client.connect_server()
    if not flag:
        get_logger("rclpy").error("connect server failed")
        return
    get_logger("rclpy").info("connected to server")


    go2_nav_client.send_goal(float(sys.argv[1]))

    rclpy.spin(Go2NavClient())
    rclpy.shutdown()



if __name__ == "__main__":
    main()
    