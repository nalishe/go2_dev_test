
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class Go2State(Node):
    def __init__(self):
        super().__init__("go2_state")
        self.get_logger().info("Go2 state Node has been started.")
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_cb, 10)

        self.last_x = 0.0
        self.last_y = 0.0
        self.is_first = True
        self.declare_parameter("distance", 0.5)

    def odom_cd(self, odom:Odometry):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        if self.is_first:
            self.last_x = x
            self.last_y = y
            self.is_first = False
            self.get_logger().info("init:%.2f, %.2f", x, y)
            return
        
        distance_x = x - self.last_x
        distance_y = y - self.last_y

        distance = math.sqrt(distance_x ** 2 + distance_y ** 2)

        if distance >= self.get_parameter("distance").value:
            self.get_logger().info("now:%.2f, %.2f", x, y)
            self.last_x = x
            self.last_y = y
    



def main():
    rclpy.init()
    rclpy.spin(Go2State())
    rclpy.shutdown()
if __name__ == "__main__":
    main()