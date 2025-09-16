# 匯入 ROS2 相關模組
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

# Go2 狀態節點類別
class Go2State(Node):
    # 初始化函式，設定 ROS2 節點、訂閱 odom、初始化狀態
    def __init__(self):
        super().__init__("go2_state")
        self.get_logger().info("Go2 state Node has been started.")  # 節點啟動訊息
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_cb, 10)  # 訂閱 odom 主題

        self.last_x = 0.0  # 上一次 x 座標
        self.last_y = 0.0  # 上一次 y 座標
        self.is_first = True  # 是否為第一次接收 odom
        self.declare_parameter("distance", 0.5)  # 觸發距離參數

    # odom 訂閱回調，計算移動距離並根據條件輸出座標
    def odom_cd(self, odom:Odometry):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        if self.is_first:
            self.last_x = x
            self.last_y = y
            self.is_first = False
            self.get_logger().info("init:%.2f, %.2f", x, y)  # 第一次收到 odom，初始化座標
            return
        # 計算與上次的距離
        distance_x = x - self.last_x
        distance_y = y - self.last_y
        distance = math.sqrt(distance_x ** 2 + distance_y ** 2)

        # 若移動距離超過參數，則輸出目前座標並更新
        if distance >= self.get_parameter("distance").value:
            self.get_logger().info("now:%.2f, %.2f", x, y)
            self.last_x = x
            self.last_y = y

# 主程式進入點，啟動 ROS2 節點
def main():
    rclpy.init()
    rclpy.spin(Go2State())
    rclpy.shutdown()

if __name__ == "__main__":
    main()