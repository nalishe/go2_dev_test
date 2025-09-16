# 匯入 ROS2 相關模組
import rclpy
from rclpy.node import Node
from go2_tutorial_inter.srv import Cruising
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from unitree_api.msg import Request
from .sport_model import ROBOT_SPORT_API_IDS
import json

# Go2 巡航服務節點類別
class Go2CruisingService(Node):
    # 初始化函式，設定 ROS2 節點、服務、訂閱與定時器
    def __init__(self):
        super().__init__("go2_cruising_service")

        # 建立服務，處理巡航請求
        self.service =  self.create_service(Cruising, "cruising", self.cru_cb)
        # 訂閱 odom 主題以獲取機器人位置
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_cb, 10)
        # 建立請求的 publisher
        self.req_pub = self.create_publisher(Request, "/api/sport/request", 10)
        # 設定定時器，週期性發送請求
        self.timer = self.create_timer(0.1, self.on_timer)
        # 初始 API 狀態為平衡站立
        self.api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]

        # 宣告三個參數 x, y, z
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.5)

        self.point = Point()  # 儲存當前位置

    # 巡航服務的回調函式，根據請求啟動或停止巡航
    def cru_cb(self, request : Cruising.Request, response : Cruising.Response):
        flag = request.flag
        if flag == 0:
            self.api_id = ROBOT_SPORT_API_IDS["STOPMOVE"]  # 停止移動
            self.get_logger().info("end cruising")
        else:
            self.api_id = ROBOT_SPORT_API_IDS["MOVE"]  # 啟動移動
            self.get_logger().info("start cruising")

        response.point = self.point  # 回傳當前位置
        return response

    # odom 訂閱回調函式，更新當前位置
    def odom_cb(self, odom : Odometry):
        self.point = odom.pose.pose.position

    # 定時器回調函式，週期性發送控制請求
    def on_timer(self):
        req = Request()
        req.header.identity.api_id = self.api_id  # 設定 API 狀態
        js = {
            "x": self.get_parameter("x").value,
            "y": self.get_parameter("y").value,
            "z": self.get_parameter("z").value
        }
        req.parameter = json.dumps(js)  # 將參數轉為 JSON 字串
        self.req_pub.publish(req)  # 發布請求

# 主程式進入點，啟動 ROS2 節點
def main():
    rclpy.init()
    rclpy.spin(Go2CruisingService())
    rclpy.shutdown()

if __name__ == "__main__":
    main()