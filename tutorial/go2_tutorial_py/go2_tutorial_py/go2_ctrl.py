# 匯入 ROS2 相關模組
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
from .sport_model import ROBOT_SPORT_API_IDS
import json


# Go2 控制節點類別
class Go2Ctrl(Node):
    # 初始化函式，設定 ROS2 節點、參數與定時器
    def __init__(self):
        super().__init__("go2_ctrl")
        self.get_logger().info("Go2 Control Node has been started.")  # 節點啟動訊息
        # 宣告參數，包含運動模式與三軸座標
        self.declare_parameter("sport_api_id", ROBOT_SPORT_API_IDS["BALANCESTAND"])
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)

        # 建立請求的 publisher
        self.req_pub = self.create_publisher(Request, "/api/sport/request", 10)
        # 設定定時器，週期性發送請求
        self.timer = self.create_timer(0.1, self.on_timer)


    # 定時器回調函式，根據參數發送控制請求
    def on_timer(self):
        request = Request()
        id = self.get_parameter("sport_api_id").get_parameter_value().integer_value

        request.header.identity.api_id = id  # 設定 API ID

        # 如果是移動模式，加入三軸座標參數
        if id == ROBOT_SPORT_API_IDS["MOVE"]:
            js = {
                "x": self.get_parameter("x").get_parameter_value().double_value,
                "y": self.get_parameter("y").get_parameter_value().double_value,
                "z": self.get_parameter("z").get_parameter_value().double_value,
            }

            request.parameter = json.dumps(js)  # 將參數轉為 JSON 字串
        
        self.req_pub.publish(request)  # 發布請求


# 主程式進入點，啟動 ROS2 節點
def main():
    rclpy.init()
    rclpy.spin(Go2Ctrl())
    rclpy.shutdown()

if __name__ == "__main__":
    main()