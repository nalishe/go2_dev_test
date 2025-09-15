# 匯入 ROS2 Python 函式庫
import rclpy
from rclpy.node import Node
# 匯入 Unitree API 的 Request 訊息型別
from unitree_api.msg import Request
# 匯入 ROS2 Twist 訊息型別（速度指令）
from geometry_msgs.msg import Twist

# 匯入自訂的運動模式 API ID 定義
from .sport_model import ROBOT_SPORT_API_IDS

import json


# TwistBridge 節點：將 /cmd_vel 速度指令轉換為 Unitree API 請求，發佈給機器人底層
class TwistBridge(Node):
    def __init__(self):
        super().__init__("twist_bridge_py")
        self.get_logger().info("py created!")

        # 建立 Publisher，發佈 Request 到 Unitree API
        self.request_pub = self.create_publisher(Request,'/api/sport/request', 10)
        # 建立 Subscriber，訂閱 /cmd_vel 速度指令
        self.request_sub = self.create_subscription(Twist,'cmd/vel',self.twist_cb ,10)

    # 當收到 /cmd_vel 訊息時的回呼函式
    def twist_cb(self, twist:Twist):
        request = Request()  # 建立請求訊息

        # 取得 Twist 中的線速度與角速度
        x = twist.linear.x
        y = twist.linear.y
        z = twist.angular.z

        # 預設 API 為平衡站立
        api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]
        # 若有速度指令則切換為移動模式，並將速度參數包裝成 JSON
        if x != 0 or y != 0 or z != 0:
            api_id = ROBOT_SPORT_API_IDS["MOVE"]

            js = {"x":x, "y":y, "z":z}  # 線速度與角速度
            request.parameter = json.dumps(js)  # 轉成字串存入 parameter

        # 設定 API ID
        request.header.identity.api_id = api_id
        # 發佈請求
        self.request_pub.publish(request)
        

# 主程式進入點
def main():
    rclpy.init()
    rclpy.spin(TwistBridge())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
