
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
    # 初始化函式，設定 ROS2 節點、參數、訂閱與發布
    def __init__(self):
        super().__init__("go2_nav_server")
        self.get_logger().info("Go2 Control Node has been started.")  # 節點啟動訊息

        self.point = Point()  # 儲存當前位置

        # 訂閱 odom 主題以獲取機器人位置
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_cb, 10)
        # 宣告三個參數 x, y, z
        self.declare_parameter("x", 0.1)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)

        # 建立請求的 publisher
        self.req_pub = self.create_publisher(Request, "/api/sport/request", 10)
        # 設定定時器，週期性發送請求
        self.timer = self.create_timer(0.1, self.on_timer)
        # 初始 API 狀態為平衡站立
        self.api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]

        # 建立 action server，負責導航任務
        self.action_server = ActionServer(
            self,
            Nav,
            'nav',
            self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    # action server 的執行函式，負責處理導航目標
    def execute(self, goal_handle : ServerGoalHandle):
        feedback = Nav.Feedback()  # 建立回饋物件
        while rclpy.ok():
            time.sleep(0.5)  # 每 0.5 秒檢查一次
            # 計算目前與起始點的距離
            dis_x = self.point.x - self.start_point.x
            dis_y = self.point.y - self.start_point.y
            dis = math.sqrt(math.pow(dis_x, 2) + math.pow(dis_y, 2))
            feedback.distance = goal_handle.request.goal - dis  # 計算剩餘距離
            goal_handle.publish_feedback(feedback)  # 發布回饋
            if feedback.distance <= 0.0:
                self.api_id = ROBOT_SPORT_API_IDS["STOPMOVE"]  # 到達目標則停止移動
                break
        goal_handle.succeed()  # 任務成功
        result = Nav.Result()
        result.point = self.point  # 回傳最終位置
        return result
        

    # 接收 action goal 的 callback，檢查目標是否合法
    def goal_callback(self, goal_request : Nav.Goal):
        if goal_request.goal > 0.0:
            self.start_point = self.point  # 記錄起始點
            self.get_logger().info(f"Received goal: {goal_request.goal}")
            self.api_id = ROBOT_SPORT_API_IDS["MOVE"]  # 設定為移動模式
            return GoalResponse.ACCEPT  # 接受目標
        else:
            self.api_id = ROBOT_SPORT_API_IDS["STOPMOVE"]  # 目標不合法則停止
            self.get_logger().error("Goal must be positive.")
            return GoalResponse.REJECT  # 拒絕目標


    # action 取消時的 callback，收到取消請求時停止移動
    def cancel_callback(self, goal_handle):
        self.api_id = ROBOT_SPORT_API_IDS["STOPMOVE"]  # 設定為停止
        return CancelResponse.ACCEPT


    # odom 訂閱回調，更新目前位置
    def odom_cb(self, odom : Odometry):
        self.point = odom.pose.pose.position

    # 定時器回調，週期性發送控制請求到下游
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
    rclpy.init()  # 初始化 ROS2
    node = Go2NavServer()  # 建立導航伺服器節點
    excutor = MultiThreadedExecutor()  # 多執行緒執行器
    excutor.add_node(node)
    excutor.spin()  # 開始執行
    rclpy.shutdown()  # 關閉 ROS2
if __name__ == "__main__":
    main()
    