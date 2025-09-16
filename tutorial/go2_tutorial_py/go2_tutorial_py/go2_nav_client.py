

# 匯入 ROS2 相關模組
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle
from rclpy.action import ActionClient
from go2_tutorial_inter.action import Nav
import sys


# Go2 導航 Action Client 節點
class Go2NavClient(Node):
    # 初始化函式，建立 action client
    def __init__(self):
        super().__init__("go2_nav_client")
        self.get_logger().info("Go2 Control Node has been started.")  # 節點啟動訊息
        self.client = ActionClient(self, Nav, "nav")  # 建立 action client

    # 連線到 action server，若失敗則重試
    def connect_server(self):
        while not self.client.wait_for_server(1.0):
            if not rclpy.ok():
                return False
            get_logger("rclpy").error("action server not available, waiting again...")
        self.get_logger().info("action server available")
        return True
    
    # 發送目標到 action server
    def send_goal(self, goal):
        goal_msg = Nav.Goal()
        goal_msg.goal = float(goal)
        # 非同步發送目標，並註冊回饋 callback
        future : Future = self.client.send_goal_async(goal_msg, self.feedback_callback)
        future.add_done_callback(self.goal_response)
    
    # 收到 action server 回饋時的 callback
    def feedback_callback(self, fb_msg):
        fb : Nav.Feedback = fb_msg.feedback
        self.get_logger().info(f"distance to goal: {fb.distance:.2f} m")

    # 目標送出後，收到 server 回應的 callback
    def goal_response(self, future : Future):
        goal_handle : ClientGoalHandle = future.result()
        if goal_handle.accepted:
            self.get_logger().info("goal accepted")
            # 取得最終結果的 future
            result_future : Future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_response)
        else:
            self.get_logger().info("goal rejected")
            rclpy.shutdown()
            
    # 收到最終結果時的 callback
    def result_response(self, future : Future):
        result : Nav.Result = future.result().result
        self.get_logger().info(f"result: x={result.point.x:.2f}, y={result.point.y:.2f}, z={result.point.z:.2f}")

# 主程式進入點
def main():
    # 檢查參數數量
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
    rclpy.spin(go2_nav_client)  # 讓 client 節點持續運作
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    