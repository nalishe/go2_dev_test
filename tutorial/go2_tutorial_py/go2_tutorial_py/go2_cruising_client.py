# 匯入 ROS2 相關模組
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
import sys
from go2_tutorial_inter.srv import Cruising
from rclpy.task import Future

# Go2 巡航客戶端節點類別
class Go2CruisingClient(Node):
    # 初始化函式，建立服務客戶端
    def __init__(self):
        super().__init__("go2_cruising_client")
        self.client = self.create_client(Cruising, "cruising")  # 建立服務客戶端

    # 連線到服務端，若失敗則重試
    def connect_server(self):
        while not self.client.wait_for_service(1.0):
            if not rclpy.ok():
                return False
            get_logger("rclpy").error("service not available, waiting again...")
        self.get_logger().info("service available")
        return True

    # 發送請求到服務端
    def send_request(self, flag) -> Future:
        req = Cruising.Request()
        req.flag = int(flag)  # 設定請求的 flag
        return self.client.call_async(req)  # 非同步呼叫服務

# 主程式進入點
def main():
    # 檢查參數數量
    if len(sys.argv) != 2:
        get_logger("rclpy").error("please input 1 int argument")
        return

    rclpy.init()

    cru_client = Go2CruisingClient()  # 建立巡航客戶端節點

    flag = cru_client.connect_server()  # 嘗試連線到服務端
    if not flag:
        get_logger("rclpy").error("connect server failed")
        return

    future = cru_client.send_request(int(sys.argv[1]))  # 發送請求

    rclpy.spin_until_future_complete(cru_client, future)  # 等待請求完成
    if future.done():
        response : Cruising.Response = future.result()
        get_logger("rclpy").info(f"current position: {response.point}")  # 輸出當前位置
    else:
        get_logger("rclpy").error("service call failed")
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()