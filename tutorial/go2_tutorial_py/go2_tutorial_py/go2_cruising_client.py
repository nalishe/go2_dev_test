import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
import sys
from go2_tutorial_inter.srv import Cruising
from rclpy.task import Future

class Go2CruisingClient(Node):
    def __init__(self):
        super().__init__("go2_cruising_client")
        self.client = self.create_client(Cruising, "cruising")
    
    def connect_server(self):
        while not self.client.wait_for_service(1.0):
            if not rclpy.ok():
                return False
            get_logger("rclpy").error("service not available, waiting again...")
        self.get_logger().info("service available")
        return True

    def send_request(self, flag) -> Future:
        req = Cruising.Request()
        req.flag = int(flag)
        
        return self.client.call_async(req)

def main():

    if len(sys.argv) != 2:
        get_logger("rclpy").error("please input 1 int argument")
        return


    rclpy.init()

    cru_client = Go2CruisingClient()

    flag = cru_client.connect_server()
    if not flag:
        get_logger("rclpy").error("connect server failed")
        return

    future = cru_client.send_request(int(sys.argv[1]))

    rclpy.spin_until_future_complete(cru_client, future)
    if future.done():
        response : Cruising.Response = future.result()
        get_logger("rclpy").info(f"current position: {response.point}")
    else:
        get_logger("rclpy").error("service call failed")
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()