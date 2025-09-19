import rclpy
from rclpy.node import Node
import sys
import os

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient


class Go2Video(Node):
    def __init__(self):
        super().__init__("go2_video")
        self.get_logger().info("Go2 Control Node has been started.")
        ChannelFactoryInitialize(0, "eno1")
        client = VideoClient()
        client.SetTimeout(3.0)
        client.Init()

        self.get_logger().info("##################GetImageSample###################")
        code, data = client.GetImageSample()
        if code != 0:
            print("get image sample error. code:", code)
        else:
            imageName = "/workspace/images/imgs.jpg"
            print("ImageName:", imageName)

            with open(imageName, "+wb") as f:
                f.write(bytes(data))


def main():
    rclpy.init()
    rclpy.spin(Go2Video())
    rclpy.shutdown()
if __name__ == "__main__":
    main()