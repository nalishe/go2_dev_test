brightness = {
    "0": 0,
    "1": 1,
    "2": 2,
    "3": 3,
    "4": 4,
    "5": 5,
    "6": 6,
    "7": 7,
    "8": 8,
    "9": 9,

}





import rclpy
from rclpy.node import Node

import termios
import sys
import tty
import threading
import json
import os
import subprocess


class SdkLight(Node):
    def __init__(self):
        super().__init__("sdk_light")
        self.get_logger().info("Go2 Control Node has been started.")




def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



def main():
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    sdk_light = SdkLight()
    spinner = threading.Thread(target=rclpy.spin, args=(sdk_light, ))
    spinner.start()

    try:
        while True:
            key = getKey(settings)

            if key == '\x03':
                break
            elif key in brightness.keys():
                level = brightness[key]
                sdk_light.get_logger().info(f"set brightness to {level}")
                sudo_pw = os.environ.get("SUDO_PASS", "a")
                cmd = ["sudo", "-S", "-p", "", "./user_vui_client", str(level)]
                cwd = "/workspace/src/unitree_sdk2/build/bin"

                res = subprocess.run(cmd, cwd=cwd, input=sudo_pw + "\n", capture_output=True, text=True, check=False, timeout=5)
                sdk_light.get_logger().info(f"stdout: {res.stdout}")
                sdk_light.get_logger().info(f"stderr: {res.stderr}")


    finally:
        rclpy.shutdown()



        
if __name__ == "__main__":
    main()