# go2_basic_services/light_audio.py
import json
import rclpy
from rclpy.node import Node



TOPIC_VUI_REQ  = "/api/vui/request"
TOPIC_VUI_RES  = "/api/vui/response"
TOPIC_AH_REQ   = "/api/audiohub/request"
TOPIC_AH_RES   = "/api/audiohub/response"
TOPIC_PLAYER   = "/audiohub/player/state"  # 只讀（可選）

class Go2LightAudio(Node):
    def __init__(self):
        super().__init__("go2_light_audio")
        


    












def main():
    rclpy.init()
    node = Go2LightAudio()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
