import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState
from .sport_model import ROBOT_SPORT_API_IDS
import json

TOPIC_HEIGHSTATE  = "lf/sportmodestate"



class Go2DeviceState(Node):
    def __init__(self):
        super().__init__("go2_device_state")
        self.get_logger().info("Go2 Control Node has been started.")

        self.state_sub = self.create_subscription(SportModeState, TOPIC_HEIGHSTATE, self.state_cb, 10)
        

        


    def state_cb(self, msg : SportModeState):
        error_code = msg.error_code
        mode = msg.mode
        progress = msg.progress
        gait_type = msg.gait_type
        foot_raise_height = msg.foot_raise_height
        velocity = msg.velocity
        yaw_speed = msg.yaw_speed
        position = msg.position
        body_height = msg.body_height
        range_obstacle = msg.range_obstacle

        summary = {
            "error_code": error_code,
            "mode": mode,
            "progress": progress,
            "gait_type": gait_type,
            "foot_raise_height": foot_raise_height,
            "velocity": {
                "x": float(velocity[0]),
                "y": float(velocity[1]),
                "z": float(velocity[2])
            },
            "yaw_speed": yaw_speed,
            "position": {
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2])
            },
            "body_height": body_height,
            # "range_obstacle": range_obstacle
        }

        self.get_logger().info(f"Device State: {json.dumps(summary)}")





def main():
    rclpy.init()
    rclpy.spin(Go2DeviceState())
    rclpy.shutdown()
if __name__ == "__main__":
    main()