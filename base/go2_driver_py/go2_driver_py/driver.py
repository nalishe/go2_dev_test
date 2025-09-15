import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState, LowState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState


class Driver(Node):
    def __init__(self):
        super().__init__("driver")

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.mode_sub = self.create_subscription(SportModeState, "lf/sportmodestate", self.mode_cb, 10)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("publish_tf", True)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = self.get_parameter("publish_tf").value

        self.tf_bro = TransformBroadcaster(self)

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.state_sub = self.create_subscription(LowState, "lf/lowstate", self.state_cb, 10)

    def mode_cb(self, mode: SportModeState):

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(mode.position[0])
        odom.pose.pose.position.y = float(mode.position[1])
        odom.pose.pose.position.z = float(mode.position[2])

        odom.pose.pose.orientation.w = float(mode.imu_state.quaternion[0])
        odom.pose.pose.orientation.x = float(mode.imu_state.quaternion[1])
        odom.pose.pose.orientation.y = float(mode.imu_state.quaternion[2])
        odom.pose.pose.orientation.z = float(mode.imu_state.quaternion[3])

        odom.twist.twist.linear.x = float(mode.velocity[0])
        odom.twist.twist.linear.y = float(mode.velocity[1])
        odom.twist.twist.linear.z = float(mode.velocity[2])
        odom.twist.twist.angular.z = float(mode.yaw_speed)

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = odom.pose.pose.position.x
            tf_msg.transform.translation.y = odom.pose.pose.position.y
            tf_msg.transform.translation.z = odom.pose.pose.position.z
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_bro.sendTransform(tf_msg)

    def state_cb(self, state: LowState):

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
            
        ]
        for i in range(12):
            joint_state.position.append(float(state.motor_state[i].q))

        self.joint_pub.publish(joint_state)


def main():
    rclpy.init()
    rclpy.spin(Driver())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
