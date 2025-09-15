// 匯入 ROS2 節點、訊息型別與 TF 廣播相關標頭檔
#include <rclcpp/rclcpp.hpp> // ROS2 節點基礎函式庫
#include <nav_msgs/msg/odometry.hpp> // ROS2 里程計訊息型別
#include <unitree_go/msg/sport_mode_state.hpp> // Unitree 四足機器人運動狀態訊息型別
#include <geometry_msgs/msg/transform_stamped.hpp> // ROS2 TF 轉換訊息型別
#include <tf2_ros/transform_broadcaster.h> // ROS2 TF 廣播器
#include <sensor_msgs/msg/joint_state.hpp> // ROS2 關節狀態訊息型別
#include <unitree_go/msg/low_state.hpp> // Unitree 四足機器人低階狀態訊息型別

using namespace std::placeholders; // 用於綁定回呼函式參數

// Driver 節點：負責接收四足機器人運動狀態，並轉換為 ROS2 odometry 與 TF 座標
class Driver : public rclcpp::Node{
public:
  // 建構子：初始化節點、參數、Publisher、Subscriber、TF 廣播器
  Driver(): Node("driver"){
    RCLCPP_INFO(this->get_logger(), "Driver created!");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base");
    this->declare_parameter("publish_tf", true);

    odom_frame = this->get_parameter("odom_frame").as_string();
    base_frame = this->get_parameter("base_frame").as_string();
    publish_tf = this->get_parameter("publish_tf").as_bool();

    tf_bro_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    mode_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "/lf/sportmodestate",
      10,
      std::bind(&Driver::mode_cb, this, _1)
    );

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
      "/lf/lowstate",
      10,
      std::bind(&Driver::low_state_cb, this, _1)
    );




  }
private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_bro_;

  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr mode_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;



  std::string odom_frame, base_frame;
  bool publish_tf;


  void low_state_cb(const unitree_go::msg::LowState::SharedPtr low_state){
    sensor_msgs::msg::JointState joint_state;

    joint_state.header.stamp = this->now();
    joint_state.name = {
      "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
      
    };
    
    for(size_t i = 0; i < 12; i++){
      auto motor = low_state->motor_state[i];
      joint_state.position.push_back(motor.q);
    }
    
    joint_state_pub_->publish(joint_state);

  }


  void mode_cb(const unitree_go::msg::SportModeState::SharedPtr mode){
    nav_msgs::msg::Odometry odom;

    odom.header.stamp.sec = mode->stamp.sec;
    odom.header.stamp.nanosec = mode->stamp.nanosec;

    odom.header.frame_id = odom_frame;
    odom.child_frame_id = base_frame;

    odom.pose.pose.position.x = mode->position[0];
    odom.pose.pose.position.y = mode->position[1];
    odom.pose.pose.position.z = mode->position[2];

    odom.pose.pose.orientation.w = mode->imu_state.quaternion[0];
    odom.pose.pose.orientation.x = mode->imu_state.quaternion[1];
    odom.pose.pose.orientation.y = mode->imu_state.quaternion[2];
    odom.pose.pose.orientation.z = mode->imu_state.quaternion[3];

    odom.twist.twist.linear.x = mode->velocity[0];
    odom.twist.twist.linear.y = mode->velocity[1];
    odom.twist.twist.linear.z = mode->velocity[2];
    
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0; 
    odom.twist.twist.angular.z = mode->yaw_speed;
    
    odom_pub_->publish(odom);

    if(!publish_tf){
      return;
    } 
    
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = this->now();
    transform.header.frame_id = odom_frame;
    transform.child_frame_id = base_frame;

    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;

    transform.transform.rotation = odom.pose.pose.orientation;





    tf_bro_->sendTransform(transform);
  }

};

// 主程式進入點
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); // 初始化 ROS2 系統
  rclcpp::spin(std::make_shared<Driver>()); // 執行 Driver 節點，持續運作直到結束
  rclcpp::shutdown(); // 關閉 ROS2 系統
  return 0;
}
