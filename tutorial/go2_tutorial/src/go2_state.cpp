#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::placeholders;

// Go2State 節點：監控機器人狀態，並根據 odometry 訊息計算移動距離
class Go2State : public rclcpp::Node{
public:
    // 構造函數，初始化節點名稱並建立訂閱器
    Go2State(): Node("go2_state"){
        RCLCPP_INFO(this->get_logger(), "Go2State created!"); // 節點建立提示
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&Go2State::odom_cb, this, _1)); // 訂閱 odom topic

        last_x = last_y = 0.0; // 初始化最後位置
        is_first = true; // 標記是否為第一次接收訊息
        this->declare_parameter("distance", 0.5); // 宣告距離參數，預設值為 0.5
    }
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // odometry 訂閱器
    double last_x, last_y; // 上一次的位置座標
    bool is_first; // 是否為第一次接收 odometry 訊息

    // odometry 回呼函式，計算移動距離並輸出位置
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr odom){

        double x = odom->pose.pose.position.x; // 獲取當前 x 座標
        double y = odom->pose.pose.position.y; // 獲取當前 y 座標

        if (is_first){ // 如果是第一次接收訊息
            last_x = x;
            last_y = y;
            is_first = false;
            RCLCPP_INFO(this->get_logger(), "init:(%.2f, %.2f)", x, y); // 輸出初始位置
            return;
        }

        double distance_x = x - last_x; // 計算 x 軸移動距離
        double distance_y = y - last_y; // 計算 y 軸移動距離

        double distance = sqrt(pow(distance_x, 2) + pow(distance_y, 2)); // 計算總移動距離

        if ( distance > this->get_parameter("distance").as_double()){ // 如果移動距離超過設定值
            RCLCPP_INFO(this->get_logger(), "now:(%.2f, %.2f)", x, y); // 輸出當前位置
            last_x = x; // 更新最後位置
            last_y = y;
        }
    }
};

// 主程式進入點
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); // 初始化 ROS2 系統
  rclcpp::spin(std::make_shared<Go2State>()); // 執行 Go2Ctrl 節點，持續運作直到結束
  rclcpp::shutdown(); // 關閉 ROS2 系統
  return 0;
}