


#include "rclcpp/rclcpp.hpp" // ROS2 C++ 節點基礎函式庫
#include "unitree_api/msg/request.hpp" // Unitree API 請求訊息型別
#include "geometry_msgs/msg/twist.hpp" // ROS2 Twist 訊息型別（速度指令）
#include "sport_model.hpp" // 運動模式 API 定義
#include "nlohmann/json.hpp" // JSON 處理函式庫

using namespace std::placeholders;


// TwistBridge 節點：將/cmd_vel速度指令轉換為Unitree API請求，發佈給機器人底層
class TwistBridge : public rclcpp::Node{
public:
    TwistBridge(): Node("my_node"){
        // 節點初始化時，顯示提示訊息
        RCLCPP_INFO(this->get_logger(), "TwistBridge created!");

        // 建立Publisher，發佈Request到Unitree API
        request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request",10);
        // 建立Subscriber，訂閱/cmd_vel速度指令
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10, std::bind(&TwistBridge::twist_cb, this, _1));
    }

private:
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_; // 請求發佈器
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_; // 速度指令訂閱器

    // 當收到/cmd_vel訊息時的回呼函式
    void twist_cb(const geometry_msgs::msg::Twist::SharedPtr twist){
        unitree_api::msg::Request request; // 建立請求訊息

        // 取得Twist中的線速度與角速度
        double x = twist->linear.x;
        double y = twist->linear.y;
        double z = twist->angular.z;

        // 預設API為平衡站立
        auto api_id = ROBOT_SPORT_API_BALANCESTAND;

        // 若有速度指令則切換為移動模式，並將速度參數包裝成JSON
        if(x != 0 || y != 0 || z != 0){
            api_id = ROBOT_SPORT_API_MOVE;

            nlohmann::json js;
            js['x'] = x; // 線速度x
            js['y'] = y; // 線速度y
            js['z'] = z; // 角速度z

            request.parameter = js.dump(); // 轉成字串存入parameter
        }
        // 設定API ID
        request.header.identity.api_id = api_id;
        // 發佈請求
        request_pub_->publish(request);
    }
};


// 主程式進入點
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv); // 初始化ROS2
    rclcpp::spin(std::make_shared<TwistBridge>()); // 執行TwistBridge節點
    rclcpp::shutdown(); // 關閉ROS2
    return 0;
}
