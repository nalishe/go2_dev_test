#include "rclcpp/rclcpp.hpp" // ROS2 C++ 節點基礎函式庫
#include "unitree_api/msg/request.hpp" // Unitree API 請求訊息型別
#include "sport_model.hpp" // 運動模式 API 定義
#include "nlohmann/json.hpp" // JSON 處理函式庫

using namespace std::chrono_literals;

// Go2Ctrl 節點：控制機器人運動模式的核心節點
class Go2Ctrl : public rclcpp::Node{
public:
  // 構造函數，初始化節點名稱並建立參數與定時器
  Go2Ctrl(): Node("go2_ctrl"){
    RCLCPP_INFO(this->get_logger(), "Go2Ctrl created!"); // 節點建立提示
    req_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request",10); // 建立請求的發布器
    timer_ = this->create_wall_timer(100ms, std::bind(&Go2Ctrl::on_timer, this)); // 每100ms執行一次定時器回呼函式

    // 宣告參數，包含運動模式 ID 和三軸座標
    this->declare_parameter("sport_api_id", ROBOT_SPORT_API_BALANCESTAND);
    this->declare_parameter("x", 0.0);
    this->declare_parameter("y", 0.0);
    this->declare_parameter("z", 0.0);
  }

private:
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_; // 請求的發布器
  rclcpp::TimerBase::SharedPtr timer_; // 定時器

  // 定時器回呼函式，每100ms執行一次
  void on_timer(){
    auto id = this->get_parameter("sport_api_id").as_int(); // 獲取運動模式 ID

    unitree_api::msg::Request request; // 建立請求訊息

    request.header.identity.api_id = id; // 設定 API ID
    
    if (id == ROBOT_SPORT_API_MOVE){ // 如果是移動模式，設定參數
      nlohmann::json js;
      js["x"] = this->get_parameter("x").as_double();
      js["y"] = this->get_parameter("y").as_double();
      js["z"] = this->get_parameter("z").as_double();
      request.parameter = js.dump(); // 將參數轉換為 JSON 字串
    }

    req_pub_->publish(request); // 發布請求
  }
};

// 主程式進入點
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); // 初始化 ROS2 系統
  rclcpp::spin(std::make_shared<Go2Ctrl>()); // 執行 Go2Ctrl 節點，持續運作直到結束
  rclcpp::shutdown(); // 關閉 ROS2 系統
  return 0;
}