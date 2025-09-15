#include "rclcpp/rclcpp.hpp"
#include "go2_tutorial_inter/srv/cruising.hpp"

using namespace std::chrono_literals;

// 定義一個 Go2CruisingClient 類別，繼承自 rclcpp::Node
class Go2CruisingClient : public rclcpp::Node{
public:
    // 構造函數，初始化節點名稱並創建服務客戶端
    Go2CruisingClient(): Node("go2_cruising_client"){
        RCLCPP_INFO(this->get_logger(), "Go2CruisingClient created!");
        cru_client_ = this->create_client<go2_tutorial_inter::srv::Cruising>("cruising");
    }

    // 連接服務器的方法，等待服務可用
    bool connect_server(){
      while (!cru_client_->wait_for_service(1s))
      {
        if (!rclcpp::ok()){
          return false; // 如果 ROS2 系統已關閉，返回 false
        }
        RCLCPP_INFO(this->get_logger(), "service connecting...");
      }
      return true; // 服務可用時返回 true
    }

    // 發送請求的方法，傳入一個整數參數 flag
    rclcpp::Client<go2_tutorial_inter::srv::Cruising>::FutureAndRequestId send_request(int32_t flag){
      auto req_ = std::make_shared<go2_tutorial_inter::srv::Cruising_Request>();
      req_->flag = flag; // 設置請求的 flag 參數
      return cru_client_->async_send_request(req_); // 發送異步請求
    }

private:
    // 定義服務客戶端的共享指針
    rclcpp::Client<go2_tutorial_inter::srv::Cruising>::SharedPtr cru_client_;
};

// 主函數，程序入口點
int main(int argc, char * argv[])
{
  if (argc != 2){
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "please commit an int"); // 檢查參數數量是否正確
    return 1;
  }

  rclcpp::init(argc, argv); // 初始化 ROS2 系統

  auto client_ = std::make_shared<Go2CruisingClient>(); // 創建 Go2CruisingClient 節點
  auto flag = client_->connect_server(); // 嘗試連接服務器

  if (!flag){
    return 1; // 如果連接失敗，退出程序
  }

  auto response_future = client_->send_request(atoi(argv[1])); // 發送請求，參數來自命令行

  // 等待請求結果
  if(rclcpp::spin_until_future_complete(client_, response_future) == rclcpp::FutureReturnCode::SUCCESS){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SUCCESS"); // 請求成功
    auto response_ = response_future.get(); // 獲取響應
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "robot at:(%.2f, %.2f)", response_->point.x, response_->point.y); // 輸出響應結果
  }
  else{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FAIL"); // 請求失敗
  }

  rclcpp::shutdown(); // 關閉 ROS2 系統
  return 0;
}