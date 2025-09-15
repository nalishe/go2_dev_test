#include "rclcpp/rclcpp.hpp" // ROS2 節點基礎函式庫
#include "go2_tutorial_inter/srv/cruising.hpp" // 自訂服務型別 Cruising
#include "geometry_msgs/msg/point.hpp" // ROS2 點座標訊息型別
#include "nav_msgs/msg/odometry.hpp" // ROS2 里程計訊息型別
#include "sport_model.hpp" // 運動模式 API 定義

using namespace std::placeholders;
using namespace std::chrono_literals;

// Go2CruisingService 節點：提供巡航服務，並訂閱 odometry 取得目前位置
class Go2CruisingService : public rclcpp::Node{
public:
    // 構造函數，初始化節點名稱並建立服務與訂閱器
    Go2CruisingService(): Node("go2_cruising_service"){
        RCLCPP_INFO(this->get_logger(), "Go2CruisingService created!"); // 節點建立提示
        this->declare_parameter("x", 0.1); // 宣告 x 參數，預設 0.1
        this->declare_parameter("y", 0.0); // 宣告 y 參數，預設 0.0
        this->declare_parameter("z", 0.5); // 宣告 z 參數，預設 0.5

        // 訂閱 odom topic，取得機器人目前位置
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&Go2CruisingService::odom_cb, this, _1));
        // 建立參數 client，連線到 go2_ctrl 節點
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "go2_ctrl");
        while (!param_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok()){
                return; // 如果 ROS2 系統已關閉，結束等待
            }
            RCLCPP_INFO(this->get_logger(), "loading...."); // 等待參數服務啟動
        }
        RCLCPP_INFO(this->get_logger(), "params client connect succes!"); // 參數 client 連線成功

        // 建立 cruising 服務
        cru_service_ = this->create_service<go2_tutorial_inter::srv::Cruising>(
            "cruising",
            std::bind(&Go2CruisingService::cru_cb, this, _1, _2)
        );
    }
private:
    rclcpp::AsyncParametersClient::SharedPtr param_client_; // 參數 client
    rclcpp::Service<go2_tutorial_inter::srv::Cruising>::SharedPtr cru_service_; // 巡航服務
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // odometry 訂閱器
    geometry_msgs::msg::Point current_point_; // 目前位置

    // 巡航服務回呼函式
    void cru_cb(const go2_tutorial_inter::srv::Cruising::Request::SharedPtr request,
                go2_tutorial_inter::srv::Cruising::Response::SharedPtr response){
        int32_t id;
        auto flag = request->flag; // 取得請求旗標
        if (flag != 0){
            id = ROBOT_SPORT_API_MOVE; // 啟動巡航
            RCLCPP_INFO(this->get_logger(), "start cruising!");
        }else{
            id = ROBOT_SPORT_API_STOPMOVE; // 停止巡航
            RCLCPP_INFO(this->get_logger(), "stop cruising!");
        }
        // 設定 go2_ctrl 節點的參數
        param_client_->set_parameters({
            this->get_parameter("x"),
            this->get_parameter("y"),
            this->get_parameter("z"),
            rclcpp::Parameter("sport_api_id",  id)
        });
        response->point = current_point_; // 回傳目前位置
    }

    // odometry 訊息回呼函式，更新目前位置
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr odom){
        current_point_ = odom->pose.pose.position;
    }
};

// 主程式進入點
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); // 初始化 ROS2 系統
  rclcpp::spin(std::make_shared<Go2CruisingService>()); // 執行 Go2CruisingService 節點，持續運作直到結束
  rclcpp::shutdown(); // 關閉 ROS2 系統
  return 0;
}