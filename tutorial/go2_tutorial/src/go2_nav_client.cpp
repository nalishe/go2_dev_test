#include "rclcpp/rclcpp.hpp" // ROS2 節點基礎函式庫
#include "rclcpp_action/rclcpp_action.hpp" // ROS2 Action 客戶端函式庫
#include "go2_tutorial_inter/action/nav.hpp" // 自訂導航 Action 型別

using namespace std::chrono_literals;
using namespace std::placeholders;

// Go2NavClient 節點：負責發送導航目標給 Action Server 並接收回饋
class Go2NavClient : public rclcpp::Node{
public:
    Go2NavClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("go2_nav_client", options){
        RCLCPP_INFO(this->get_logger(), "Go2NavClient created!"); // 節點建立提示
        // 建立 Action Client，連線到 nav action server
        client_ = rclcpp_action::create_client<go2_tutorial_inter::action::Nav>(this, "nav");
    }
    // 連線到 Action Server，若失敗則重試
    bool connect_server(){
        while (!client_->wait_for_action_server(1s))
        {
          if (!rclcpp::ok())
          {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the action server. Exiting.");
            return false;
          }
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for action server to be available...");
        }
        RCLCPP_INFO(this->get_logger(), "Connected to server");
        return true;
    }

    // 發送導航目標請求
    void send_request(float x){
      go2_tutorial_inter::action::Nav::Goal goal;
      goal.goal = x; // 設定目標值
      rclcpp_action::Client<go2_tutorial_inter::action::Nav>::SendGoalOptions options;
      // 設定目標回應、回饋、結果的 callback
      options.goal_response_callback = std::bind(&Go2NavClient::goal_response_callback, this, _1);
      options.feedback_callback = std::bind(&Go2NavClient::feedback_callback, this, _1, _2);
      options.result_callback = std::bind(&Go2NavClient::result_callback, this, _1);
      client_->async_send_goal(goal, options); // 非同步發送目標
      RCLCPP_INFO(this->get_logger(), "Sending goal: %.2f", x);
    }

    ~Go2NavClient(){
      client_->async_cancel_all_goals(); // 節點結束時取消所有目標
    }

private:
    rclcpp_action::Client<go2_tutorial_inter::action::Nav>::SharedPtr client_; // Action Client 指標

    // 目標回應 callback
    void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<go2_tutorial_inter::action::Nav>> goal_handle){
      if (goal_handle){
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
        rclcpp::shutdown();
      }
    }

    // 回饋 callback，持續接收導航距離
    void feedback_callback(rclcpp_action::ClientGoalHandle<go2_tutorial_inter::action::Nav>::SharedPtr goal_handle,
        const std::shared_ptr<const go2_tutorial_inter::action::Nav::Feedback> feedback){
            (void)goal_handle;
            RCLCPP_INFO(this->get_logger(), "Distance feedback: %.2f", feedback->distance);
    }
        
    // 結果 callback，收到導航結果後處理
    void result_callback(const rclcpp_action::ClientGoalHandle<go2_tutorial_inter::action::Nav>::WrappedResult &result){
      switch (result.code)
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal was succeeded! (%.2f, %.2f)", result.result->point.x, result.result->point.y);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_INFO(this->get_logger(), "Goal failed with code: %d", static_cast<int>(result.code));
        break;
      }
      rclcpp::shutdown(); // 結束後關閉 ROS2
    }
};

// 主程式進入點
int main(int argc, char * argv[])
{
  if (argc != 2){
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Please provide one float argument");
    return 1;
  }

  rclcpp::init(argc, argv); // 初始化 ROS2 系統
  auto nav_client = std::make_shared<Go2NavClient>(); // 建立導航客戶端
  auto flag = nav_client->connect_server(); // 連線到 action server
  if (!flag){
    RCLCPP_ERROR(nav_client->get_logger(), "Cannot connect to server");
    return 1;
  }
  nav_client->send_request(atof(argv[1])); // 發送導航目標
  rclcpp::spin(nav_client); // 執行節點，持續運作直到結束
  rclcpp::shutdown(); // 關閉 ROS2 系統
  return 0;
}