#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "go2_tutorial_inter/action/nav.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sport_model.hpp"


using namespace std::placeholders;
using namespace rclcpp_action;
using namespace std::chrono_literals;


// Go2NavServer 節點：提供導航功能的行動伺服器
class Go2NavServer : public rclcpp::Node{
public:
    // 構造函數，初始化節點名稱並建立伺服器與訂閱器
    Go2NavServer(): Node("go2_nav_server"){
        RCLCPP_INFO(this->get_logger(), "Go2NavServer created!"); // 節點建立提示
        
        // 建立參數 client，連線到 go2_ctrl 節點
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "go2_ctrl");
        while (!param_client_->wait_for_service(1s)){
            if (!rclcpp::ok()){
                return; // 如果 ROS2 系統已關閉，結束等待
            }
            RCLCPP_INFO(this->get_logger(), "loading...."); // 等待參數服務啟動
        }

        // 建立導航行動伺服器
        nav_server_ = rclcpp_action::create_server<go2_tutorial_inter::action::Nav>(
            this,
            "nav",
            std::bind(&Go2NavServer::goal_cb, this, _1, _2), // 目標回呼函式
            std::bind(&Go2NavServer::cancel_cb, this, _1), // 取消回呼函式
            std::bind(&Go2NavServer::accepted_cb, this, _1) // 接受回呼函式
        );

        // 訂閱 odom topic，取得機器人目前位置
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&Go2NavServer::odom_cb, this, _1)
        );

        // 宣告參數，包含導航目標誤差範圍
        this->declare_parameter("x", 0.1);
        this->declare_parameter("error", 0.2);

    }
private:
    rclcpp_action::Server<go2_tutorial_inter::action::Nav>::SharedPtr nav_server_; // 導航伺服器
    rclcpp::AsyncParametersClient::SharedPtr param_client_; // 參數 client

    geometry_msgs::msg::Point current_point_, start_point_; // 目前位置與起始位置

    // 目標回呼函式，處理導航目標請求
    GoalResponse goal_cb(const GoalUUID &uuid, std::shared_ptr<const go2_tutorial_inter::action::Nav::Goal> goal){
        float goal_dis = goal->goal;
        (void)uuid; 
        if (goal_dis > 0.0){
            start_point_ = current_point_; // 設定起始位置
            RCLCPP_INFO(this->get_logger(), "Goal accepted");
            param_client_->set_parameters({
                rclcpp::Parameter("sport_api_id", ROBOT_SPORT_API_MOVE),
                this->get_parameter("x")
       
            });

            
            return GoalResponse::ACCEPT_AND_EXECUTE; // 接受目標
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Goal rejected");
            return GoalResponse::REJECT; // 拒絕目標
        }

    };

    // 取消回呼函式，處理取消導航請求
    CancelResponse cancel_cb(std::shared_ptr<ServerGoalHandle<go2_tutorial_inter::action::Nav>> server_go_handler){
        (void)server_go_handler;
        RCLCPP_INFO(this->get_logger(), "Goal cancel request received");
        stop_move(); // 停止移動
        
        return CancelResponse::ACCEPT; // 接受取消請求
    };

    // 停止移動的方法
    void stop_move(){
        param_client_->set_parameters({
            rclcpp::Parameter("sport_api_id", ROBOT_SPORT_API_STOPMOVE),
            rclcpp::Parameter("x", 0.0),
            rclcpp::Parameter("y", 0.0),
            rclcpp::Parameter("z", 0.0)
        });
    }

    // 接受回呼函式，處理接受的導航請求
    void accepted_cb(std::shared_ptr<ServerGoalHandle<go2_tutorial_inter::action::Nav>> server_goal_handle){
        std::thread(std::bind(&Go2NavServer::execute, this, _1), server_goal_handle).detach();

    }

    // 執行導航的主要邏輯
    void execute(std::shared_ptr<ServerGoalHandle<go2_tutorial_inter::action::Nav>> server_goal_handle){
        rclcpp::Rate rate(1.0); // 設定執行頻率
        auto feedback = std::make_shared<go2_tutorial_inter::action::Nav::Feedback>();
        auto nav_result = std::make_shared<go2_tutorial_inter::action::Nav::Result>();
        while(rclcpp::ok()){

            auto dis_x = current_point_.x - start_point_.x;
            auto dis_y = current_point_.y - start_point_.y;
            auto dis = sqrt(dis_x*dis_x + dis_y*dis_y); // 計算距離

            auto distance = server_goal_handle->get_goal()->goal - dis;

            feedback->distance = distance; // 更新回饋距離
            server_goal_handle->publish_feedback(feedback); // 發布回饋

            if (server_goal_handle->is_canceling()){ // 如果取消請求
                stop_move();
                nav_result->point = current_point_;
                server_goal_handle->canceled(nav_result);
                return;
            }

            if (distance <= this->get_parameter("error").as_double()){ // 如果到達目標
                RCLCPP_INFO(this->get_logger(), "Goal reached");
                stop_move();
                break;
            }
            rate.sleep();
            
            
        
        
        }
        if (rclcpp::ok()){
                stop_move();
                nav_result->point = current_point_;
                server_goal_handle->succeed(nav_result); // 導航成功
        }

    }


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // odometry 訂閱器

    // odometry 回呼函式，更新目前位置
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr odom){
        current_point_ = odom->pose.pose.position;
    }   


};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); // 初始化 ROS2 系統
  rclcpp::spin(std::make_shared<Go2NavServer>()); // 執行 Go2Ctrl 節點，持續運作直到結束
  rclcpp::shutdown(); // 關閉 ROS2 系統
  return 0;
}