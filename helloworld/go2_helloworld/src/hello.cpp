#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

using namespace std::chrono_literals;

class HelloWorld : public rclcpp::Node{
public:
    HelloWorld(): Node("my_node"){
        RCLCPP_INFO(this->get_logger(), "Hello World!");
        
        pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        timer_ = this->create_wall_timer(1s, std::bind(&HelloWorld::on_timer, this));

    }

private:
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    void on_timer(){
        
        unitree_api::msg::Request request;
        request.header.identity.api_id = 1016;
        pub_->publish(request);

    }
};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HelloWorld>());
    rclcpp::shutdown();
    return 0;
}