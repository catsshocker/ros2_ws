#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/srv/move.hpp"

#include "my_robot_base/motion/MoveTask.hpp"

static constexpr double DEFAULT_VM = 0.4;
static constexpr double DEFAULT_ACC = 0.2;

static constexpr double CONTROL_FREQ = 50.0;
static constexpr double DT = 1.0 / CONTROL_FREQ;

using namespace std::chrono_literals;


class MotionController: public rclcpp::Node{
public:
    MotionController();
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Service<my_robot_interfaces::srv::Move>::SharedPtr service_;
    void move(
    const std::shared_ptr<my_robot_interfaces::srv::Move::Request> request,
    std::shared_ptr<my_robot_interfaces::srv::Move::Response> response);
    
};

MotionController::MotionController():Node("motion_controller"){
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    service_ = this->create_service<my_robot_interfaces::srv::Move>(
            "/movvve",
            std::bind(&MotionController::move,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "MotionController Service Ready");
}

void MotionController::move(
    const std::shared_ptr<my_robot_interfaces::srv::Move::Request> request,
    std::shared_ptr<my_robot_interfaces::srv::Move::Response> response)
{
    RCLCPP_INFO(this->get_logger(),"Move request: dx=%.2f dy=%.2f dtheta=%.2f",request->dx, request->dy, request->dtheta);

    MovvveTask movetask(request->dx, request->dy, request->dtheta,DEFAULT_VM,DEFAULT_ACC,0.02);

    while(!movetask.finished()){
        auto vel = movetask.update();
        cmd_pub_ -> publish(vel);
        rclcpp::sleep_for(20ms);
    }

    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
    
    RCLCPP_INFO(this->get_logger(),"Move end");

    response->success = true;
    response->message = "Motion completed";
}

 
int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<MotionController>());
    rclcpp::shutdown();
    return 0;
}