#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>

using namespace std::chrono_literals;

#define MAX_MOTOR_SPEED 120.0
#define MIN_MOTOR_SPEED -120.0

class DCspeed: public rclcpp::Node{
public:
    DCspeed() : Node("DCspeed_node"){
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&DCspeed::callback,this,std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "DCspeed is running");
        
    }

private :

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    void callback(const geometry_msgs::msg::Twist::SharedPtr msg){  // callback function for /cmd_vel to wheel speed
        double Vx = msg->linear.x;    // get x speed
        double Vy = msg->linear.y;    // get y speed
        double angular = msg->angular.z;    // get angle speed(anguler speed)

        double speed_m1 = -0.8660254038 * Vx + 0.5 * Vy + angular;
        double speed_m2 =  -Vy + angular;
        double speed_m3 =  0.8660254038 * Vx + 0.5 * Vy + angular;
        RCLCPP_INFO(this->get_logger(), "%.2f, %.2f, %.2f | %.2f, %.2f, %.2f",Vx,Vy,angular,speed_m1,speed_m2,speed_m3);

        setMotorSpeed(1,speed_m1);
        setMotorSpeed(2,speed_m2);
        setMotorSpeed(3,speed_m3);
    }

    void setMotorSpeed(int motor ,double speed){
        speed = std::clamp(speed,MIN_MOTOR_SPEED,MAX_MOTOR_SPEED);      //motor is for car not for port -> motor1 is motor port 0
        switch (motor)
        {
        case 1:
            RCLCPP_INFO(this->get_logger(), "motor1 set : %f",speed);
            break;
        case 2:
            RCLCPP_INFO(this->get_logger(), "motor2 set : %f",speed);
            break;
        case 3:
            RCLCPP_INFO(this->get_logger(), "motor3 set : %f",speed);
            break;
        default:
            break;
        }
    }
};

int main(int argc,char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<DCspeed>());
    rclcpp::shutdown();
    return 0;
}