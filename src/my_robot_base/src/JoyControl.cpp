#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

#define SPEED_LINEAR_MAX 0.800
#define SPEED_ANGULAR_MAX 0.1
#define LINEAR_ACC 1.0
#define ANGLAR_ACC 1.0

static constexpr double CONTROL_FREQ = 50.0;
static constexpr double DT = 1.0 / CONTROL_FREQ;



class SlewRateLimiter{
public:
    double Vm,d_Acc;
    double current; 

    SlewRateLimiter() : Vm(0.0), d_Acc(0.0), current(0.0) {}

    SlewRateLimiter(double Vm ,double Acc, double dt){ // Vm:m/s, Acc m/s2, dt:s
        this->Vm = Vm;
        this->d_Acc = Acc * dt;
        current = 0;
    }

    double calculate(double target){
        target = std::clamp(target,-Vm,Vm);
        if(target-current > d_Acc){
            current += d_Acc;
        }
        else if (target - current < -d_Acc){
            current -= d_Acc;
        }
        else{
            current = target;
        }
        return current;
    }
};

class JoyController : public rclcpp::Node
{
public:
    JoyController():Node("joy_controller"){
        pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
        sub_ = create_subscription<sensor_msgs::msg::Joy>("/joy",10,std::bind(&JoyController::joyCallback,this,std::placeholders::_1));

        lim_x = SlewRateLimiter(SPEED_LINEAR_MAX,LINEAR_ACC,DT);
        lim_y = SlewRateLimiter(SPEED_LINEAR_MAX,LINEAR_ACC,DT);
        lim_th = SlewRateLimiter(SPEED_ANGULAR_MAX,ANGLAR_ACC,DT);

        target_x = 0;
        target_y = 0;
        target_theta = 0;

        timer_ = create_wall_timer(
            std::chrono::milliseconds(
                static_cast<int>(1000.0 / CONTROL_FREQ)
            ),
            std::bind(&JoyController::controlLoop, this)
        );
        RCLCPP_INFO(get_logger(),"Joy Controller Start");
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    double target_x,target_y,target_theta;
    SlewRateLimiter lim_x,lim_y,lim_th;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)  // read joystick data to target velocity
    {
        auto deadzone = [](double x){return std::abs(x) < 0.05 ? 0.0 : x;};

        this->target_x = deadzone(msg->axes[1]) * SPEED_LINEAR_MAX;
        this->target_y = deadzone(msg->axes[0]) * SPEED_LINEAR_MAX;
        this->target_theta = deadzone(msg->axes[0]) * SPEED_ANGULAR_MAX;
    }

    void controlLoop(){  // publish data to /cmd_vel 
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = lim_x.calculate(target_x);
        cmd.linear.y = lim_y.calculate(target_y);
        cmd.angular.z = lim_th.calculate(target_theta);
        pub_->publish(cmd);
    }

};

int main(int argc,char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<JoyController>());
    rclcpp::shutdown();
    return 0;
}
