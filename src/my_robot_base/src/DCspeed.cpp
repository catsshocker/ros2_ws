#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/Float64MultiArray.hpp"
#include <algorithm>

using namespace std::chrono_literals;

constexpr double MAX_MOTOR_SPEED = 480;

#define PI 3.14159265358979323846

constexpr double Ms_TO_DEGs = 3600.0 / PI;  // Conversion factor from m/s to degrees per second (deg/s) for a wheel with a radius of 1 meters
constexpr double Rads_TO_DEGs = 3.0 * 180.0 / PI; // Conversion factor from radians per second to motor degrees per second

constexpr double linear_factor = 1.0;
constexpr double angular_factor = 1.0;


class DCspeed: public rclcpp::Node{
public:
    DCspeed() : Node("DCspeed_node"){
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&DCspeed::callback,this,std::placeholders::_1)
        );

        motor_speed_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/speed_cmd", 10);
        RCLCPP_INFO(this->get_logger(), "DCspeed is running");
        
    }

private :
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_speed_pub_;

    void callback(const geometry_msgs::msg::Twist::SharedPtr msg){  // callback function for /cmd_vel to wheel speed
        double Vx = msg->linear.x;    // get x speed
        double Vy = msg->linear.y;    // get y speed
        double angular = msg->angular.z;    // get angle speed(anguler speed)

        double speed_m1 = -0.8660254038 * Vx * Ms_TO_DEGs*linear_factor + 0.5 * Vy * Ms_TO_DEGs*linear_factor + angular * Rads_TO_DEGs*angular_factor;
        double speed_m2 =  -Vy * Ms_TO_DEGs*linear_factor + angular * Rads_TO_DEGs*angular_factor;
        double speed_m3 =  0.8660254038 * Vx * Ms_TO_DEGs*linear_factor + 0.5 * Vy * Ms_TO_DEGs*linear_factor + angular * Rads_TO_DEGs*angular_factor;
        RCLCPP_INFO(this->get_logger(), "%.2f, %.2f, %.2f | %.2f, %.2f, %.2f",Vx,Vy,angular,speed_m1,speed_m2,speed_m3);

        if(std::abs(speed_m1) > MAX_MOTOR_SPEED || std::abs(speed_m2) > MAX_MOTOR_SPEED || std::abs(speed_m3) > MAX_MOTOR_SPEED){
            RCLCPP_WARN(this->get_logger(), "Calculated motor speeds exceed maximum limits. Clamping to max speed.");
        }
        setMotorSpeed(speed_m1, speed_m2, speed_m3);
    }

    void setMotorSpeed(double speed0, double speed1, double speed2){
        speed0 = std::clamp(speed0,-MAX_MOTOR_SPEED,MAX_MOTOR_SPEED);
        speed1 = std::clamp(speed1,-MAX_MOTOR_SPEED,MAX_MOTOR_SPEED);
        speed2 = std::clamp(speed2,-MAX_MOTOR_SPEED,MAX_MOTOR_SPEED);
        RCLCPP_INFO(this->get_logger(), "motor speeds set : %.2f, %.2f, %.2f", speed0, speed1, speed2);
        std_msgs::msg::Float64MultiArray motor_speeds;
        motor_speeds.data = {speed0, speed1, speed2};
        motor_speed_pub_->publish(motor_speeds);
    }
};

int main(int argc,char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<DCspeed>());
    rclcpp::shutdown();
    return 0;
}