#ifndef MOTOR_MANAGER_HPP
#define MOTOR_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "my_robot_base/lib/pid.h"

#define ENCODER_COUNT 1464.0
constexpr double DEG_TO_ENC = ENCODER_COUNT / 360;
constexpr double DT = 0.020;
constexpr int BRAKE_TIMEOUT_COUNTS = 25;

class MotorManager:public rclcpp::Node{
public:
    MotorManager();

private:
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr enc0_sub_, enc1_sub_, enc2_sub_, enc3_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor0_pub_, motor1_pub_, motor2_pub_, motor3_pub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;

    double encoder_counts_[4] = {0, 0, 0, 0};
    double target_enc_[4] = {0, 0, 0, 0}; // encoder counts to speed conversion will be needed
    double target_speed_[4] = {0, 0, 0, 0};  // deg/s
    double d_enc_[4] = {0, 0, 0, 0}; // 這裡存放每個馬達的編碼器增量

    int zero_speed_counter_ = 0;
    bool enc_initialized_[4] = {false, false, false, false};

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    double wheel_kp_, wheel_ki_, wheel_kd_;
    double updown_kp_, updown_ki_, updown_kd_;
    double max_motor_speed_;
    double min_updown_enc_, max_updown_enc_;
    bool updown_invert_;

    PIDController pid_controllers_[4];

    void speedCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void encoderCallback(const std_msgs::msg::Float64::SharedPtr msg,int id);
    void controlLoop();
    void updateParameters() ;
    rcl_interfaces::msg::SetParametersResult parameterEventCallback(const std::vector<rclcpp::Parameter> &parameters);
    void updateWheelPIDs(){
        for(int i=0;i<=2;i++){
            pid_controllers_[i].setGains(this->wheel_kp_,this->wheel_ki_,this->wheel_kd_);
        }
    }
    void updateUpdownPID(){
        pid_controllers_[3].setGains(this->wheel_kp_,this->wheel_ki_,this->wheel_kd_);
    }

};

#endif