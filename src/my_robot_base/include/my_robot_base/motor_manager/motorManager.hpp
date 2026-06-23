#ifndef MOTOR_MANAGER_HPP
#define MOTOR_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "pid.h"

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

    PIDController pid_controllers_[4];

    void speedCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void encoderCallback(const std_msgs::msg::Float64::SharedPtr msg,int id);
    void controlLoop();

};

#endif