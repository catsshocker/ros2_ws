#include "my_robot_base/motor_manager/motorManager.hpp"

MotorManager::MotorManager(): Node("motor_manager"){
    speed_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/speed_cmd", 10, std::bind(&MotorManager::speedCallback, this, std::placeholders::_1)
    );

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&MotorManager::controlLoop, this)
    );

    enc0_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/titan0/m_0/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { encoderCallback(msg, 0); }
    );
    enc1_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/titan0/m_1/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { encoderCallback(msg, 1); }
    );
    enc2_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/titan0/m_2/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { encoderCallback(msg, 2); }
    );
    enc3_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/titan0/m_3/encoder", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { encoderCallback(msg, 3); }
    );

    motor0_pub_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_0/cmd", 10);
    motor1_pub_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_1/cmd", 10);
    motor2_pub_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_2/cmd", 10);
    motor3_pub_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_3/cmd", 10);

    pid_controllers_[0].setGains(1.0, 0.0, 0.1); 
    pid_controllers_[1].setGains(1.0, 0.0, 0.1);
    pid_controllers_[2].setGains(1.0, 0.0, 0.1);
    pid_controllers_[3].setGains(1.0, 0.0, 0.1); // updown rail
    for (int i = 0; i < 4; i++) {
        pid_controllers_[i].setOutputLimits(1.0, -1.0);
    }
}

void MotorManager::encoderCallback(const std_msgs::msg::Float64::SharedPtr msg,int id)
{
    encoder_counts_[id]=msg->data;

    if(!enc_initialized_[id]){
        target_enc_[id]=encoder_counts_[id];
        enc_initialized_[id]=true;
    }
}


void MotorManager::speedCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){  // deg/s
    if (msg->data.size() >= 3) {
        target_speed_[0] = msg->data[0];
        target_speed_[1] = msg->data[1];
        target_speed_[2] = msg->data[2];

        d_enc_[0] = target_speed_[0] * DEG_TO_ENC * DT;
        d_enc_[1] = target_speed_[1] * DEG_TO_ENC * DT;
        d_enc_[2] = target_speed_[2] * DEG_TO_ENC * DT;

    } else {
        RCLCPP_WARN(this->get_logger(), "Received speed command with insufficient data.");
    }
}

void MotorManager::controlLoop(){
    if(!(enc_initialized_[0]
        && enc_initialized_[1]
        && enc_initialized_[2]
        && enc_initialized_[3]
    )){return;}
    double motor_outputs[4];
    if(
        std::abs(target_speed_[0])<1e-6 && 
        std::abs(target_speed_[1])<1e-6 &&
        std::abs(target_speed_[2])<1e-6
    ){
        zero_speed_counter_++;
    }
    else{
        zero_speed_counter_ = 0;
    }
    // motor for wheel
    if (zero_speed_counter_ > BRAKE_TIMEOUT_COUNTS){
        zero_speed_counter_--;          //let it not too more
        for(int i = 0;i<=2;i++){
            target_enc_[i] = encoder_counts_[i];
            pid_controllers_[i].reset();    // 清空積分
            motor_outputs[i] = 0.0;
        }
    }
    else {
        for(int i = 0;i<=2;i++){
            target_enc_[i] += d_enc_[i];
            motor_outputs[i] = pid_controllers_[i].update(target_enc_[i],encoder_counts_[i], DT);
        }
    }

    
    //updown rail (special)
    target_enc_[3] += d_enc_[3];
    motor_outputs[3] = pid_controllers_[3].update(target_enc_[3], encoder_counts_[3], DT);

    std_msgs::msg::Float64 motor_msg;
    motor_msg.data = motor_outputs[0];
    motor0_pub_->publish(motor_msg);
    
    motor_msg.data = motor_outputs[1];
    motor1_pub_->publish(motor_msg);
    
    motor_msg.data = motor_outputs[2];
    motor2_pub_->publish(motor_msg);
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}