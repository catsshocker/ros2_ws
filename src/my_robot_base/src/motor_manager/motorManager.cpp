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

    this->parameter_callback_handle_ =
    this->add_on_set_parameters_callback(
        std::bind(&MotorManager::parameterEventCallback,
                  this,
                  std::placeholders::_1));

    this->updateParameters()

    this->declare_parameter<double>("wheel.kp", 1.0);
    this->declare_parameter<double>("wheel.ki", 0.0);
    this->declare_parameter<double>("wheel.kd", 0.1);
    this->declare_parameter<double>("wheel.max_speed", 480.0);

    this->declare_parameter<double>("updown.kp", 1.0);
    this->declare_parameter<double>("updown.ki", 0.0);
    this->declare_parameter<double>("updown.kd", 0.1);
    this->declare_parameter<double>("updown.max_enc", 480.0);
    this->declare_parameter<double>("updown.min_enc", -480.0);
    this->declare_parameter<bool>("updown.invert", false);

    motor0_pub_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_0/cmd", 10);
    motor1_pub_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_1/cmd", 10);
    motor2_pub_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_2/cmd", 10);
    motor3_pub_ = this->create_publisher<std_msgs::msg::Float64>("/titan0/m_3/cmd", 10);

    pid_controllers_[0].setGains(wheel_kp_, wheel_ki_, wheel_kd_); 
    pid_controllers_[1].setGains(wheel_kp_, wheel_ki_, wheel_kd_);
    pid_controllers_[2].setGains(wheel_kp_, wheel_ki_, wheel_kd_);
    pid_controllers_[3].setGains(updown_kp_, updown_ki_, updown_kd_); // updown rail
    for (int i = 0; i < 4; i++) {
        pid_controllers_[i].setOutputLimits(1.0, -1.0);
    }
    RCLCPP_INFO(this->get_logger(), "motor_manager is running!!");
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
        zero_speed_counter_= BRAKE_TIMEOUT_COUNTS + 1;          //let it not too more
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

    RCLCPP_INFO(this->get_logger(), "target_enc:%f, enc:%f,t_s:%f,error:%f,out:%f",target_enc_[0],encoder_counts_[0],target_speed_[0],target_enc_[0]-encoder_counts_[0],motor_outputs[0]);
    
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

void MotorManager::updateParameters() {
    this->wheel_kp_ = this->get_parameter("wheel.kp").as_double();
    this->wheel_ki_ = this->get_parameter("wheel.ki").as_double();
    this->wheel_kd_ = this->get_parameter("wheel.kd").as_double();
    this->max_motor_speed_ = this->get_parameter("wheel.max_speed").as_double();

    this->updown_kp_ = this->get_parameter("updown.kp").as_double();
    this->updown_ki_ = this->get_parameter("updown.ki").as_double();
    this->updown_kd_ = this->get_parameter("updown.kd").as_double();
    this->min_updown_enc_ = this->get_parameter("updown.min_enc").as_double();
    this->max_updown_enc_ = this->get_parameter("updown.max_enc").as_double();
    this->updown_invert_ = this->get_parameter("updown.invert").as_bool();
}

rcl_interfaces::msg::SetParametersResult MotorManager::parameterEventCallback(const std::vector<rclcpp::Parameter> &parameters) {
    for (const auto &param : parameters) {
        if (param.get_name() == "wheel.kp") {
            wheel_kp_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated wheel.kp to: %.2f", wheel_kp_);
        } else if (param.get_name() == "wheel.ki") {
            wheel_ki_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated wheel.ki to: %.2f", wheel_ki_);
        } else if (param.get_name() == "wheel.kd") {
            wheel_kd_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated wheel.kd to: %.2f", wheel_kd_);
        } else if (param.get_name() == "wheel.max_speed") {
            max_motor_speed_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated max_motor_speed to: %.2f", max_motor_speed_);
        }
        
        else if (param.get_name() == "updown.kp") {
            updown_kp_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated updown.kp to: %.2f", updown_kp_);
        } else if (param.get_name() == "updown.ki") {
            updown_ki_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated updown.ki to: %.2f", updown_ki_);
        } else if (param.get_name() == "updown.kd") {
            updown_kd_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated updown.kd to: %.2f", updown_kd_);
        }
    }
    this->updateWheelPIDs();
    this->updateUpdownPID();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}