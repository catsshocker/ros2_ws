#include "my_robot_base/motion/MoveTask.hpp"

#include <cmath>

constexpr double PI = 3.14159265358979323846;
constexpr double TURN_RATIO = 1.5; // Turn speed is 1.5 times the linear speed
constexpr double MOVE_AND_TURN_RATIO = 3; // Move and turn speed is 0.5 times the linear speed

MovvveTask::MovvveTask(double dx, double dy, double dtheta, double Vm, double Acc, double dt) {  // long:m angle:rad 
    this->i = 0;
    this->dt = dt;
    this->l = std::hypot(dx, dy);
    this->dtheta = dtheta;
    this->Vm = Vm;
    this->Acc = Acc;
    this->is_finished = false;
    this->car_angle = 0; // Assuming the robot starts facing along the x-axis
    this->move_angle = 0; // Will be set based on dx and dy
    
    if(std::abs(this->l) < 1e-6 && std::abs(this->dtheta) < 1e-6){  //no move or turn
        this->is_finished = true;
        return;
    }
    if(std::abs(this->dtheta) < 1e-6){                // only move
        this->move_type_ = move_type::MOVE;
        this->move_angle = std::atan2(dy, dx);
    }
    else if (std::abs(this->l) < 1e-6){             // only turn
        this->move_type_ = move_type::TURN;
        this->move_angle = 0; // Not used for pure turning
    }
    else{                               //move and turn
        this->move_type_ = move_type::MOVE_AND_TURN;
        this->move_angle = std::atan2(dy, dx);
    }
    
    switch (this->move_type_)
    {
    case move_type::MOVE:
        this->updown_speed = UpDownSpeed(this->l, Vm, Acc);
        break;
    case move_type::TURN:
        this->updown_speed = UpDownSpeed(this->dtheta, Vm * TURN_RATIO, Acc);
        break;
    case move_type::MOVE_AND_TURN:
        // 1. 計算時間同步所需的調整速度 (您原本優秀的邏輯)
        double t_linear = this->l / this->Vm;
        double t_angular = std::abs(dtheta) / (this->Vm * TURN_RATIO);

        double t_max = std::max(t_linear, t_angular);
        double Vm_adjusted = this->l / t_max; // 時間同步基準速度

        // 2. 【核心安全鎖】：利用「歸一化比例」計算絕對不超速的安全 Vm_safe
        // 這個公式完全不需要任何外部常數，僅依靠您傳入的規格！
        double Vm_safe = this->Vm / (1.0 + std::abs(dtheta) / (this->l * TURN_RATIO));

        // 3. 取兩者最小值：既保證時間同步，又絕對不會超出馬達極限
        double Vm_final = std::min(Vm_adjusted, Vm_safe);

        // 4. 等比例縮放加速度，保持加減速曲線的平滑形狀
        double Acc_final = this->Acc * (Vm_final / this->Vm);

        // 5. 建立運動規劃
        this->updown_speed = UpDownSpeed(this->l, Vm_final, Acc_final);
        break;
    }
    
}


geometry_msgs::msg::Twist MovvveTask::update(){
    geometry_msgs::msg::Twist cmd;
    switch (this->move_type_)
    {
    case move_type::MOVE:{
        float speed = this->updown_speed.update(this->dt, i);
        cmd.linear.x = speed * std::cos(this->move_angle);
        cmd.linear.y = speed * std::sin(this->move_angle);
        cmd.angular.z = 0;
        break;}
    case move_type::TURN:{
        float angular_speed = this->updown_speed.update(this->dt, i);
        cmd.linear.x = 0;
        cmd.linear.y = 0;
        cmd.angular.z = angular_speed;
        break;}
    case move_type::MOVE_AND_TURN:{
        float move_speed = this->updown_speed.update(this->dt, i);
        cmd.linear.x = move_speed * std::cos(this->move_angle-this->car_angle);
        cmd.linear.y = move_speed * std::sin(this->move_angle-this->car_angle);
        cmd.angular.z =  move_speed * (this->dtheta / this->l);
        this->car_angle = this->dtheta*this->updown_speed.getPR();
        break;}
    default:{
        break;}
    }
    i++;
    return cmd;
}

bool MovvveTask::finished(){
    return this->updown_speed.finished();
}


