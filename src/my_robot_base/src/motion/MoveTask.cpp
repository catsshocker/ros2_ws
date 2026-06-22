#include "my_robot_base/motion/MoveTask.hpp"

#include <cmath>

MovvveTask::MovvveTask(double dx, double dy, double dtheta, double Vm, double Acc, double dt) {
    this->Vm = Vm;
    this->Acc = Acc;
    this->l = std::hypot(dx, dy);
    this->r = std::atan2(dy, dx); 
    this->dtheta = dtheta;
    this->now_speed = 0;
    this->now_angle = 0;
    this->dt = dt;
    this->i = 0;
    this->is_finished = false;

    // 梯形/三角形速度規劃邏輯 (Trapezoidal / Triangular Profile)
    // 臨界距離：剛好加速到 Vm 就立刻減速所需的總距離
    double d_critical = (Vm * Vm) / Acc;

    if (this->l >= d_critical) {
        // 情況 A：距離夠長，可以達到最大速度 Vm（梯形速度曲線）
        double t_acc = Vm / Acc;                               // 加速所需時間
        double t_constant = (this->l - d_critical) / Vm;       // 等速所需時間
        
        this->t1 = t_acc;
        this->t2 = t_acc + t_constant;
        this->t3 = this->t2 + t_acc;
        this->v_max_calc = Vm;
    } 
    else {
        // 情況 B：距離太短，還沒到 Vm 就得開始減速（三角形速度曲線）
        // 此時實際能達到的最高速度 v_peak = sqrt(l * Acc)
        double v_peak = std::sqrt(this->l * Acc);
        double t_acc = v_peak / Acc;
        
        this->t1 = t_acc;
        this->t2 = t_acc; // 沒有等速階段
        this->t3 = t_acc + t_acc;
        this->v_max_calc = v_peak;
    }
}


geometry_msgs::msg::Twist MovvveTask::update(){
    geometry_msgs::msg::Twist cmd;
    if(i*dt<=t1){
        now_speed = Acc * dt*i;
    }
    else if(i*dt <= t2){
        now_speed = v_max_calc;
    }
    else if(i*dt <= t3){
        now_speed = Acc * (t3 - i*dt);
    }
    else{
        this->is_finished = true;
        return cmd;
    }
    now_speed = std::clamp(now_speed,-v_max_calc,v_max_calc);
    cmd.linear.x = now_speed * std::cos(r);
    cmd.linear.y = now_speed * std::sin(r);
    cmd.angular.z = 0;
    i++;
    return cmd;
}

bool MovvveTask::finished(){
    return this->is_finished;
}


