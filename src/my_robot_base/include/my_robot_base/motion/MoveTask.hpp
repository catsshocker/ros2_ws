#pragma once

#include "geometry_msgs/msg/twist.hpp"

class MovvveTask{
public:
    MovvveTask(
        double dx,
        double dy,
        double dtheta,
        double Vm,
        double Acc,
        double dt
    );
    bool finished();
    geometry_msgs::msg::Twist update();
private:
    double l , r , dtheta ,dt,Vm,Acc,v_max_calc;
    int i;
    double t1,t2,t3;
    double now_speed,now_angle;
    bool is_finished;
};

