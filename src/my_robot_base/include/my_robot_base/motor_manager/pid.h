#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController{
public:
    PIDController(double kp, double ki, double kd, double max_output, double min_output)
        : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output), min_output_(min_output),
          integral_(0.0), prev_error_(0.0) {}

    PIDController() : PIDController(0.0, 0.0, 0.0, 0.0, 0.0) {}

    double update(double target, double current, double dt) {
        double error = target - current;
        if (dt <= 0.0) return 0.0;

        // 比例項 (P)
        double p_out = kp_ * error;

        // 積分項 (I)
        integral_ += error * dt;
        double i_out = ki_ * integral_;

        // 微分項 (D)
        double derivative = (error - prev_error_) / dt;
        double d_out = kd_ * derivative;

        // 總輸出
        double output = p_out + i_out + d_out;

        // 5. 輸出限幅 (防暴衝) 與 抗積分飽和 (Anti-windup)
        if (output > max_output_) {
            output = max_output_;
            // 如果輸出爆了，就扣回這次的積分，避免積分無限滾雪球
            integral_ -= error * dt; 
        } else if (output < min_output_) {
            output = min_output_;
            integral_ -= error * dt;
        }

        // 保存這次的誤差給下次微分用
        prev_error_ = error;

        return output;
    }

    // 重設 PID 內部狀態 
    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
    }

    // 允許動態調整參數 (調參時很好用)
    void setGains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void setOutputLimits(double max_output, double min_output) {
        max_output_ = max_output;
        min_output_ = min_output;
    }

private:
    // PID 參數
    double kp_;
    double ki_;
    double kd_;
    
    // 限幅範圍 (例如 PWM 的 100 到 -100)
    double max_output_;
    double min_output_;

    // 內部累積變數
    double integral_;
    double prev_error_;
};

#endif // PID_CONTROLLER_HPP
