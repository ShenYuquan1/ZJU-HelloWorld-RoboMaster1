#ifndef __PID_HPP__
#define __PID_HPP__

class PIDController {
private:
    float kp_, ki_, kd_;        // PID参数
    float integral_;            // 积分项
    float prev_error_;          // 上一次误差
    float output_limit_;        // 输出限幅
    float integral_limit_;      // 积分限幅
    
public:
    // 构造函数
    PIDController(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f, 
                  float output_limit = 10000.0f, float integral_limit = 5000.0f);
    
    // 设置PID参数
    void SetParameters(float kp, float ki, float kd, 
                       float output_limit = 10000.0f, float integral_limit = 5000.0f);
    
    // PID计算
    float Calculate(float target, float actual, float dt);
    
    // 重置PID状态
    void Reset();
    
    // 获取参数
    float GetKp() const { return kp_; }
    float GetKi() const { return ki_; }
    float GetKd() const { return kd_; }
};

#endif