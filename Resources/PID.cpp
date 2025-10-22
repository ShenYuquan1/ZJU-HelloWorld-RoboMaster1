#include "PID.hpp"

PIDController::PIDController(float kp, float ki, float kd, float limit, float integral_limit) 
    : kp_(kp), ki_(ki), kd_(kd), output_limit_(limit), integral_limit_(integral_limit) {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
}

void PIDController::SetParameters(float kp, float ki, float kd, float limit, float integral_limit) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    output_limit_ = limit;
    integral_limit_ = integral_limit;
    Reset();
}

float PIDController::Calculate(float target, float actual, float dt) {
    if (dt <= 0.0f) return 0.0f;
    
    float error = target - actual;
    
    // 比例项
    float proportional = kp_ * error;
    
    // 积分项（带抗饱和）
    integral_ += error * dt;
    if (integral_ > integral_limit_) integral_ = integral_limit_;
    if (integral_ < -integral_limit_) integral_ = -integral_limit_;
    float integral = ki_ * integral_;
    
    // 微分项
    float derivative = kd_ * (error - prev_error_) / dt;
    prev_error_ = error;
    
    // 计算总输出
    float output = proportional + integral + derivative;
    
    // 输出限幅
    if (output > output_limit_) output = output_limit_;
    if (output < -output_limit_) output = -output_limit_;
    
    return output;
}

void PIDController::Reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
}