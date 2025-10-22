#ifndef __GM6020_HPP__
#define __GM6020_HPP__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "can.h"

#ifdef __cplusplus
}
#endif

#include "PID.hpp"

class GM6020 {
private:
    uint8_t motor_id_;         
    CAN_HandleTypeDef* hcan_;
    PIDController speed_pid_;
    
    // 电机状态
    int16_t actual_speed_;      // 实际转速 (RPM)
    uint16_t rotor_angle_;      // 转子角度
    int16_t actual_current_;    // 实际电流
    uint8_t temperature_;       // 温度
    
    // 控制参数
    int16_t target_speed_;      // 目标转速
    int16_t output_current_;    // 输出电流值
    
public:
    // 构造函数
    GM6020(uint8_t id, CAN_HandleTypeDef* hcan, 
           float kp , float ki , float kd );
    
    // 设置目标速度
    void SetTargetSpeed(int16_t speed);
    
    // 更新PID计算
    void UpdateSpeedControl(float dt);
    
    // 发送电流控制命令
    void SendCurrentCommand();
    
    // 解析反馈数据
    void ParseFeedback(uint8_t data[8]);
    
    // 获取电机状态
    int16_t GetActualSpeed() const { return actual_speed_; }
    uint16_t GetRotorAngle() const { return rotor_angle_; }
    int16_t GetActualCurrent() const { return actual_current_; }
    uint8_t GetTemperature() const { return temperature_; }
    float GetMechanicalAngle() const;
    
};

#endif