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
    PIDController angle_pid_;      
    
    // 电机状态
    int16_t actual_speed_;
    uint16_t rotor_angle_;
    int16_t actual_current_;
    uint8_t temperature_;
    
    // 控制参数
    float target_angle_;
    int16_t output_current_;
    
public:
    // 构造函数
    GM6020(uint8_t id, CAN_HandleTypeDef* hcan, 
           float kp , float ki , float kd );
    
    // 设置目标角度
    void SetTargetAngle(float angle);
    
    // 更新角度环控制
    void UpdateAngleControl(float dt);
    
    // 发送电流控制命令
    void SendCurrentCommand();
    
    // 解析反馈数据
    void ParseFeedback(uint8_t data[8]);
    
    // 获取电机状态
    int16_t GetActualSpeed() const { return actual_speed_; }
    uint16_t GetRotorAngle() const { return rotor_angle_; }
    int16_t GetActualCurrent() const { return actual_current_; }
    int16_t GetOutputCurrent() const { return output_current_; }
    uint8_t GetTemperature() const { return temperature_; }
    float GetTargetAngle() const { return target_angle_; }
    float GetMechanicalAngle() const;
    
    // 角度转换函数
    static float RawToAngle(uint16_t raw_angle);
    static uint16_t AngleToRaw(float angle);
    static float NormalizeAngle(float angle);
};

#endif