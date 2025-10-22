#include "gm6020.hpp"
#include <cmath>

// 构造函数
GM6020::GM6020(uint8_t id, CAN_HandleTypeDef* hcan, 
               float kp, float ki, float kd)
    : motor_id_(id), hcan_(hcan), 
      angle_pid_(kp, ki, kd, 10000.0f, 1000.0f) {   
    actual_speed_ = 0;
    rotor_angle_ = 0;
    actual_current_ = 0;
    temperature_ = 0;
    target_angle_ = 0.0f;
    output_current_ = 0;
}

void GM6020::SetTargetAngle(float angle) {
    target_angle_ = NormalizeAngle(angle);
}

void GM6020::UpdateAngleControl(float dt) {
    // 获取当前角度
    float current_angle = GetMechanicalAngle();
    
    float current_output = angle_pid_.Calculate(target_angle_, current_angle, dt);
    
    // 电流限幅
    if (current_output > 10000.0f) current_output = 10000.0f;
    if (current_output < -10000.0f) current_output = -10000.0f;
    
    output_current_ = static_cast<int16_t>(current_output);
}

void GM6020::SendCurrentCommand() {
    uint8_t data[8] = {0};
    
    int byte_offset = (motor_id_ - 1) * 2;
    if (byte_offset >= 0 && byte_offset <= 6) {
        data[byte_offset] = (output_current_ >> 8) & 0xFF;
        data[byte_offset + 1] = output_current_ & 0xFF;
    }
    
    uint32_t can_id = (motor_id_ <= 4) ? 0x1FE : 0x2FE; 
    
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    
    tx_header.StdId = can_id;
    tx_header.ExtId = 0;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    
    HAL_CAN_AddTxMessage(hcan_, &tx_header, data, &tx_mailbox);
}

void GM6020::ParseFeedback(uint8_t data[8]) {
    rotor_angle_ = (data[0] << 8) | data[1];
    actual_speed_ = (data[2] << 8) | data[3];
    actual_current_ = (data[4] << 8) | data[5];
    temperature_ = data[6];
}

float GM6020::GetMechanicalAngle() const {
    return (rotor_angle_ / 8192.0f) * 360.0f;
}

// 原始角度值转换为角度（度）
float GM6020::RawToAngle(uint16_t raw_angle) {
    return (raw_angle / 8192.0f) * 360.0f;
}

// 角度（度）转换为原始角度值
uint16_t GM6020::AngleToRaw(float angle) {
    float normalized = NormalizeAngle(angle);
    return static_cast<uint16_t>((normalized / 360.0f) * 8192.0f);
}

// 角度标准化到 0-360 度范围
float GM6020::NormalizeAngle(float angle) {
    float normalized = fmod(angle, 360.0f);
    if (normalized < 0.0f) {
        normalized += 360.0f;
    }
    return normalized;
}