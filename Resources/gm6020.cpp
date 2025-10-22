#include "gm6020.hpp"

// 构造函数
GM6020::GM6020(uint8_t id, CAN_HandleTypeDef* hcan, float kp, float ki, float kd)
    : motor_id_(id), hcan_(hcan), speed_pid_(kp, ki, kd, 10000.0f, 5000.0f) {
    actual_speed_ = 0;
    rotor_angle_ = 0;
    actual_current_ = 0;
    temperature_ = 0;
    target_speed_ = 0;
    output_current_ = 0;
}

void GM6020::SetTargetSpeed(int16_t speed) {
    target_speed_ = speed;
}

void GM6020::UpdateSpeedControl(float dt) {
    // PID计算
    float output = speed_pid_.Calculate(static_cast<float>(target_speed_), static_cast<float>(actual_speed_), dt);
    output_current_ = static_cast<int16_t>(output);
}

void GM6020::SendCurrentCommand() {
    uint8_t data[8] = {0};
    
    // 根据电机ID确定在数据帧中的位置
    int byte_offset = (motor_id_ - 1) * 2;
    
    // 确保不越界
    if (byte_offset >= 0 && byte_offset <= 6) {
        data[byte_offset] = (output_current_ >> 8) & 0xFF;   // 高8位
        data[byte_offset + 1] = output_current_ & 0xFF;      // 低8位
    }
    
    // 确定CAN ID
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
    // 大端模式解析：DATA[0]=高8位, DATA[1]=低8位
    rotor_angle_ = (data[0] << 8) | data[1];      // 字节0-1: 转子角度
    actual_speed_ = (data[2] << 8) | data[3];     // 字节2-3: 实际转速
    actual_current_ = (data[4] << 8) | data[5];   // 字节4-5: 实际电流
    temperature_ = data[6];                       // 字节6: 温度
}

float GM6020::GetMechanicalAngle() const {
    // 将原始角度值(0-8191)转换为机械角度(0-360度)
    return (rotor_angle_ / 8192.0f) * 360.0f;
}