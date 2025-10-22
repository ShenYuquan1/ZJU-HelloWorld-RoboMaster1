#include "main_task.hpp"
#include "gm6020.hpp"
#include "PID.hpp"
#include <cmath>

// 全局变量
GM6020* motor1;

// 正弦波参数
const float SPEED_AMPLITUDE = 80.0f;
const float SPEED_OFFSET = 80.0f;
const float SPEED_FREQUENCY = 1.0f; 

// 状态变量
int16_t target_speed = 0;
int16_t actual_speed = 0;
uint16_t motor_angle = 0;
int16_t motor_current = 0;
uint8_t motor_temperature = 0;

// PID参数
float kp=20.0f, ki=1.5f, kd=0.08f;

// 时间计数器（基于TIM6中断）
uint32_t time_ms = 0;
uint32_t control_counter = 0;
uint32_t pid_counter = 0;
uint32_t data_update_counter = 0;

// 控制周期定义
const uint32_t PID_INTERVAL_MS = 1;           // 1msPID周期
const uint32_t CONTROL_INTERVAL_MS = 10;      // 10ms控制周期
const uint32_t DATA_UPDATE_INTERVAL_MS = 100; // 100ms数据更新

// 初始化函数
void MainInit(void) {
    // CAN过滤器配置
    CAN_FilterTypeDef filter;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    // 创建电机对象
    motor1 = new GM6020(1, &hcan1, kp, ki, kd);
    
    // 启动TIM6中断
    HAL_TIM_Base_Start_IT(&htim6);
    
    // 初始化计数器
    time_ms = 0;
    control_counter = 0;
    pid_counter = 0;
    data_update_counter = 0;
}

// TIM6中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        
        // 喂看门狗
        HAL_IWDG_Refresh(&hiwdg);
        
        // 更新时间计数器
        time_ms++;
        control_counter++;
        pid_counter++;
        data_update_counter++;
        
        //控制周期：每10ms执行一次
        if (control_counter >= CONTROL_INTERVAL_MS) {
            control_counter = 0;
            
            // 计算正弦波目标速度
            float elapsed_time = time_ms / 1000.0f;  // 转换为秒
            float sine_speed = SPEED_OFFSET + SPEED_AMPLITUDE * sin(6.28f * SPEED_FREQUENCY * elapsed_time);
            target_speed = static_cast<int16_t>(sine_speed);
            motor1->SetTargetSpeed(target_speed);
        }
        
        //PID周期：每1ms执行一次
        if (pid_counter >= PID_INTERVAL_MS){
            pid_counter = 0;

            // 更新PID控制（1ms周期）
            motor1->UpdateSpeedControl(0.001f);  // 1ms = 0.001s
            
            // 发送电流命令
            motor1->SendCurrentCommand();
        }
        
        //数据更新周期：每100ms执行一次
        if (data_update_counter >= DATA_UPDATE_INTERVAL_MS) {
            data_update_counter = 0;
            
            // 更新状态数据
            actual_speed = motor1->GetActualSpeed();
            motor_angle = motor1->GetMechanicalAngle();
            motor_current = motor1->GetActualCurrent();
            motor_temperature = motor1->GetTemperature();
        }
    }
}

// CAN回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        if (rx_header.StdId >= 0x205 && rx_header.StdId <= 0x20B) {
            uint8_t motor_id = rx_header.StdId - 0x204;
            if (motor_id == 1 && motor1 != nullptr) {
                motor1->ParseFeedback(rx_data);
            }
        }
    }
}

// 获取状态函数（用于调试）
int16_t GetTargetSpeed(void) {
    return target_speed;
}

int16_t GetActualSpeed(void) {
    return actual_speed;
}

float GetActualAngle(void) {
    return motor_angle;
}

int16_t GetMotorCurrent(void) {
    return motor_current;
}

int16_t GetMechanicalAngle(void){
    return (motor_angle/8192.0f) * 360.0f;
}