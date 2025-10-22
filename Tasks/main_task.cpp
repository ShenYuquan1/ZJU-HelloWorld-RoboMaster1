#include "main_task.hpp"
#include "gm6020.hpp"
#include "PID.hpp"
#include <cmath>

// 全局变量
GM6020* motor1;

// 角度控制参数
volatile float target_angle = 0.0f;

// 角度序列
float angle_sequence[] = {45.0f, 180.0f};
uint8_t angle_index = 0;

// 状态变量
float actual_angle = 0.0f;
int16_t actual_speed = 0;
int16_t motor_current = 0;
uint8_t motor_temperature = 0;

// PID参数
float kp=45.0f, ki=1.0f, kd=1.0f;

// 时间计数器（基于TIM6中断）
uint32_t time_ms = 0;
uint32_t angle_change_counter = 0;
uint32_t pid_counter = 0;
uint32_t data_update_counter = 0;

// 控制周期定义
const uint32_t ANGLE_CHANGE_INTERVAL_MS = 3000;  // 3秒切换角度
const uint32_t DATA_UPDATE_INTERVAL_MS = 100;    // 100ms数据更新
const uint32_t PID_INTERVAL_MS = 1;              // 1ms PID周期

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

    // 创建电机对象 - 角度环控制
    motor1 = new GM6020(1, &hcan1, kp, ki, kd);
    
    // 设置初始角度
    target_angle = angle_sequence[0];
    motor1->SetTargetAngle(target_angle);
    
    // 启动TIM6中断
    HAL_TIM_Base_Start_IT(&htim6);
    
    // 初始化计数器
    time_ms = 0;
    angle_change_counter = 0;
    data_update_counter = 0;
}

// TIM6中断回调函数 - 核心定时逻辑
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        // 喂看门狗
        HAL_IWDG_Refresh(&hiwdg);
        
        // 更新时间计数器
        time_ms++;
        angle_change_counter++;
        pid_counter++;
        data_update_counter++;
        
        // 1. 角度切换周期：每3秒执行一次
        if (angle_change_counter >= ANGLE_CHANGE_INTERVAL_MS) {
            angle_change_counter = 0;
            
            // 切换到下一个角度
            angle_index = (angle_index + 1) % 2;
            target_angle = angle_sequence[angle_index];
            motor1->SetTargetAngle(target_angle);
        }
        
        // 2. 实时角度环控制：每1ms执行一次
        if (pid_counter >= PID_INTERVAL_MS) {
            pid_counter = 0;

            motor1->UpdateAngleControl(0.001f);

            motor1->SendCurrentCommand();
        }
        
        // 3. 数据更新周期：每100ms执行一次
        if (data_update_counter >= DATA_UPDATE_INTERVAL_MS) {
            data_update_counter = 0;
            
            // 更新状态数据
            actual_angle = motor1->GetMechanicalAngle();
            actual_speed = motor1->GetActualSpeed();
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
float GetTargetAngle(void) {
    return target_angle;
}

float GetActualAngle(void) {
    return actual_angle;
}

int16_t GetActualSpeed(void) {
    return actual_speed;
}

int16_t GetMotorCurrent(void) {
    return motor_current;
}
