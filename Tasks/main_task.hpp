#ifndef __MAIN_TASK_HPP__
#define __MAIN_TASK_HPP__

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "can.h"
#include "tim.h"
#include "iwdg.h"

// 初始化函数
void MainInit(void);

// 中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

// 获取状态函数（用于调试）
float GetTargetAngle(void);
float GetActualAngle(void);
int16_t GetActualSpeed(void);
int16_t GetMotorCurrent(void);

#ifdef __cplusplus
}
#endif

#endif
#ifdef __cplusplus
}
#endif