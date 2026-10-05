//
// Created by YZH on 2026/10/5.
//

#ifndef XIAOSAI_MOTOR_TEST_H
#define XIAOSAI_MOTOR_TEST_H

/**
 * @brief 电机台架测试主循环，直接驱动四个电机并读取编码器，永不返回
 * @note 在 MX_IWDG_Init 之后、MX_FREERTOS_Init 之前调用；循环内负责喂狗
 * @note 不经过遥控、PID 和任务心跳；电机方向修正一律为 +1，用来确认原始转向
 */
void MotorTest_Run(void);

#endif //XIAOSAI_MOTOR_TEST_H
