#ifndef XIAOSAI_SAFETY_H
#define XIAOSAI_SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 通过寄存器强制关闭底盘输出，可在异常和关中断时重复调用
 * @note PWM 引脚切换为低电平通用输出，仅复位后恢复，不提供解除急停接口
 * @note 不操作舵机定时器和引脚，但后续 IWDG 整机复位仍会中断舵机 PWM
 */
void Safety_EmergencyStop(void);

#ifdef __cplusplus
}
#endif

#endif /* XIAOSAI_SAFETY_H */
