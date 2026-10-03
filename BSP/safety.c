#include "safety.h"
#include "main.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint32_t pin;
} SafetyPin_t;

// 前四项是 PWM，后四项是方向，每个引脚独立使用 CubeMX 标签
static const SafetyPin_t SAFETY_PINS[] =
{
    {MOTOR_FL_PWM_GPIO_Port, MOTOR_FL_PWM_Pin},
    {MOTOR_FR_PWM_GPIO_Port, MOTOR_FR_PWM_Pin},
    {MOTOR_RL_PWM_GPIO_Port, MOTOR_RL_PWM_Pin},
    {MOTOR_RR_PWM_GPIO_Port, MOTOR_RR_PWM_Pin},
    {MOTOR_FL_DIR_GPIO_Port, MOTOR_FL_DIR_Pin},
    {MOTOR_FR_DIR_GPIO_Port, MOTOR_FR_DIR_Pin},
    {MOTOR_RL_DIR_GPIO_Port, MOTOR_RL_DIR_Pin},
    {MOTOR_RR_DIR_GPIO_Port, MOTOR_RR_DIR_Pin},
};

/**
 * @brief 将单个引脚位掩码换算为 MODER 中对应的两位字段偏移
 * @note 强制内联，避免 Debug 构建中急停路径产生额外函数调用
 */
__STATIC_FORCEINLINE uint32_t Safety_PinModeShift(uint32_t pin)
{
    uint32_t shift = 0U;
    while (pin > 1U)
    {
        pin >>= 1U;
        shift += 2U;
    }
    return shift;
}

void Safety_EmergencyStop(void)
{
    // 先清比较值，再无条件关闭 TIM1 主输出，不依赖通道使能状态
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    TIM1->CCR4 = 0U;
    TIM1->BDTR &= ~TIM_BDTR_MOE;

    for (uint32_t idx = 0U; idx < 4U; idx++)
    {
        GPIO_TypeDef *port = SAFETY_PINS[idx].port;
        uint32_t pin = SAFETY_PINS[idx].pin;
        uint32_t shift = Safety_PinModeShift(pin);

        // 先将输出锁存置低，再由复用模式切为通用输出 01
        port->BRR = pin;
        port->MODER = (port->MODER & ~(3UL << shift)) | (1UL << shift);
    }

    for (uint32_t idx = 4U; idx < sizeof(SAFETY_PINS) / sizeof(SAFETY_PINS[0]); idx++)
    {
        SAFETY_PINS[idx].port->BRR = SAFETY_PINS[idx].pin;
    }

    // 确保外设写入完成后再返回
    __DSB();
}
