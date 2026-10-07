//
// Created by YZH on 2026/10/6.
//

#ifndef XIAOSAI_OLED_TEST_H
#define XIAOSAI_OLED_TEST_H

/**
 * @brief OLED 测试主循环：上电即显示字库全部字符和运行时间，永不返回
 * @note 在 MX_IWDG_Init 之后、MX_FREERTOS_Init 之前调用；循环内负责喂狗
 * @note 依次尝试地址 0x3C 和 0x3D；屏幕没接或掉线时每 500 ms 重新初始化
 * @note LED_RUN 慢闪（500 ms）表示屏幕正常，快闪（100 ms）表示找不到屏幕
 */
void OledTest_Run(void);

#endif //XIAOSAI_OLED_TEST_H
