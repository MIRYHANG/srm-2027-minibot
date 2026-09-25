# srm-2027-minibot

2027 校内赛小车的 STM32 控制程序。项目使用 STM32G474VET6、STM32Cube HAL 和 FreeRTOS，通过 CLion/CMake 开发。

## 当前进度

- 已编写四轮电机的 PWM、方向控制及编码器模块
- 已编写 PID 控制器模块
- 已在 FreeRTOS 任务中初始化并启动四个电机
- 编码器测速与 PID 闭环尚未接入任务
- 硬件尚未到齐，电机方向、编码器参数及实际运行效果均未验证

## 代码结构

- `BSP/`：电机、编码器等硬件相关代码
- `Algorithm/`：PID 等控制算法
- `Core/`：STM32CubeMX 生成的初始化代码及应用任务
- `XiaoSai.ioc`：STM32CubeMX 外设配置

## 编译

用 CLion 打开项目，选择 Debug 配置后构建。编译通过不代表已经完成硬件测试。

## 下一步

- 根据电机参数确定编码器每圈计数
- 接入周期性测速与 PID 速度闭环
- 收到硬件后逐个验证电机方向、编码器方向和急停行为

## 参考项目

PID 模块参考了湖南大学岳麓战队的
[HNUYueLuRM/basic_framework](https://github.com/HNUYueLuRM/basic_framework)。
