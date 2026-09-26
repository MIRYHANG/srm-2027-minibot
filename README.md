# srm-2027-minibot

2027 校内赛小车的 STM32 控制程序，使用 STM32G474VET6、STM32Cube HAL、FreeRTOS 和 CMake，在 CLion 中开发。

## 2026-09-26 开发记录

- 封装 `Encoder_InitAndStart()`，初始化并启动四路编码器。
- 将默认任务改为底盘控制任务，使用 `osDelayUntil()` 约每 10 ms 运行一次，并根据实际 tick 间隔计算 `dt_s`。
- 初始化四个车轮的速度 PID，封装四轮及单轮的编码器更新、PID 计算流程。
- 编写麦克纳姆轮解算模块：将底盘的前后速度 `vx`、左右速度 `vy`、旋转速度 `wz` 换算为四轮目标 RPM。当前约定 `vx` 正数向前、`vy` 正数向左、`wz` 正数逆时针。
- 编写电脑端测试，检查向前、向左、逆时针旋转及无效轮半径四种情况；测试通过。
- 将麦轮解算接入底盘任务，计算结果写入四轮目标转速。

目前底盘指令固定为零；PID 只进行计算，**输出尚未施加到电机**。代码通过编译和公式测试，不代表已经完成实物验证。

## 代码结构

- `BSP/`：电机 PWM、方向控制和编码器模块
- `Algorithm/`：PID 控制器与麦轮解算
- `Core/`：STM32CubeMX 生成代码及 FreeRTOS 应用任务
- `Test/test_mecanum.c`：电脑端麦轮解算测试，不参与 STM32 固件编译
- `XiaoSai.ioc`：STM32CubeMX 外设配置

## 编译与测试

固件编译：

```powershell
cmake --build --preset Debug
```

电脑端麦轮测试：

```powershell
gcc -std=c11 -Wall -Wextra -IAlgorithm Test/test_mecanum.c Algorithm/mecanum.c -o build/test_mecanum.exe -lm
.\build\test_mecanum.exe
```

测试中的底盘尺寸仅用于核对公式，**不是实车参数**。

## TODO：待确认的参数

- [ ] 测量轮半径、前后轮中心距、左右轮中心距，替换当前为零的底盘尺寸。
- [ ] 确认输出轴转一圈对应的编码器计数；当前 `COUNT_PER_REV = 10` 尚未验证。
- [ ] 逐轮确认编码器方向；当前四轮共用 `DIRECTION_SIGN = -1`。
- [ ] 逐轮确认电机正方向；当前四轮的方向修正均为 `+1`。
- [ ] 根据实测速度抖动与响应效果调整编码器滤波时间；当前为 `0.01 s`。
- [ ] 核对麦轮实际安装方向是否符合当前 X 型解算公式。
- [ ] 硬件联调后整定 PID 的 `Kp`、`Ki`、`Kd`；当前均为零。
