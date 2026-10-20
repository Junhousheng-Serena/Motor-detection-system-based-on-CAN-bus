# 用户模块

`Core/Src` 中的 GPIO、TIM、ADC、USART、CAN 初始化由 CubeMX 维护。
`main.c` 负责启动模块以及创建 LED、CAN 接收任务。
`user` 中封装 NTC、电机、CAN 和串口打印；NTC 模块创建采样保护与打印任务。

| 文件 | 职责 |
|---|---|
| `drv_motor.c/.h` | DRV8833 PWM、正转调速、滑行停机、温度保护锁定 |
| `drv_ntc.c/.h` | ADC 双通道 DMA、两路各 15 点中位数滤波、温度/电压换算及保护/打印任务 |
| `drv_can.c/.h` | CAN 标准数据帧发送、接收中断与任务通知 |
| `drv_print.c/.h` | 串口重定向与带互斥锁的 `my_printf()` |

CAN 接收当前使用单份缓存；发送接口需由业务代码主动调用。

## 当前硬件连接

| 功能 | 引脚与配置 |
|---|---|
| DRV8833 AIN1 | PA6 / TIM3_CH1 |
| DRV8833 AIN2 | PA7 / TIM3_CH2 |
| NTC AO | PA4 / ADC1_IN4，12 MHz ADC 时钟，239.5 周期采样时间 |
| 电压输入 / INA180 OUT | PA3 / ADC1_IN3，239.5 周期采样时间，输入量程 0～VDDA |
| 串口发送、接收 | PB6 / USART1_TX、PB7 / USART1_RX，115200 / 8N1 |
| LED | PC13 |
| CAN | PA12 TX、PA11 RX；500 kbit/s，通过收发器连接总线 |

PWM 频率由 CubeMX 中的 TIM3 配置决定，当前为 20 kHz。
`Motor_Init()` 启动两个 PWM 通道，但占空比都为零。
取得有效且低于 50℃ 的温度后，CH1 才输出默认 50% 占空比，CH2 保持为零。

## 双通道采样

ADC1 扫描顺序固定为 Rank 1 = Channel 4（NTC），Rank 2 = Channel 3（电压）。
连续转换配合 Normal DMA，每批采集 30 个 `uint16_t`，即两路各 15 点。
停止 DMA 后，按下标拆分：

```c
ntc_samples[i]     = adc_buf[2 * i];
voltage_samples[i] = adc_buf[2 * i + 1];
```

`adc_buf` 保留原始交替顺序；`ntc_samples`、`voltage_samples` 分别排序并取中位数。
温度结果 `ntc_res` 单位为 ℃，仍用于电机保护；电压结果 `voltage_res` 单位为 V，
计算公式为 `ADC中位数 × ADC_REFERENCE_VOLTAGE / 4095`。
`drv_ntc.h` 中参考电压暂设为 3.3V，测量精度需要按实际 VDDA 校正。
电压端点 0、4095 分别按 0V、参考电压显示，不套用 NTC 的端点故障规则；
满量程读数不能区分刚好满量程和输入超量程。
这里测量的是 PA3 引脚电压；INA180 型号、增益及采样电阻尚未确定，未换算电流，
也未添加电流保护。DMA 故障或采样超时时，两路结果都更新为 `NAN`。

串口输出示例：`ntc_res: 25.0 C, voltage: 1.650 V, motor: READY`。

## 温度与电机联动

默认按 NTC 测量电机外壳温度考虑：感温头应贴近绕组所在区域的外壳，
并使用绝缘导热材料固定。若测 DRV8833，则固定在芯片塑封顶部，避开引脚。
外壳/封装表面温度不等于绕组/芯片内部温度，且有热响应延迟。
当前 NTC 尚未标定，60℃ 只是实验参数，不是电机的额定温度或可靠保护保证。

阈值在 `drv_motor.h`：

- `MOTOR_OVERHEAT_STOP_C = 60`：温度达到或超过 60℃，锁定停机。
- `MOTOR_START_MAX_TEMP_C = 50`：每次 MCU 复位后，必须取得低于 50℃ 的有效采样才允许启动。
- `MOTOR_DEFAULT_DUTY_PERCENT = 50`：启动时的默认占空比。

过温或采样异常后，即使温度恢复正常也不自动重启；需要手动复位 MCU，
重新通过温度检查。复位时若温度仍在 50～60℃ 之间则保持等待；若仍达到
60℃，会再次锁定。`Motor_Init()` 和调速命令不能解除已锁定的故障。

停机将 TIM3_CH1、CH2 的比较值都设为零，保持 PWM 通道启用。
AIN1=AIN2=0 对应 DRV8833 的高阻滑行模式：停止驱动，电机靠惯性减速，
不提供主动刹车或保持力矩。依据：
[TI DRV8833 数据手册，H-Bridge Control](https://www.ti.com/lit/ds/symlink/drv8833.pdf)。

`Motor_SetForwardDuty()` 在保护禁止输出时只记录目标占空比并返回 `HAL_BUSY`；
`Motor_Stop()` 同时将目标占空比清零。保护状态可通过
`Motor_GetProtectionState()` 或串口的 `motor:` 字段查看：
`WAIT_TEMP`、`READY`、`OVERHEAT_LATCHED`、`SENSOR_FAULT_LATCHED`。
`READY` 表示温度允许驱动，不代表电机一定正在转动。

## 默认运行行为与调参

- `main.c`：LED 每 500 ms 切换电平，CAN 任务等待接收并打印。
- `drv_ntc.c`：优先级 3 的任务负责采样和电机保护，每轮延时 200 ms；
  优先级 1 的独立任务输出同一批的温度、电压和保护状态，串口等待不阻塞采样保护任务。
- ADC DMA 在调度器启动后才开始，每批最多等待 100 ms，停止 DMA 后再处理。
  启动失败、超时、停止失败或无效温度均触发故障锁定。
- `drv_ntc.h`：每路样本数、DMA 总长度、参考电压、采样间隔、超时和端点容差；
  `drv_ntc.c`：温度模型参数。
- 温度模型暂用 R25 = 10 kΩ、B = 3950 K，固定上拉电阻为 10 kΩ。
  分压连接为 `VCC → 10 kΩ → AO → NTC → GND`，模块与 ADC 共用 3.3 V 电源。
  `ntc_res` 是按假设参数计算的估算温度，未做实测标定。

`ntc_res` 的单位是 ℃。ADC 中位数 ≤8 或 ≥4087 时，换算结果为 `NAN`，
用于识别靠近电源轨的异常读数；这不能检测所有传感器故障或感温头脱落。

CAN 驱动不创建任务；接收中断读取报文并通知 `main.c` 创建的接收任务。
CAN 和 NTC 打印任务共用 `drv_print.c` 的 `my_printf()`，每次调用由互斥锁保护。
温度保护依赖 MCU 和调度器正常执行；之前的 LED 停闪问题尚未通过板上现场定位，
本次联动不能保证在 HardFault 或 CPU 停止执行时关闭硬件 PWM。

顶层 `CMakeLists.txt` 显式列出 `user` 中的源文件。新增模块时将其 `.c`
加入该列表。CubeMX 重新生成代码时，应保留 `main.c` 的 USER CODE 区域。

## 验证

主机逻辑测试（需要本机 `gcc` 和 Python，不连接开发板）：

```text
python tests/thermal/run_tests.py
```

测试覆盖启动温度边界、过温及传感器故障锁定、调速不能绕过锁定、手动停止、
ADC 超时/错误、双通道拆分与独立滤波、电压换算，以及采样保护不执行串口打印。
固件使用 `cmake --build --preset Debug`
编译。主机模拟不能替代板上温度标定、PWM 电平及故障恢复验证。
