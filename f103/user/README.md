# 用户模块

`Core/Src` 中的 GPIO、TIM、ADC、USART、CAN 初始化由 CubeMX 维护。
`main.c` 负责启动模块以及创建 LED、CAN 接收任务。
`user` 中封装 NTC、电流换算、电机、CAN 和串口打印；NTC 模块创建两路采样保护与打印任务。

| 文件 | 职责 |
|---|---|
| `drv_motor.c/.h` | DRV8833 PWM、正转调速、滑行停机、温度/过流/通信保护锁定 |
| `drv_ntc.c/.h` | ADC 双通道 DMA、两路各 15 点中位数滤波、温度/电压换算及保护/打印任务 |
| `drv_current.c/.h` | INA180A1 的增益、采样电阻配置及电压到电流的换算 |
| `drv_can.c/.h` | CAN 命令接收、双向心跳、温度/电流/状态上报及故障事件 |
| `can_protocol.h` | F103 和 H743 共用的 CAN 报文编码与解码 |
| `drv_print.c/.h` | 串口重定向与带互斥锁的 `my_printf()` |

CAN 接收使用 16 帧静态队列；F103 每 50 ms 上报测量与状态，每 500 ms 发送心跳。
当前对端目录实际使用 STM32H743，FDCAN 配置为经典 CAN 模式，并非 F407 工程。
H743 每 50 ms 下发使能、PWM 占空比和过流停机阈值，默认使能、50%、160 mA。
没有转速传感器，不传转速，不提供远程故障复位。
报文和接线说明见 [CAN_PROTOCOL.md](../../docs/CAN_PROTOCOL.md)，
H743 完整工程位于仓库的 `h743` 目录，
其接口说明见 [H743 用户模块](../../h743/User/README.md)。
两个工程各自使用本地的 `can_protocol.h`；更改报文格式时需同步更新两端。

## 当前硬件连接

| 功能 | 引脚与配置 |
|---|---|
| DRV8833 AIN1 | PA6 / TIM3_CH1 |
| DRV8833 AIN2 | PA7 / TIM3_CH2 |
| NTC AO | PA4 / ADC1_IN4，12 MHz ADC 时钟，239.5 周期采样时间 |
| INA180A1IDBVR OUT | PA3 / ADC1_IN3，239.5 周期采样时间，输入量程 0～VDDA；当前采样电阻 1 Ω |
| 串口发送、接收 | PB6 / USART1_TX、PB7 / USART1_RX，115200 / 8N1 |
| LED | PC13 |
| CAN | PA12 TX、PA11 RX；500 kbit/s，通过收发器连接总线 |

PWM 频率由 CubeMX 中的 TIM3 配置决定，当前为 20 kHz。
`Motor_Init()` 启动两个 PWM 通道，但占空比都为零。
收到有效 H743 心跳和控制命令，且同一批温度有效并低于 50℃、电流有效并低于阈值后，
CH1 才按命令输出占空比（默认 50%），CH2 保持为零。

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
电压中位数进一步换算为 `current_res_ma`，用于显示；每批电压最大值换算为
`current_peak_ma`，用于过流判断。中位数不是 PWM 电流的平均值或有效值，
采样最大值也不代表捕获了全部实际峰值。DMA 故障或采样超时时，
温度、电压和两项电流结果均更新为 `NAN`。

串口输出示例：
`ntc_res: 25.0 C, voltage: 0.600 V, current: 30.0 mA, peak: 30.0 mA, motor: READY`。

## 电流检测与过流停机

硬件参数来自当前电路：INA180A1IDBVR，增益 20 V/V，采样电阻 1 Ω。
依据 [TI INA180 数据手册](https://www.ti.com/lit/ds/symlink/ina180.pdf)，
理想换算为：

```text
I(mA) = Vout(V) × 1000 / (20 × 1 Ω) = Vout × 50
```

`drv_current.h` 中 `CURRENT_AMPLIFIER_GAIN = 20`、
`CURRENT_SHUNT_MILLIOHM = 1000`；电阻参数单位是毫欧，必须与实物一致。
尚未做零点、增益、采样电阻误差及 VDDA 的实测校准。

用户提供电机空载电流 30 mA、额定电流 150 mA、堵转电流 300 mA。
额定电流对应厂家规定的额定运行条件，不是任何时刻都固定消耗的电流，
也不等同于瞬时过流阈值。当前 1 Ω 电阻下，150 mA 对应 3.0 V，
300 mA 理想输出需 6.0 V，实际放大器会饱和；3.3 V ADC 的理论满量程
仅 165 mA，实际线性范围还受 INA180 供电及输出摆幅限制。
饱和后的读数不能用于区分 200 mA 和 300 mA。

当前 `drv_motor.h` 的 `MOTOR_OVERCURRENT_STOP_MA = 160` 只是现有量程内的
实验停机上限；H743 可下发 1～160 mA 的阈值。它距离满量程很近，不是已验证的电机保护参数。每批 15 个电流样本中，
任意一个换算值达到或超过阈值，就在该批处理时锁定停机；
不使用中位数掩盖采到的峰值，也没有启动屏蔽期或多批去抖，
因此启动电流和噪声尖峰都可能触发。首次采样也先检查电流，再允许启动 PWM。
配置阈值达到或超过 ADC 理论满量程时，`Ntc_Init()` 返回错误并保持停机。

建议改用 0.1 Ω 电阻后再覆盖完整电流范围，此时 150 mA 对应 0.3 V，
300 mA 对应 0.6 V。更换实物后须同步将 `CURRENT_SHUNT_MILLIOHM` 改为 `100`；
可从 200 mA 实验阈值开始，结合启动波形、PWM 采样位置和负载验证。
当前代码仍按实际已知的 1 Ω 配置，未假定硬件已经更换。
更换电阻并提高阈值时，还需同步修改双方协议的 `CAN_CURRENT_LIMIT_MAX_MA` 与默认参数。

此处测量的是采样电阻流过的正向电流。如果电阻位于驱动器电源回路，
读数是该回路的电流，PWM 下不一定等于电机绕组电流。
0 V 允许作为零电流，不能据此识别 INA180 断线、掉电或采样极性接反。
INA180 若以 5 V 供电，应在硬件上确保 OUT 不会超过 PA3 的 ADC 输入范围。

## 温度与电机联动

默认按 NTC 测量电机外壳温度考虑：感温头应贴近绕组所在区域的外壳，
并使用绝缘导热材料固定。若测 DRV8833，则固定在芯片塑封顶部，避开引脚。
外壳/封装表面温度不等于绕组/芯片内部温度，且有热响应延迟。
当前 NTC 尚未标定，60℃ 只是实验参数，不是电机的额定温度或可靠保护保证。

阈值在 `drv_motor.h`：

- `MOTOR_OVERHEAT_STOP_C = 60`：温度达到或超过 60℃，锁定停机。
- `MOTOR_START_MAX_TEMP_C = 50`：每次 MCU 复位后，必须取得低于 50℃ 的有效采样才允许启动。
- `MOTOR_DEFAULT_DUTY_PERCENT = 50`：保留的本地实验默认值；联动模式使用 H743 的 `CAN_DEFAULT_DUTY_PERCENT`。

过温、过流、采样异常或通信超时后，即使测量值恢复正常也不自动重启；需要手动复位 MCU，
重新通过温度与电流检查。复位时若温度仍在 50～60℃ 之间则保持等待；若仍达到
60℃，会再次锁定。`Motor_Init()` 和调速命令不能解除已锁定的故障。

停机将 TIM3_CH1、CH2 的比较值都设为零，保持 PWM 通道启用。
AIN1=AIN2=0 对应 DRV8833 的高阻滑行模式：停止驱动，电机靠惯性减速，
不提供主动刹车或保持力矩。依据：
[TI DRV8833 数据手册，H-Bridge Control](https://www.ti.com/lit/ds/symlink/drv8833.pdf)。

`Motor_SetForwardDuty()` 在保护禁止输出时只记录目标占空比并返回 `HAL_BUSY`；
`Motor_Stop()` 同时将目标占空比清零。保护状态可通过
`Motor_GetProtectionState()` 或串口的 `motor:` 字段查看：
`WAIT_SENSORS`、`READY`、`OVERHEAT_LATCHED`、`OVERCURRENT_LATCHED`、
`SENSOR_FAULT_LATCHED`、`COMM_FAULT_LATCHED`。故障锁定后保留最先锁定的原因。
`READY` 表示温度与电流允许驱动，不代表电机一定正在转动。
保护接口为 `Motor_UpdateProtection(temperatureC, currentPeakMa)`，
必须传入同一批的温度和电流；原来的单温度更新接口已由此接口替代。

## 默认运行行为与调参

- `main.c`：LED 每 500 ms 切换电平，CAN 任务处理接收队列并定时上报。
- `drv_ntc.c`：优先级 3 的任务负责采样和电机保护，每轮处理后延时 10 ms；
  优先级 1 的独立任务每轮延时 200 ms，输出同一批的温度、电压、电流及保护状态，
  串口等待不阻塞采样保护任务。
- ADC DMA 在调度器启动后才开始，每批最多等待 20 ms，停止 DMA 后再处理。
  启动失败、超时、停止失败或无效温度/电流均触发故障锁定。
  当前 RTOS tick 为 10 ms，延时值不等于精确采样周期或最坏停机响应时间。
  批次之间仍有采样间隙，短脉冲可能漏检，这不是连续硬件短路保护。
- `drv_ntc.h`：每路样本数、DMA 总长度、参考电压、采样间隔、超时和端点容差；
  `drv_ntc.c`：温度模型参数。
- 温度模型暂用 R25 = 10 kΩ、B = 3950 K，固定上拉电阻为 10 kΩ。
  分压连接为 `VCC → 10 kΩ → AO → NTC → GND`，模块与 ADC 共用 3.3 V 电源。
  `ntc_res` 是按假设参数计算的估算温度，未做实测标定。

`ntc_res` 的单位是 ℃。ADC 中位数 ≤8 或 ≥4087 时，换算结果为 `NAN`，
用于识别靠近电源轨的异常读数；这不能检测所有传感器故障或感温头脱落。

CAN 驱动不创建任务；`Can_Init()` 在调度器启动前创建静态接收队列，
`main.c` 创建优先级 2 的通信任务，任务启动后才使能 CAN 和接收通知。
接收中断只复制报文并入队，通信任务每轮最多处理 16 帧，然后延时 10 ms；
中断和通信任务都不执行串口打印，温度/电流保护任务优先级仍为 3。
`my_printf()` 仍可供其他任务使用，每次调用由互斥锁保护。
启动后 1.5 秒未收到有效命令、运行中 200 ms 未收到新命令或 1.5 秒未收到心跳，
均锁定通信故障并停机。通信恢复不能自动解除锁定；本地温度和电流保护始终运行。
温度和电流保护依赖 MCU 和调度器正常执行；之前的 LED 停闪问题尚未通过板上现场定位，
本次联动不能保证在 HardFault 或 CPU 停止执行时关闭硬件 PWM。

顶层 `CMakeLists.txt` 显式列出 `user` 中的源文件。新增模块时将其 `.c`
加入该列表。CubeMX 重新生成代码时，应保留 `main.c` 的 USER CODE 区域。

## 验证

在仓库根目录运行主机逻辑测试（需要本机 `gcc` 和 Python，不连接开发板）：

```text
python tests/thermal/run_tests.py
python tests/communication/run_tests.py
```

通信测试直接编译仓库内 `f103/user` 和 `h743/User` 源码。
H743 工程在其他位置时使用 `--h743-project 路径`；仅有 F103 工程时，
使用 `python tests/communication/run_tests.py --target f103` 单独测试。

测试覆盖启动温度边界、过温/过流及传感器故障锁定、调速不能绕过锁定、手动停止、
ADC 超时/错误、双通道拆分与独立滤波、电压/电流换算、单点过流与 ADC 阈值边界，
以及首次过流不会产生启动 PWM、采样保护不执行串口打印。
固件在 `f103` 目录中先运行 `cmake --preset Debug`，再运行 `cmake --build --preset Debug`
编译。主机模拟不能替代板上温度/电流标定、启动波形、PWM 电平及故障恢复验证。
通信测试使用双方实际 CAN 驱动和主机 HAL/RTOS 桩，覆盖报文字节序、默认使能、控制周期、启停、阈值调节、
非法帧、重复/倒序命令、队列溢出、命令和心跳超时、故障保持/重试、计时回绕和发送忙；
双板物理总线仍需联调。
