# 基于 CAN 总线的电机检测与控制
供电板原理图：
<img width="2362" height="1672" alt="SCH_Schematic1_1-P1_2026-10-23" src="https://github.com/user-attachments/assets/3f5f8c49-10c7-4f6f-aaa8-d2c4520b08fb" />

供电板PCB图：
<img width="2160" height="1854" alt="PCB_PCB1_2026-10-23" src="https://github.com/user-attachments/assets/7eea61de-3c4f-4953-b501-e0c70cb4dd31" />



仓库包含两个独立的 STM32 工程：F103 负责电机驱动、采样和保护，H743 负责下发控制命令及接收状态。

```text
.
├── f103/       STM32F103C8T6 工程
├── h743/       STM32H743IIT6 工程
├── docs/       双方 CAN 协议和接线说明
└── tests/      采样保护与双端通信的主机测试
```

两个工程各自包含 CubeMX 配置、HAL/CMSIS、FreeRTOS、启动文件、链接脚本和 CMake 构建文件。
CLion 中分别打开 `f103` 或 `h743` 文件夹；CubeMX 分别打开 `f103/complete_f103.ioc`、`h743/Demo.ioc`。
编译产物、IDE 状态、CubeMX 缓存和本地备份不上传。

## 当前功能

- F103：ADC 双通道 DMA，每路 15 点；NTC 温度、INA180A1 电流检测、PWM 驱动和故障停机。
- H743：周期下发使能、PWM 占空比、过流停机阈值；接收温度、电流、PWM、状态和故障事件。
- 500 kbit/s 经典 CAN：控制与测量每 50 ms，双向心跳每 500 ms。
- 默认使能、50% 占空比、160 mA 过流停机阈值。F103 取得有效命令、心跳和满足启动条件的采样后才驱动。
- 过温、过流、采样异常或通信超时后锁定停机，需手动复位 F103。

详细说明：[F103 模块](f103/user/README.md)、[H743 模块](h743/User/README.md)、[CAN 协议](docs/CAN_PROTOCOL.md)。



## 主机测试

160 mA 和 60℃ 为当前实验保护阈值，尚需结合实际采样标定及电机启动过程验证。
