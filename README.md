# 基于 CAN 总线的电机检测与控制

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
- 过温、过流、采样异常或通信超时后锁定停机，需手动复位 F103。未安装编码器，不发送转速。

详细说明：[F103 模块](f103/user/README.md)、[H743 模块](h743/User/README.md)、[CAN 协议](docs/CAN_PROTOCOL.md)。

## 编译

需要 CMake、Ninja 和加入 PATH 的 `arm-none-eabi-gcc`。
在 `f103` 或 `h743` 目录内分别运行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

输出分别为 `f103/build/Debug/complete_f103.elf` 和 `h743/build/Debug/Demo.elf`。
两个工程使用各自的 `can_protocol.h`；更改协议时需同步更新两端。

## 主机测试

需要 Python 和主机 `gcc`。从仓库根目录运行：

```powershell
python tests/thermal/run_tests.py
python tests/communication/run_tests.py
```

通信测试默认读取本仓库的两个工程，可用 `--target f103` 或 `--target h743` 单独测试，
也可用 `--h743-project 路径` 指向其他本地 H743 工程。

目前已完成软件实现、两端编译和主机测试；尚未烧录及实测双板 CAN 总线。
160 mA 和 60℃ 为当前实验保护阈值，尚需结合实际采样标定及电机启动过程验证。
