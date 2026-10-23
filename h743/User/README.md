# H743 用户模块

本工程使用 STM32H743IIT6，CAN 控制代码直接在本工程的 `User` 目录维护。

| 文件 | 用途 |
|---|---|
| `drv_can.c/.h` | 下发电机命令、双向心跳、接收测量及故障、串口状态显示 |
| `can_protocol.h` | 本工程使用的 CAN v2 报文定义和编解码 |
| `drv_print.c/.h` | 带互斥锁的串口打印 |

`Core/Src/main.c` 初始化打印和 CAN，创建通信、显示及 LED 任务。
通信任务每 50 ms 下发使能、PWM 占空比和过流停机阈值，每 500 ms 发心跳。
默认使能、占空比 50%、阈值 160 mA。没有编码器，不发送转速。

在 `FDCAN_Init()` 后或按键、上位机任务中调用：

```c
Can_SetMotorCommand(1, 50, 160); // 使能，50%，160 mA 停机阈值
Can_SetMotorCommand(1, 60, 140); // 调整占空比和停机阈值
Can_SetMotorCommand(0, 0, 160);  // 正常停机

CanRemoteSnapshot snapshot;
Can_GetRemoteSnapshot(&snapshot);
```

接口更新期望值，下一发送周期生效；返回成功不代表对端已经执行。
F103 必须通过本地传感器与通信检查才能驱动。过流、过温、采样异常或通信超时后锁定停机，
需要手动复位 F103，没有远程清除故障接口。

FDCAN1 使用 500 kbit/s 经典 CAN，PB9 为 TX、PB8 为 RX。
标准帧过滤器数量保留为 1，范围接收 0x301～0x3F1，再在中断中精确筛选有效 ID。
接收中断与通信任务不打印，显示任务每 500 ms 输出状态。
原来的 `vCanSend_Task` 演示函数保留在 main.c 中，当前不创建该任务。

在本工程目录构建：

```powershell
cmake --build --preset Debug
```

尚未配置时先运行 `cmake --preset Debug`。
工程移动后缓存路径失效时，运行 `cmake --fresh --preset Debug` 重新配置。
本工程独立构建，不需要从 F103 工程复制或安装驱动。

双方分别使用各自工程内的 `can_protocol.h`，更改报文格式时需同步更新两端。
完整协议见仓库根目录下的 [CAN_PROTOCOL.md](../../docs/CAN_PROTOCOL.md)。
跨工程通信测试位于仓库的 `tests/communication`，在 `h743` 工程目录运行：

```powershell
python ../tests/communication/run_tests.py --target h743
```

原本地工程保留在 `D:\CLionproject\RTOS\260920_complete\260920_completeh743`；
其中的 `build/before_can_integration` 为接入前备份，未上传到仓库。
代码编译和主机测试已通过，尚未烧录或进行双板总线实测。
