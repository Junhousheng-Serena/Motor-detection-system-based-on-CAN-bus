# H743 对端接入文件

目标工程：
`D:\CLionproject\RTOS\260920_complete\260920_completeh743`，
实际 MCU 为 STM32H743IIT6，FDCAN1 采用 500 kbit/s 经典 CAN。

此目录保存对端 `User/drv_can.c`、`User/drv_can.h` 的源文件。
双方共用的协议头以本仓库 `user/can_protocol.h` 为准。
这样 F103 的 Git 仓库同时保留可复现的 H743 通信改动，无需复制整套 H7 HAL。

从 F103 仓库运行：

```powershell
python peer_h743/integrate.py D:\CLionproject\RTOS\260920_complete\260920_completeh743
```

脚本会：

- 检查 `.ioc` 是否为 STM32H743。
- 将通信驱动和共用协议头复制到 H743 的 `User`。
- 接入 `drv_print.c`、初始化串口打印互斥锁。
- 把当前创建的空数据发送任务替换为优先级 2 的通信任务，
  另建优先级 1 的显示任务；原 `vCanSend_Task` 演示函数保留在源文件中，暂不创建。
- 将 H743 FreeRTOS 动态堆从 4096 字节改为 8192 字节，以容纳通信、显示和 LED 任务。
- 在本仓库忽略的 `build/h743_before_can` 目录保留原文件备份。

脚本以原工程的初始化位置为依据；找不到关键插入点时退出，不盲目替换其他代码。
再次执行时内容相同的文件不会重复写入。修改对端驱动后应同步此目录，避免下一次安装覆盖。

随后在 H743 工程目录构建：

```powershell
cmake --build --preset Debug
```

若尚未配置 CMake，先执行 `cmake --preset Debug`。
若工程搬迁后缓存仍指向旧目录，执行 `cmake --fresh --preset Debug` 重新生成构建缓存。
本次不包含远程电机控制；H743 每 500 ms 发心跳、解析 F103 上报，
每 500 ms 在串口显示最近一组完整数据或离线状态。
报文内容及验证方法见 [CAN_PROTOCOL.md](../docs/CAN_PROTOCOL.md)。

2026-10-20 已在上述 H743 工程完成接入并通过 Debug 编译；
F103 Debug 编译、15 项通信主机测试及 25 项温度/电流保护测试均通过。
尚未进行烧录或双板总线实测。
