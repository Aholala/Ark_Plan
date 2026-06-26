# SUPERARK_MilkForg F405 主控工程

这是 STM32F405RGT6 主控工程，当前主要负责接收遥控器 NRF24L01 控制包、通过 USB 虚拟串口接收视觉颜色结果，并提供 PWM、编码器、WS2812 等底层和模块化接口。目标平台是麦轮底盘，不是平衡车。

## 工程结构

```text
User/
  App/
    app_remote_task.c/.h         遥控接收任务与遥控数据状态
    app_ws2812_task.c/.h         WS2812 灯带任务，包含 USB 初始化和视觉颜色显示逻辑
  Bsp/
    bsp_encoder.c/.h             TIM2/TIM3 编码器硬件适配
    bsp_nrf24l01.c/.h            NRF24L01 芯片驱动
    bsp_nrf24l01_define.h        NRF24L01 寄存器和命令定义
    bsp_pwm.c/.h                 TIM1/TIM4/TIM5/TIM8 PWM 输出适配
    bsp_radio_link.c/.h          无线链路适配层
    bsp_time.c/.h                系统时间接口
    bsp_usb_cdc.c/.h             USB CDC 接收缓存和回显
    bsp_ws2812.c/.h              WS2812 硬件适配
  Module/
    module_encoder.c/.h          编码器抽象层
    module_remote_protocol.c/.h  遥控协议编解码
    module_vision_protocol.c/.h  视觉 USB 颜色协议解析
    module_ws2812.c/.h           WS2812 抽象层
```

## 分层说明

- `Bsp` 层绑定具体 STM32F405 外设和引脚。
- `Module` 层提供可移植抽象逻辑，通过结构体和 ops 接口隔离硬件。
- `App` 层处理应用业务，比如接收遥控器数据、维护连接状态、记录按键事件。

不要在 App 层直接读写 NRF 寄存器、TIM 通道或 GPIO。

## USB 视觉颜色协议

视觉通过 USB 虚拟串口发送 6 字节定长帧：

```text
Byte0  Byte1  Byte2          Byte3        Byte4  Byte5
0xA5   0x5A   基础框颜色ID   核心框颜色ID  0x00   CRC
```

CRC 为前 5 个字节异或：

```text
CRC = Byte0 ^ Byte1 ^ Byte2 ^ Byte3 ^ Byte4
```

颜色 ID：

```text
0x00 关闭/无颜色
0x01 红色
0x02 橙色
0x03 黄色
0x04 绿色
0x05 青色
0x06 蓝色
0x07 紫色
```

示例：

```text
A5 5A 01 06 00 F8
```

含义：基础框为红色，核心框为蓝色。

收到有效视觉帧后，WS2812 前半段显示基础框颜色，后半段显示核心框颜色，并按 500 ms 周期亮灭闪烁。解析失败时忽略该帧，灯带保持当前状态。USB CDC 当前会把收到的数据原样回显，便于串口助手测试。

## NRF24L01 配置

- 地址：`{'n', 'a', 'i', 'w', 'a'}`，也就是 ASCII `naiwa`
- 地址宽度：5 字节
- 频道：`RF_CH = 0x02`，即 2402 MHz
- 空中速率：2 Mbps
- 自动重发：间隔 750 us，最多 15 次
- payload 长度：32 字节
- IRQ：PB4，下降沿中断
- CE：PB3
- CSN：PA15
- SPI3：PC10/PC11/PC12

遥控器和 F405 两边必须保持同样的 NRF 参数。

## 遥控接收数据

F405 收到遥控器控制包后，会更新：

```c
const AppRemoteData_t *App_Remote_GetData(void);
```

重点字段：

- `mode`：遥控器模式
- `lh/lv/rh/rv`：摇杆数据
- `key`：最后一次收到的新按键编号
- `key_seq`：最后一次按键事件序号
- `key_event_count`：收到新按键事件的累计次数
- `connected`：500 ms 内收到包则为 1，否则为 0
- `nrf_rx_count`：NRF 成功收包计数
- `nrf_status/nrf_config/nrf_fifo_status`：NRF 调试寄存器

做模式切换时建议使用：

```text
key_event_count 是否增加 -> 判断是否有新按键
key -> 判断具体是哪一个按键
```

## 麦轮反馈协议

反馈包已按麦轮四电机建模：

```c
typedef struct
{
  int8_t pwm[4];
  float speed[4];
} RemoteProtocolFeedbackPacket_t;
```

约定：

- `pwm[0] / speed[0]`：M1
- `pwm[1] / speed[1]`：M2
- `pwm[2] / speed[2]`：M3
- `pwm[3] / speed[3]`：M4

当前工程还没有实现 F405 向遥控器发送反馈。后续如果要做反馈，建议优先考虑 NRF24L01 ACK payload，让主控在 ACK 中带回上一帧电机状态。

## 电机与编码器

PWM 当前只保留 BSP 层，不再有单独的 motor 抽象层：

- `bsp_pwm`：TIM/PWM 通道映射和硬件输出
- `Bsp_Pwm_Init()`：启动所有 PWM 输出，初始比较值为 0
- `Bsp_Pwm_SetDuty()`：按百分比设置占空比
- `Bsp_Pwm_SetPulse()`：直接设置比较值

编码器也已拆分：

- `module_encoder`：计数、增量等可移植逻辑
- `bsp_encoder`：TIM2/TIM3 编码器模式适配

麦轮底盘后续控制算法建议新增独立 Module，例如 `module_mecanum`，不要写进 BSP。

## 更新日志

### 2026-06-27

- 调整 App 层结构：删除单独的 `app_remote.c/.h`，遥控接收逻辑合并到 `app_remote_task.c/.h`。
- 删除独立 `app_usb_task.c/.h`，USB 虚拟串口初始化合并到 `app_ws2812_task`。
- 新增 `bsp_usb_cdc.c/.h`，USB CDC 收到数据后保存到缓存，并原样回显用于串口测试。
- 新增 `module_vision_protocol.c/.h`，视觉协议独立放在 Module 层解析。
- 视觉协议调整为双颜色 ID：基础框颜色 + 核心框颜色。
- WS2812 任务收到有效视觉帧后，前半段显示基础框颜色，后半段显示核心框颜色，并以 500 ms 周期非阻塞闪烁。
- 删除 `bsp_motor.c/.h` 和 `module_motor.c/.h`，改为更直接的 `bsp_pwm.c/.h`。
- `main.c` 初始化流程改为 `Bsp_Encoder_Init()` + `Bsp_Pwm_Init()`，不再初始化 motor 抽象层。

## 编译

```powershell
cmake --build --preset Debug
```

编译产物在：

```text
build/Debug/SUPERARK_MilkForg.elf
```

## Ozone 观察点

建议观察全局变量：

```text
app_remote_data
```

常用字段：

- `connected`
- `lh/lv/rh/rv`
- `key`
- `key_seq`
- `key_event_count`
- `nrf_rx_count`
- `nrf_status`
- `nrf_config`
- `nrf_fifo_status`

如果 `nrf_status` 和 `nrf_config` 都是 `255`，通常说明 SPI 没有正确读到 NRF24L01。若 `connected=0` 但寄存器正常，优先检查地址、频道、速率和两边是否都已烧录最新固件。
