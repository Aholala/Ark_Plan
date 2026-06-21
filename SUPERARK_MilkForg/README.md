# SUPERARK_MilkForg F405 主控工程

这是 STM32F405RGT6 主控工程，当前主要负责接收遥控器 NRF24L01 控制包，并提供电机、编码器、WS2812 等底层和模块化接口。目标平台是麦轮底盘，不是平衡车。

## 工程结构

```text
User/
  App/
    app_remote.c/.h              遥控接收应用逻辑
  Bsp/
    bsp_encoder.c/.h             TIM2/TIM3 编码器硬件适配
    bsp_motor.c/.h               电机 PWM 硬件适配
    bsp_nrf24l01.c/.h            NRF24L01 芯片驱动
    bsp_nrf24l01_define.h        NRF24L01 寄存器和命令定义
    bsp_radio_link.c/.h          无线链路适配层
    bsp_time.c/.h                系统时间接口
    bsp_ws2812.c/.h              WS2812 硬件适配
  Module/
    module_encoder.c/.h          编码器抽象层
    module_motor.c/.h            电机抽象层
    module_remote_protocol.c/.h  遥控协议编解码
    module_ws2812.c/.h           WS2812 抽象层
```

## 分层说明

- `Bsp` 层绑定具体 STM32F405 外设和引脚。
- `Module` 层提供可移植抽象逻辑，通过结构体和 ops 接口隔离硬件。
- `App` 层处理应用业务，比如接收遥控器数据、维护连接状态、记录按键事件。

不要在 App 层直接读写 NRF 寄存器、TIM 通道或 GPIO。

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

电机 PWM 已按抽象层拆分：

- `module_motor`：电机控制抽象层
- `bsp_motor`：TIM/PWM 通道映射和硬件输出

编码器也已拆分：

- `module_encoder`：计数、增量等可移植逻辑
- `bsp_encoder`：TIM2/TIM3 编码器模式适配

麦轮底盘后续控制算法建议新增独立 Module，例如 `module_mecanum`，不要写进 BSP。

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
