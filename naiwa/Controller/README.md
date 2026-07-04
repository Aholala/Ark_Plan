# Controller 遥控器工程

这是 STM32F103 遥控器工程，负责读取摇杆、扫描 12 个按键，通过 NRF24L01 向 F405 主控发送遥控数据，并预留接收主控反馈的显示页面。

## 工程结构

```text
User/
  App/
    app_remote.c/.h              遥控器应用逻辑
  Bsp/
    bsp_ad.c/.h                  ADC 摇杆采样
    bsp_key.c/.h                 12 键状态机
    bsp_nrf24l01.c/.h            NRF24L01 芯片驱动
    bsp_nrf24l01_define.h        NRF24L01 寄存器和命令定义
    bsp_oled.c/.h                OLED 显示驱动
    bsp_oled_data.c/.h           OLED 图片/字模数据
    bsp_radio_link.c/.h          无线链路适配层
    bsp_time.c/.h                TIM1 1ms tick 和系统时间接口
  Module/
    module_periodic_timer.c/.h   通用软件周期定时器
    module_remote_protocol.c/.h  遥控协议编解码
```

## 分层说明

- `Bsp` 层只处理板级硬件和芯片驱动，比如 GPIO、ADC、OLED、NRF24L01、TIM1 tick。
- `Module` 层放可移植逻辑，不直接依赖 STM32 HAL，比如遥控协议编解码、软件周期定时器。
- `App` 层组织遥控器业务逻辑，不直接操作 NRF 寄存器或 HAL 外设。

## NRF24L01 配置

- 地址：`{'n', 'a', 'i', 'w', 'a'}`，也就是 ASCII `naiwa`
- 地址宽度：5 字节
- 频道：`RF_CH = 0x02`，即 2402 MHz
- 空中速率：2 Mbps
- 自动重发：间隔 750 us，最多 15 次
- payload 长度：32 字节

遥控器和 F405 两边的地址、频道、速率、payload 长度必须一致。

## 控制数据包

控制包由 `module_remote_protocol` 编解码：

```text
byte0  mode
byte1  lh
byte2  lv
byte3  rh
byte4  rv
byte5  key, 低 6 位有效
byte6  key_seq
byte7~31  预留
```

其中：

- `lh/lv/rh/rv`：摇杆值，范围约为 `-100 ~ +100`
- `key`：按键编号，`1~12`，`0` 表示无按键事件
- `key_seq`：按键事件序号，每次按键按下递增，用于接收端识别新事件

## 按键用途

- `K10`：欢迎界面进入运行界面
- `K9`：遥控器本地显示模式切换
- `K1~K12`：发送给 F405 的模式/功能按键事件

F405 端建议用 `key_seq` 判断是否收到新的按键事件，用 `key` 判断具体按下的是哪个键。

## 反馈页面

遥控器在反馈页会尝试接收 F405 回传的麦轮四电机数据：

```c
typedef struct
{
  int8_t pwm[4];
  float speed[4];
} RemoteProtocolFeedbackPacket_t;
```

当前 F405 工程还没有实现反馈发送，所以这个页面目前只是预留显示入口。后续建议用 NRF24L01 ACK payload 或固定时隙方式回传。

## 编译

```powershell
cmake --build --preset Debug
```

编译产物在：

```text
build/Debug/Controller.elf
```

## 调试建议

- 遥控器 OLED 会显示当前摇杆值、按键提示和发送成功率图标。
- 如果更改 NRF 地址、频道或协议字段，F405 工程必须同步修改并重新烧录。
