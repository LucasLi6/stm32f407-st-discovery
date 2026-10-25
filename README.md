# STM32F407 双分区 OTA Bootloader + RT-Thread 执行器控制

基于 STM32F407VET6（Cortex-M4, 168MHz, 512KB Flash / 128KB RAM）的固件工程，包含两部分：

- **Bootloader**（裸机，不依赖 RT-Thread）：双分区（A/B）OTA 升级，支持串口 Ymodem 与 ESP8266 WiFi 两种升级通道，带 CRC32 校验与掉电恢复。
- **应用**（RT-Thread）：OLED 显示、ESP8266 WiFi、交互式终端 Shell，以及一套电机执行器控制（继电器、舵机、电动推杆、升降台）。

## 功能特性

### Bootloader
- 双分区 A/B OTA：`APP1` 为常驻运行槽位，`APP2` 为字节级备份槽位，升级失败可回滚。
- 校验与恢复：镜像 CRC32 校验；OTA 进行中掉电后，通过 Boot meta 状态机恢复，保证固件始终可用。
- 两种 OTA 通道：
  - **串口 OTA**：UART1 上的 Ymodem 协议。
  - **WiFi OTA**：ESP8266 建立 TCP 服务器，Ymodem-over-TCP。
- 状态指示：LED 快闪 = 连接中，常亮 = 客户端已连接，慢闪 3 次 = 升级完成。

### 应用
- **终端 Shell**：本地 UART1（USB 串口）＋ 远程 UART2（ESP8266 TCP Server，端口 23，支持 telnet）。
- **WiFi**：STA / AP 双模式切换，AP 凭据与配置持久化存储。
- **执行器控制**：
  - 8 路继电器（PE0..PE7，低电平有效）。
  - PCA9685 16 通道 PWM / 舵机驱动（软件 I2C：SCL=PB6, SDA=PB7）。
  - 电动推杆 MP（继电器 6-8，H 桥）、CP 推杆（继电器 4-5）、升降台 LIFT（继电器 2-3）。
  - 舵机运动、限位开关、预设（preset）与线程脚本（thread script）存储。

## Flash 分区表

STM32F407VET6 共 512KB Flash，扇区大小 16/16/16/16/64/128/128/128 KB：

| 区域 | 起始地址 | 大小 | 用途 |
| :--- | :--- | :---: | :--- |
| S0 | 0x08000000 | 16 KB | Bootloader |
| S1 | 0x08004000 | 16 KB | Boot meta（槽位 CRC + OTA 状态） |
| S2 | 0x08008000 | 16 KB | 配置（WiFi / 模式 / OTA 服务器） |
| S3~S5 | 0x0800C000 | 208 KB | APP1（主槽位，始终从此启动） |
| S6~S7 | 0x08040000 | 256 KB | APP2（备份副本） |

每个分区均为整数个扇区，可独立擦除。固件镜像固定为 `FW_SIZE = APP1_SIZE = 208 KB`，APP2 多出的 48 KB 为冗余空间，仅前 208 KB 参与拷贝与校验。

## 目录结构

```
.
├── bootloader/            裸机 Bootloader（Keil MDK 工程）
│   ├── main.c             入口 + OTA 状态机
│   ├── esp8266_at.c       ESP8266 AT 指令 / TCP 透传
│   ├── ymodem.c           Ymodem 接收
│   ├── flash_ops.c        内部 Flash 擦写
│   ├── partition.h        Flash 分区定义（bootloader 与应用共享）
│   └── oled.c             OLED 显示
├── applications/          RT-Thread 应用
│   ├── main.c             应用入口
│   ├── hardware/          pca9685 / relay / pushrod / cprod / lift
│   ├── esp8266/           WiFi 驱动（STA/AP）
│   ├── oled/              OLED 显示
│   ├── shell/             终端 Shell
│   ├── wifi_config.c      WiFi 配置
│   ├── preset_store.c     预设存储
│   ├── thread_store.c     线程脚本存储 / 运行器
│   ├── servo_motion.c     舵机运动
│   └── flash_config.c     统一 Flash 配置读写
├── ota.py                 一键串口 OTA（自动识别 COM、Ymodem 传输）
├── tcp_server.py          一键 WiFi OTA（局域网扫描 + 分块推送）
├── ymodem_send.py         Ymodem 发送端
└── hex2bin.py             HEX 转 BIN
```

## 构建

- **应用**：Keil MDK5 打开 `project.uvprojx`，或用 RT-Thread `scons` 工具链构建，输出 `rtthread.bin`。
- **Bootloader**：Keil MDK5 打开 `bootloader.uvprojx`，输出 `bootloader.bin`（烧录到 0x08000000）。

> 应用链接地址为 `0x0800C000`（对应 `partition.h` 中的 `APP1_BASE`）。

## 使用说明

### 串口 Shell

串口 115200-8-1-N，复位后进入 Shell，输入 `help` 查看命令。常用命令：

| 命令 | 说明 |
| :--- | :--- |
| `help` / `hardware` | 显示帮助 |
| `info` / `who` / `reset` | 系统信息 / 连接状态 / 复位 |
| `led on\|off\|blink` | LED 控制 |
| `oled <行> <文本>` | OLED 显示（行 2-4） |
| `wifi show\|save\|delete\|mode` | WiFi 配置与 STA/AP 切换 |
| `S<n> <度> [ms]` | 舵机 n（0-15）转到指定角度 |
| `MP <UP\|DN> [ms]` | 电动推杆（继电器 6-8） |
| `CP <UP\|DN> [ms]` | CP 推杆（继电器 4-5） |
| `LIFT <UP\|DN> [ms\|end]` | 升降台（继电器 2-3，支持限位） |
| `set S<n> P<m> ...` | 保存舵机 / 推杆预设 |
| `Run <name>` / `edit <name>` | 运行 / 编辑线程脚本（Thread1..16） |
| `stop` | 停止一切并复位执行器 |

### OTA 升级

**串口 OTA**（自动识别 COM 口，通过 Ymodem 传输 `rtthread.bin`）：

```bat
ota.bat
```

**WiFi OTA**（局域网扫描 ESP8266，自动分块推送）：

```bat
tcp_server.bat
```

或手动流程：Shell 输入 `ota serial`（串口）/ `ota wifi`（WiFi）→ 设备重启进入 Bootloader → 主机发送固件 → 校验成功后跳转新固件。

## 硬件连接

| 功能 | 引脚 |
| :--- | :--- |
| 串口控制台 UART1 | PA9 (TX) / PA10 (RX) |
| ESP8266 UART2 | PA2 (TX) / PA3 (RX) |
| LED（低电平点亮） | PA1 |
| 继电器 1-8（低电平有效） | PE0..PE7 |
| PCA9685 软件 I2C | SCL=PB6, SDA=PB7 |
| 推杆限位开关 | PC0 |
| 升降台下限位开关 | PC1（S2） |
