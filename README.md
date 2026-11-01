# STM32F407 Dual-Slot OTA Bootloader + RT-Thread Actuator Control

Firmware for the STM32F407VET6 (Cortex-M4, 168 MHz, 512 KB Flash / 128 KB RAM), consisting of two parts:

- **Bootloader** (bare-metal, no RT-Thread): dual-slot (A/B) OTA update with serial Ymodem and ESP8266 WiFi channels, CRC32 validation, and power-loss recovery.
- **Application** (RT-Thread): OLED display, ESP8266 WiFi, an interactive terminal shell, and a set of motor-actuator controls (relays, servos, electric push rods, lift platform).

## Features

### Bootloader
- Dual-slot A/B OTA: `APP1` is the always-booted running slot; `APP2` is a byte-level backup for rollback on a failed update.
- Validation and recovery: firmware CRC32 check; a boot-meta state machine survives power loss mid-OTA so a valid firmware is always bootable.
- Two OTA channels:
  - **Serial OTA**: Ymodem over UART1.
  - **WiFi OTA**: ESP8266 acts as a TCP server, Ymodem-over-TCP.
- Status indication: fast-blink = connecting, solid = client connected, 3 slow blinks = update complete.

### Application
- **Terminal shell**: local UART1 (USB-serial) + remote UART2 (ESP8266 TCP server on port 23, telnet-capable).
- **WiFi**: STA / AP mode switching, with AP credentials and configuration persisted to flash.
- **Actuator control**:
  - 8-channel relay driver (PE0..PE7, active-low).
  - PCA9685 16-channel PWM / servo driver (software I2C: SCL=PB6, SDA=PB7).
  - Electric push rod `MP` (relays 6-8, H-bridge), CP push rod (relays 4-5), lift platform `LIFT` (relays 2-3).
  - Servo motion, limit switches, presets, and thread-script storage.

## Flash Layout

STM32F407VET6 has 512 KB of flash with sector sizes 16/16/16/16/64/128/128/128 KB:

| Region | Base | Size | Purpose |
| :--- | :--- | :---: | :--- |
| S0 | 0x08000000 | 16 KB | Bootloader |
| S1 | 0x08004000 | 16 KB | Boot meta (slot CRC + OTA state) |
| S2 | 0x08008000 | 16 KB | Config (WiFi / mode / OTA server) |
| S3~S5 | 0x0800C000 | 208 KB | APP1 (primary slot, always booted) |
| S6~S7 | 0x08040000 | 256 KB | APP2 (backup copy) |

Every region is a whole number of sectors, so each can be erased independently. The firmware image is always `FW_SIZE = APP1_SIZE = 208 KB`; APP2's extra 48 KB are spare, and only its first `FW_SIZE` bytes are copied or validated.

## Directory Structure

```
.
├── bootloader/            Bare-metal bootloader (Keil MDK project)
│   ├── main.c             Entry point + OTA state machine
│   ├── esp8266_at.c       ESP8266 AT commands / TCP passthrough
│   ├── ymodem.c           Ymodem receiver
│   ├── flash_ops.c        Internal flash erase/write
│   ├── partition.h        Flash layout (shared by bootloader and app)
│   └── oled.c             OLED display
├── applications/          RT-Thread application
│   ├── main.c             Application entry point
│   ├── hardware/          pca9685 / relay / pushrod / cprod / lift
│   ├── esp8266/           WiFi driver (STA/AP)
│   ├── oled/              OLED display
│   ├── shell/             Terminal shell
│   ├── wifi_config.c      WiFi configuration
│   ├── preset_store.c     Preset storage
│   ├── thread_store.c     Thread-script storage / runner
│   ├── servo_motion.c     Servo motion
│   └── flash_config.c     Unified flash config read/write
├── ota.py                 One-click serial OTA (auto COM detect + Ymodem)
├── tcp_server.py          One-click WiFi OTA (LAN scan + paced push)
├── ymodem_send.py         Ymodem sender
└── hex2bin.py             HEX to BIN converter
```

## Building

- **Application**: open `project.uvprojx` in Keil MDK5, or build with the RT-Thread `scons` toolchain. Output: `rtthread.bin`.
- **Bootloader**: open `bootloader.uvprojx` in Keil MDK5. Output: `bootloader.bin` (flash at 0x08000000).

> The application link address is `0x0800C000` (the `APP1_BASE` in `partition.h`).

## Usage

### Serial Shell

Connect at 115200-8-1-N, reset the board, and type `help` to list commands. Common commands:

| Command | Description |
| :--- | :--- |
| `help` / `hardware` | Show help |
| `info` / `who` / `reset` | System info / connections / reset |
| `led on\|off\|blink` | LED control |
| `oled <line> <text>` | OLED text (lines 2-4) |
| `wifi show\|save\|delete\|mode` | WiFi config and STA/AP switching |
| `S<n> <deg> [ms]` | Servo n (0-15) to angle |
| `MP <UP\|DN> [ms]` | Electric push rod (relays 6-8) |
| `CP <UP\|DN> [ms]` | CP push rod (relays 4-5) |
| `LIFT <UP\|DN> [ms\|end]` | Lift platform (relays 2-3, limit-switch aware) |
| `set S<n> P<m> ...` | Save servo / push-rod presets |
| `Run <name>` / `edit <name>` | Run / edit a thread script (Thread1..16) |
| `stop` | Stop everything and reset actuators |

### OTA Update

**Serial OTA** (auto-detects the COM port, transfers `rtthread.bin` via Ymodem):

```bat
ota.bat
```

**WiFi OTA** (scans the LAN for the ESP8266 and pushes the firmware in paced chunks):

```bat
tcp_server.bat
```

Or manually: type `ota serial` / `ota wifi` in the shell → the device reboots into the bootloader → send the firmware from the host → after validation it jumps to the new firmware.

## Pinout

| Function | Pins |
| :--- | :--- |
| Console UART1 | PA9 (TX) / PA10 (RX) |
| ESP8266 UART2 | PA2 (TX) / PA3 (RX) |
| LED (active-low) | PA1 |
| Relays 1-8 (active-low) | PE0..PE7 |
| PCA9685 software I2C | SCL=PB6, SDA=PB7 |
| Push-rod limit switch | PC0 |
| Lift bottom limit switch | PC1 (S2) |
