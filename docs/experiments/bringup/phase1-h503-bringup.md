# Phase 1 底板启动验证 (`biasctrl_h503`)

## 烧录与启动验证 (2026-11-22)

- **固件镜像**: `build/h503_bringup/zephyr/zephyr.elf` (commit `3413de0`)
  - Flash: `22,264 / 110,592 B` (`20.1%`)
  - 静态 RAM: `5,728 / 24,576 B` (`23.3%`)
- **SWD 探针**: `Devanarchy DAP42 CMSIS-DAP` (`4D3233200643324B07802400`)
- **烧录命令**: `west flash -d build/h503_bringup` (退出码 `0`, 写入 `22,528` 字节，速率 `19.85 kB/s`)
- **串口采集** (`/dev/cu.usbmodem1102` @ `115200 bps`):
  ```text
  *** Booting Zephyr OS build 41d10b597304 ***
  biasctrl_h503 bring-up, configured SYSCLK 250000000 Hz
  I2C1 ACK at 0x3C
  I2C1 ACK at 0x50
  I2C1 scan complete, 2 responding addresses
  ```
- **自动验证结果**:
  - `8 MHz` HSE + `PLL1` (`250 MHz` SYSCLK) 时钟启动正常
  - `USART1` (`PA9/PA10`) 控制台通过板载 `STM32F042` 虚拟串口输出
  - `I2C1` (`PB8/PB7`) 应答地址 `0x3C` (`OLED1`) 和 `0x50` (`U21` `P24C02A` 经 FPC2 连接)

## 人工观察与按键验证（2026-11-22 已确认）
- [x] 用户确认 LED1（PC13）以 1 Hz 稳定闪烁，每 500 ms 翻转一次。
- [x] 用户拨动或按下 U18（KEY1、KEY2、KEY3），串口分别输出 `KEY1 pressed` / `KEY2 pressed` / `KEY3 pressed`。20 ms 软件去抖列入后续阶段。
