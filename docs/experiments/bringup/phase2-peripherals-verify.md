# Phase 2 分板外设验证 (`biasctrl_h503` + `20261110-MZM`)

## 烧录与自动诊断 (2026-11-22)

- **固件镜像**: `build/h503_p2/zephyr/zephyr.elf`
  - Flash: `44,920 / 110,592 B` (`40.6%`)
  - 静态 RAM: `9,404 / 24,576 B` (`38.3%`)
- **SWD 探针**: `Devanarchy DAP42 CMSIS-DAP` (`4D3233200643324B07802400`)
- **烧录命令**: `west flash -d build/h503_p2` (退出码 `0`, 擦除 `49,152` B, 写入 `45,056` B，速率 `20.51 kB/s`)
- **板级时钟配置**: `PLL1` `div-q = 5` (`PLL1_Q = 100 MHz`, `SYSCLK = 250 MHz`), `SPI1` `6.25 MHz` (`bias_dac`), `SPI2` `12.5 MHz` (`bias_adc`), FPC2 全部引脚保持 `slew-rate = "low-speed"`.
- **串口采集** (`/dev/cu.usbmodem102` @ `115200 bps`, 本地原始日志 `.agent-bridge/p2_hw_verify_uart.log`):
  ```text
  *** Booting Zephyr OS build 41d10b597304 ***
  biasctrl_h503 Phase 2 diag, SYSCLK 250000000 Hz
  analog_board init (20261110-MZM): 0
  oled_lite init: 0
  keys debounce init (20 ms): 0
  eeprom probe: 0
  ads131m02 init (64 kSPS): 0
  ```
- **自动验证结果**:
  - **`I2C1` (`PB8/PB7`)**: 应答地址 `0x3C` (`OLED1`) 和 `0x50` (`U21` `P24C02A`).
  - **`DAC8568` (`SPI1` + `GPDMA1`)**: 内部 2.5 V 基准已启用，启动时 100 次 DMA 写入通过 (`init=0 status=0`), `dac dc 0 32768 -> 0`, `dac pilot 0 32768 -> 0`, `dac dc/pilot 0 0 -> 0`.
  - **`TS5A23159` (`pilot_mux`)**: 八路开关控制线开机均为低电平; `mux 0 0 1 -> 0`, `mux 0 1 1 -> 0`, 双导频冲突被拒绝 (`mux 1 1 1 -> -16` `-EBUSY`), 未映射路由被拒绝 (`mux 0 2 1 -> -2` `-ENOENT`), `mux off -> 0`.
  - **`ADS131M02` (`SPI2` + `GPDMA1` + `PA11` `DRDY#` EXTI)**:
    - `ID=0x2205`, `STAT=0x0103`, `MODE=0x1111` (`24-bit`, CRC-16-CCITT 已启用).
    - `64 kSPS` (`CLK=0x0322`, TBM): `1000/1000` 帧通过 CRC 校验，用时 `15,607 us` (`64,009 SPS`, `err=0`, `stop=0`; `CH0 min=-7362063 mean=5151 max=6154969`, `CH1 min=-14407 mean=4503 max=6615331`).
    - `32 kSPS` (`CLK=0x0302`, OSR=128 HR): `1000/1000` 帧通过 CRC 校验，用时 `31,215 us` (`32,003 SPS`, `err=0`, `stop=0`; `CH0 min=-117612 mean=1337 max=3676`, `CH1 min=-123830 mean=-12922 max=-10668`).
  - **`P24C02A` (`eeprom_board`)**: `eeprom write-default -> 0` (`crc=0xA0D5AC42`) 复位后重读成功 (`eeprom probe: 0`, `id=20261110-MZM ver=1 len=148 tia=10000 ohm`).
  - **`OLED1` (`oled_lite`) & `KEY1–KEY3`**: `oled_lite` 更新为 `5x7` （`6x8` 字格）字体 (`oled P2 HW VERIFY OK -> 0`), `keys debounce init (20 ms): 0`, `KEY1=0(0) KEY2=0(0) KEY3=0(0)`.

## 人工观察与按键验证（2026-11-22 已确认）
- [x] 用户确认 OLED1 清晰显示 `BIASCTRL H503` 和 `P2 HW VERIFY OK`（5×7 字体）。
- [x] 用户确认 LED1（PC13）每 500 ms 翻转，1 Hz 心跳正常，无启动故障。
- [x] 用户确认拨动或按下 WS-001CB 三键时，20 ms 去抖后，115200 bps 串口每次仅报告一次 `KEYx pressed` / `released`。
