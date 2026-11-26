# Phase 4 单 MZM 64 kSPS 光学闭环验证（biasctrl_h503 + 20261110-MZM）

## 1. 固件构建与烧录（2026-11-26，极性修正后）

- **ELF**: `build/h503_p4/zephyr/zephyr.elf`
  - `text=79576 B, data=1152 B, bss=14662 B`
  - **Flash**: `80,728 / 110,592 B (73.0%)` — PASS
  - **RAM**: `15,814 / 24,576 B (64.3%)` — PASS
- **SWD 探针**：`Devanarchy DAP42 CMSIS-DAP`（`4D3233200643324B07802400`）
- **烧录**：`west flash -d build/h503_p4` 退出码 0。擦除 10 个扇区共 81,920 B，写入 79 页共 80,896 B，速率 20.27 kB/s。
- **修正后回归**：
  - `-Wall -Wextra -Werror` 下 7 个文件零警告：`drivers/analog_board/mzm_20261110.c`、`drivers/dac8568/dac8568.c` 和 app/src/ 下的 main.c、rt_pipeline.c、cal_service.c、ctrl_thread.c、cli_service.c。
  - QEMU `qemu_cortex_m3` `ztest`: `5/5 PASS` (`27.059 s`, `PROJECT EXECUTION SUCCESSFUL`)
  - Host CTest: `13/13 PASS` (`5.26 s`)

## 2. 子板描述符、DAC 修正与接光前验证

依据子板网表，U10 DAC8568 以内部 2.500 V 基准输出 0..5.000 V，U11B OPA4192 输出 VA = 4 × (VOA − 2.500 V)，范围 ±10 V。导频增益 −R23/R22 = −12 kΩ / 120 kΩ = −0.10 V/V。U12A 的 R17 为 47 kΩ。据此更新 U21 默认描述符与导频换算系数并烧录。
- `subtractor_gain_ppm = 8000000`, `subtractor_offset_uv = -10000000`, `tia_gain_ohms = 47000` (`VA = -10.0..+10.0 V`, `0 V` idle `safe_code = 32768`)
- `RT_PIPELINE_PILOT_MAX_V = 0.25f` V peak at `VA` electrode (`SCAN_PILOT_V = 0.13f` V peak)

参照 biascontrol_h523 的 drv_dac8568.c，修正本工程 DAC8568 驱动：
1. 通道命令由 0x00（WRITE_REG）改为 0x03（CMD_WRITE_UPDATE），在 SYNC# 上升沿更新输出。PB2 保持 low-speed，LDAC# 低脉冲延长至 32 个 NOP。
2. startup() 在 POST_KERNEL 90 阶段用 k_busy_wait 等待。先 RESET（0x07000000）后等 2 ms，再 REF_ENABLE（0x08000001）后等 5 ms（C46 = 10 µF），最后 CH_ALL = 32768（0x03F80000，偏压 0.000 V，导频 DAC 中点 2.500 V），连续写中点 100 次。
3. SPI1 保持 12.5 MHz（SPI1->CFG1 = 0x2007801F），rt_pipeline.c 中 wait_limit = 8 µs。空闲时 SWD 读回 tx_word = 0x03080000（safe_code = 32768）。

早期实板执行 64 kSPS 的 `scan vpi` 时，GPDMA1 完成中断忙等和线程侧 SPI1 直流写入与下一个 DRDY# 边沿重叠，报 rt_fault=-16（-EBUSY）。所有 FPC2 引脚保持 low-speed，并作如下修正。
1. dac8568.{c,h} 用 finish_if_eot_locked() 轮询寄存器取代 GPDMA1 TC 中断忙等，增加 ISR 可调用的 dac8568_set_dc_code_async()。
2. biasctrl_h503.dts 将 bias_dac 时钟提高至 12.5 MHz，与 bias_adc 一致。32 位帧占线时间减半至 2.56 µs，100 次启动写入通过。
3. rt_pipeline.c 的 publish_block() 改用硬件单精度 goertzel_result_fast()，取代软件双精度 hypot/atan2。
4. 采样期间由 DRDY# ISR 独占 SPI1，先提交 DITHER1 再计算 Goertzel，使计算与传输重叠。在计算后的时隙经 dac8568_set_dc_code_async() 写入 5 Hz 的 VA 直流更新。

### 接光前串口采集（`/dev/cu.usbmodem102`，115200 bps，本地日志 `.agent-bridge/p4_hw_preflight_uart.log`）
- **U21 初始化与复位重读**：
  - `eeprom init` -> `eeprom init: ok crc=0x1E71EE41`
  - `reinit` -> `reinit: uncalibrated (err=-77), run cal bias`
  - `pyocd reset` 后启动输出：
    ```text
    biasctrl_h503 Phase 4, SYSCLK 250000000 Hz
    analog_board init (20261110-MZM): 0
    oled_lite init: 0
    keys init: 0
    ctrl_thread init: -77
    cli_service start: 0
    Phase 4 CLI ready. Type 'help'.
    ```
    返回 −77（−EBADMSG）。此时 ±10 V 描述符有效，仿射记录尚为空，符合 cal bias 前的预期。status 报告 `state=IDLE init=1 ... bias=0 mV`。
- **CLI**：help、status 与目标 min、max、custom 135、quad 均通过，`target custom abc` 返回 −22（−EINVAL）。
- **实时扫描**：执行 `scan vpi` 后 `stop`，连续采样 3.5 s，VA 每秒更新 5 次。
  - 扫描中 (`2.5 s`): `state=SCAN ... bias=-9531 mV ... updates=12 pass=1 step=6 ... drops=0 crc_errs=0 fault=0 cleanup_err=0 load_err=-77 rt_fault=0` (`-10.000 V + 6/256 * 20.000 V = -9.531 V`)
  - 停止后 (`3.5 s`): `state=IDLE ... bias=0 mV ... updates=17 pass=1 step=8 ... drops=0 crc_errs=0 fault=0 cleanup_err=0 load_err=-77 rt_fault=0`

## 3. 光路闭环验证（2026-11-26）

接光前调整底板 R9，使 LMR51430 输出 VSYS = 5.339 V，旁路子板 U7 后直供 +5V。子板拆除 U14/U16/U17/U20 四颗 TS5A23159，固定飞线 U10.7 → C48.1（VA）、U10.10 → C56.1（VC）。

ctrl_thread.c 使用 ctrl_bias_scan_begin_fit(&scan, -1)。U12A 反相和 2 kHz 接收链相移使暗谷 MIN 的 H2 正交分量为负（相位约 −1.708 rad），亮峰 MAX 为正（约 +1.433 rad），故 null_q_sign = -1。

完整串口记录保存在采集机 `.agent-bridge/p4_hw_closed_loop_uart.log`（OPTICAL_VERIFY_PASS，/dev/cu.usbmodem102，115200 bps）。

- [x] 连接激光器 → MZM → PD → 子板交流与直流输入，VA 偏压与 DITHER1 接 MZM。
- [x] 用户确认 OLED1 显示 BIASCTRL H503 P4，LED1 正常时 500 ms 翻转，故障时 150 ms 翻转。
- [x] 执行 cal bias 两遍扫压，检查半波电压、暗谷及仿射拟合收敛。
  - 扫描直流范围: `-0.992 mV` （暗谷） .. `787.601 mV` （亮峰）; 正交点基波峰值 `h1 ≈ 355.4 mV`, 二次谐波峰值 `h2 ≈ 5.72 mV`.
  - 校准收敛结果: **`Vpi = 5382 mV`**, **`null = -3029 mV`**, `updates = 1028`, `pass = 2`, `step = 256`, `drops = 0`, `crc_errs = 0`, `fault = 0`, `rt_fault = 0`.
- [x] 执行 save 后复位，U21 自动重载（ctrl_thread init: 0）。
  - `save` -> `save: ok`; 执行 `pyocd reset` 后启动输出 `ctrl_thread init: 0`; `status` 输出 `state=IDLE init=1 cal=1 vpi=5382 mV null=-3029 mV load_err=0 drops=0 crc_errs=0 fault=0 rt_fault=0`.
- [x] 逐点确认 locked=1、hold_assist=1、drops=0、crc_errs=0 及光学极性。

| 目标 | 锁定时间 | hold_assist 生效时间 | 稳态偏压 | H1 幅相 | H2 幅相 | 直流 | 归一化强度 | 更新次数 | 丢块 / CRC / 故障 |
| --- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `QUAD(90)` | `1.0 s` | `5.2 s` | `-340 mV` | `355.394 mV@-2.287 rad` | `0.311 mV@2.447 rad` | `418.561 mV` | `53.2%` | `46` | `0 / 0 / 0` |
| `MAX(180)` | `1.0 s` | `5.2 s` | `2285 mV` | `2.981 mV@0.853 rad` | `5.721 mV@1.433 rad` | `787.601 mV` | `100.0%` | `46` | `0 / 0 / 0` |
| `MIN(0)` | `2.1 s` | `7.3 s` | `-3131 mV` | `7.114 mV@0.857 rad` | `5.720 mV@-1.708 rad` | `-0.992 mV` | `0.0%` | `57` | `0 / 0 / 0` |
| `CUSTOM(135)` | `1.0 s` | `5.2 s` | `1006 mV` | `245.850 mV@-2.286 rad` | `3.991 mV@1.474 rad` | `678.135 mV` | `86.1%` | `46` | `0 / 0 / 0` |
