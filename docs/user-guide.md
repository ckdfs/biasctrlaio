# biasctrlaio 用户使用说明

本手册面向 `biasctrlaio` 分板式电光调制器偏压控制器（`STM32H503CBT6` 数字控制底板 `20261110-底板` + `20261110-MZM` 模拟子板）的硬件接线、固件编译烧录、串口校准操作及故障排查。

## 1. 当前功能边界

- **已完成（Phase 1–4 实板闭环基线）**：
  - 单 MZM `64 kSPS` 实时采集、Goertzel `1 kHz / 2 kHz` 谐波解调与 `5 Hz` 抗饱和 PI 闭环锁定（支持 `QUAD (90°)`、`MIN (0°)`、`MAX (180°)`、`CUSTOM <deg>`）。
  - 两遍扫压校准（`cal bias`，`Vpi` 与四参数仿射矩阵），并写入子板 `U21 (P24C02A)` 实现掉电重启自动恢复。
  - 串口非阻塞命令行控制与 telemetry 输出；`OLED1` 实时状态显示与 `LED1` 闪烁状态指示。
- **规划中（Phase 5，待开发）**：
  - 单 MZM 运行时任意偏置点无缝切换（`QUAD <-> MIN <-> MAX <-> CUSTOM` 不停止实时采样、不跨周期跳枝）。
  - 无开关实板 3 臂 DPMZM（`CN1: VA / VB / VC`）固定双导频闭环控制。
  - `WS-001CB` 三向拨轮本地菜单与 `OLED1` 交互（脱离上位机独立完成目标切换、启停与校准保存）。

---

## 2. 硬件组成、接线与当前原型飞线状态

板间引脚与电源测试点见[电气设计](design/electrical.md)，机械装配与外壳步骤见[机械与外壳设计](design/mechanical.md)，返修位置见[PCB 勘误](design/pcb-errata.md)。

### 2.1 板卡连接与供电
1. 通过 `30-pin FPC2` 排线连接数字底板 `20261110-底板` 与模拟子板 `20261110-MZM`（连接前断电确认防呆与方向）。
2. **当前原型实板硬件改造状态（已验证基线）**：
   - **跨板 `+5V` 供电**：拆除模拟子板 `U7 (TPS7A2050)` 并旁路，调整底板 `U3 (LMR51430)` 反馈电阻 `R9` 使 `VSYS = 5.339 V`（低于 `DAC8568` 最大耐压 `5.5 V`）后直接飞线供 `+5V`。
   - **模拟子板直连双导频飞线**：拆除全部 4 颗 `TS5A23159` 模拟开关（`U14 / U16 / U17 / U20`），固定飞线连接为：
     - `U10.7 (DAC8568 CH6 DITHER1)` -> `C48.1`（叠加至偏压通道 `VA`）
     - `U10.10 (DAC8568 CH7 DITHER2)` -> `C56.1`（叠加至偏压通道 `VC`）
     - `VB` 通道保持纯直流输出（无交流导频注入）
   - **`VF` 通道说明**：`R56` 错焊为 `1 kΩ`，当前单 MZM 与 DPMZM 仅使用 `CN1 (VA / VB / VC)`，待后续按需返修。

### 2.2 光路与调测接口连接
| 接口 | 信号与用途 | 接线要求 |
| --- | --- | --- |
| `CN1` (`VA / VB / VC`) | 偏置与导频输出至调制器电极 | 单 MZM 接 `VA` 与 `GND`；DPMZM 接 `VA`（子臂 I）、`VC`（子臂 Q）、`VB`（主臂直流） |
| 模拟子板光电输入 | `PD -> U12A` 跨阻输入 + 直流监测 `CH1` | 外部激光器 → 调制器 → 光电探测器接入模拟板 AC/DC 输入端 |
| 底板 USB（`U19 STM32F042`） | DAPLink 调试串口（`USART1`）与 SWD 烧录口 | 串口波特率 **`115200 bps, 8N1`**，无硬件流控；macOS 常见设备名为 `/dev/cu.usbmodem*` |

---

## 3. 编译环境、固件烧录与自动化测试

首次拉取仓库后按系统类型执行一次初始化（详参 [README.md](../README.md)）：
- macOS / Linux：`./scripts/bootstrap.sh && source scripts/env.sh`
- Windows PowerShell：`.\scripts\bootstrap.ps1` 后运行 `. .\scripts\env.ps1`

### 3.1 固件编译与资源预算检查
```bash
source scripts/env.sh
west build -b biasctrl_h503 -d build/h503_p4 app --pristine
arm-zephyr-eabi-size build/h503_p4/zephyr/zephyr.elf
python3 scripts/check_mem_budget.py --elf build/h503_p4/zephyr/zephyr.elf
```
硬性门禁：静态 RAM（`data + bss`）`<= 24576 B`，Flash（`text + data`）`<= 110592 B`。

### 3.2 烧录与复位（CMSIS-DAP / pyOCD）
```bash
source scripts/env.sh
west flash -d build/h503_p4
# 仅硬件复位
.venv/bin/pyocd reset -t stm32h503cbtx
```

### 3.3 宿主机与 QEMU 回归测试
```bash
# 宿主机 DSP / 控制 / 物理光路仿真测试（13 项）
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure

# Zephyr 集成测试（QEMU cortex-m3，5 项）
source scripts/env.sh
export PATH="$ZEPHYR_SDK_INSTALL_DIR/hosttools/usr/bin:$PATH"
west build -b qemu_cortex_m3 -d build/ztest_p4 tests/ztest -t run
```
> 注意：QEMU 执行完 `PROJECT EXECUTION SUCCESSFUL` 后不会主动退出，交互终端中需按 `Ctrl+a` 再按 `x` 退出。

---

## 4. 首次启用与单 MZM 闭环锁定操作流程

通过串口终端（`115200 8N1`，以 `\r` 或 `\n` 结尾）连接后，应看到提示符 `Phase 4 CLI ready. Type 'help'.` 与 `> `。

### 4.1 首次初始化子板 EEPROM（新模拟板或更换参数时执行）
```text
> eeprom init
eeprom init: ok crc=0x1E71EE41
```
向 `U21 (P24C02A)` `0..147 B` 写入与 `20261110-MZM` 原理图匹配的默认板卡描述符（`±10 V` 量程、`47 kΩ` TIA）。

### 4.2 双遍扫压校准并保存（连接完整光路后执行）
1. 确认激光器已开启且 MZM 光输出已接入探测器；`VA` 处于空闲 `0 mV`。
2. 执行两遍校准（单次耗时约 `210 s`，期间 `VA` 从 `-10.000 V` 扫描至 `+10.000 V`）：
   ```text
   > cal bias
   cal bias: ok
   ```
3. 通过 `status` 观察扫描进度（`state=CAL pass=1..2 step=0..256`），完成后自动回到 `state=IDLE` 且 `cal=1`、`vpi` 与 `null` 更新为实测值。
4. 写入 `U21` 非易失保存，并通过复位验证自动加载：
   ```text
   > save
   save: ok
   ```
   硬件复位后串口启动打印 `ctrl_thread init: 0`，表示已自动载入有效校准记录并可跳过重校。

### 4.3 启动、切换目标与停止闭环
```text
> target quad              # 选择正交点 (90 deg)，可选 min (0) / max (180) / custom <deg>
target: ok
> start
start: ok
> status                   # 观察 locked=1, hold_assist=1, bias, h1/h2, dc 及误码计数
> stop                     # 停止控制并将 VA 复零至 0 mV
stop: ok
```
> 当前 Phase 4 固件中，`target`、`cal bias`、`scan vpi`、`save`、`load` 均要求控制器处于 `IDLE` 状态；切换工作点前先执行 `stop`。运行时无间断切换将在 Phase 5 支持。

---

## 5. 串口 CLI 命令与 `status` 字段参考

### 5.1 命令列表（来自 [cli_service.c](../app/src/cli_service.c)）
| 命令 | 功能说明 | 前置状态要求 | 典型成功返回 |
| --- | --- | --- | --- |
| `help` | 打印全部 CLI 命令列表 | 任意 | 命令清单 |
| `status` | 打印状态机、目标、偏压、谐波、光功率及错误计数 | 任意 | 四行状态报告 |
| `cal bias` | 两遍扫描校准（测 `Vpi`、零点偏压与仿射矩阵 `[o, M]`） | `IDLE` | `cal bias: ok` |
| `scan vpi` | 仅执行第一遍开环扫描，测量并报告 `scan_vpi` | `IDLE` | `scan vpi: ok` |
| `target quad` | 设定目标为正交点（`90.000°`） | `IDLE` | `target: ok` |
| `target min` | 设定目标为暗谷点（`0.000°`） | `IDLE` | `target: ok` |
| `target max` | 设定目标为峰点（`180.000°`） | `IDLE` | `target: ok` |
| `target custom <deg>` | 设定任意相位目标（十进制度数） | `IDLE` | `target: ok` |
| `start` | 启动 `64 kSPS` 采集与 `5 Hz` 闭环锁定 | `IDLE` 且 `cal=1` | `start: ok` |
| `stop` | 停止闭环与实时采集，并将 `VA` 归零 | `SCAN/CAL/RUNNING` | `stop: ok` |
| `save` | 将当前校准记录写入 `U21`（字节 `160..215`） | `IDLE` 且 `cal=1` | `save: ok` |
| `load` | 从 `U21` 重新读取并校验描述符及校准记录 | `IDLE` | `load: ok` |
| `reinit` | 重新初始化模拟子板、`rt_pipeline` 与故障状态 | `IDLE` 或 `FAULT` | `reinit: ok` / `uncalibrated` |
| `eeprom init` | 写入 `20261110-MZM` 默认板卡描述符至 `U21` | 非运行/非扫描态 | `eeprom init: ok crc=0x...` |

### 5.2 `status` 输出字段解释
```text
state=RUNNING init=1 target=QUAD(90.000 deg) cal=1 locked=1 hold_assist=1
bias=-340 mV scan_vpi=0 mV vpi=5382 mV null=-3029 mV updates=46 pass=0 step=0
harmonics h1=355.394 mV@-2.287 rad h2=0.311 mV@2.447 rad dc=418.561 mV
drops=0 crc_errs=0 fault=0 cleanup_err=0 load_err=0 rt_fault=0
```
- `state`：`IDLE`（空闲）/ `SCAN`（仅测 Vpi）/ `CAL`（两遍校准）/ `RUNNING`（闭环锁定）/ `FAULT`（已触发安全切断）。
- `locked=1`：同时满足 `bias_ok`、`observer_ok`、`radius_ok`、`error_ok`、`phase_ok` 五项锁定判据；连续 25 次（`5 s`）满足后 `hold_assist=1`。
- `harmonics`：`h1`/`h2` 为 `ADS131M02 CH0`（交流通道）`1 kHz`/`2 kHz` 幅值与相位；`dc` 为 `CH1` 监测通道直流均值（仅用于监测，不参与相位闭环反馈）。
- `drops` / `crc_errs` / `fault` / `rt_fault`：正常运行时四项必须持续为 `0`。

---

## 6. OLED 显示、`LED1` 与常见故障排查

### 6.1 `OLED1`（`0x3C`）与 `LED1`（`PC13`）
- **屏幕第 1 行**：固定显示 `BIASCTRL H503 P4`。
- **屏幕第 2–4 行**：
  - 正常/校准中：`ST:<IDLE|SCAN|CAL|RUN|FLT> CAL:<0|1> LCK:<0|1>`、`TGT:<QUAD|MIN|MAX|CUST> VA:<mV>`、`VPI:<mV> HA:<0|1>`（未校准时显示 `UNCAL LOAD:<err>`，故障时显示 `FLT:<err> CLN:<err>`）。
  - 初始化失败：`BRD:<rc> CTRL:<load_err>`、`KEYS:<rc> CLI:<rc>`、`RUN EEPROM INIT`。
- **`LED1` 状态**：正常周期翻转 `500 ms`；发生启动故障或进入 `CTRL_STATE_FAULT` 时变为 `150 ms` 快闪。

### 6.2 常见错误码与处置
| 错误码 | 含义 | 常见原因与排查措施 |
| --- | --- | --- |
| `-77` (`-EBADMSG`) | CRC 校验失败或记录未写入 | 1. 开机 `ctrl_thread init: -77`：新板仅有描述符但未校准，运行 `cal bias` 后执行 `save`；2. 若 `eeprom init` 前报错，先执行 `eeprom init`；3. 运行时 `crc_errs > 0`：检查 `FPC2` 排线接触与 `ADS131M02` 供电。 |
| `-33` (`-EDOM`) | 数学域/观测器收敛失败 | 未接通光路、光功率过弱导致观测半径不足（`< 0.10`），或单遍扫压未能识别足够极值点。确认光路信号后执行 `reinit` 重试。 |
| `-16` (`-EBUSY`) | 通道/资源忙 | 控制线程非 `IDLE` 时重复发命令（先执行 `stop`），或实时流水线 SPI 未在周期内完成。 |
| `-116` (`-ETIMEDOUT`) | 通信或异步写入超时 | 检查模拟板 `+5V` / `DAC8568` / `ADS131M02` 供电及 `FPC2` 连接后执行 `reinit`。 |
| `-19` (`-ENODEV`) | 外设未就绪或未初始化 | 检查 `I2C1`（`0x3C/0x50`）上拉及排线连接，通过 `reinit` 恢复。 |
| `-22` (`-EINVAL`) | 命令参数非法 | 检查 `target custom <deg>` 数值格式或命令拼写，运行 `help` 核对语法。 |

进入 `CTRL_STATE_FAULT` 后系统已自动切断导频并将 `VA` 置回 `0 V`，排查物理原因后发送 `reinit` 恢复至 `IDLE`。

---

## 7. 文档变更记录

| 日期 | 版本 | 修改内容 | 对应阶段/任务 |
| --- | --- | --- | --- |
| 2026-11-26 | v1.0 | 新建用户使用手册，涵盖实板飞线基线、编译烧录测试、Phase 4 CLI 全流程、OLED/LED 指示及故障排查 | Phase 4 收尾 / 文档建档 |
