# biasctrlaio 用户使用说明

本手册面向 `biasctrlaio` 光-电一体化分板式电光调制器偏压控制器（`10%` 监测光路 + `20261110-MZM` 模拟子板 + `STM32H503CBT6` 数字控制底板 `20261110-底板`）的光路连接、硬件接线、固件编译烧录、串口校准操作及故障排查。

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

光路预算与台架见[光学设计](design/optical.md)，板间引脚与电源测试点见[电气设计](design/electrical.md)，机械装配与外壳步骤见[机械与外壳设计](design/mechanical.md)，返修位置见[PCB 勘误](design/pcb-errata.md)。

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
| `H1` (`PD IN`, `- + G`) | 监测光路 `InGaAs PIN PD -> U12A` 跨阻输入 + `R16` 直流监测 `CH1` | `1550 nm` 激光器 -> 手动偏振控制器（PC）-> MZM/DPMZM -> `1:9` 光耦合器（`90%` 主业务输出、`10%` 监测支路）-> `InGaAs PIN PD` 接入前面板 `H1` |
| 底板 USB（`U19 STM32F042`） | DAPLink 调试串口（`USART1`）与 SWD 烧录口 | 串口波特率 **`115200 bps, 8N1`**，无硬件流控；macOS 常见设备名为 `/dev/cu.usbmodem*` |

- **光路接入检查要点（详参[光学设计](design/optical.md)）**：
  - 接入前先通过手动 PC 对准调制器入射偏振态，防止消光比劣化或出现虚假极值。
  - 监测支路（`10%` 分束端）名义平均光功率为 `-9 dBm`（`125.89 µW`），峰值须控制在 `≤ 344.4 µW`（`I_max ≤ 310 µA`，对应 `CH1 ≤ 1.023 V`），低于 `ADS131M02 ±1.2 V` 满量程上限（预留安全裕量防削波与 `D1` 钳位触发）。

---

## 3. 编译环境、固件烧录与自动化测试

首次拉取仓库后按系统类型执行一次初始化（详参 [README.md](../README.md)，脚本会自动配置 `git config core.hooksPath .githooks` 以启用本地提交前文档同步门禁）：
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

# Zephyr 集成测试（QEMU cortex-m3，5 项，自动查找工具链并判定结果）
python3 scripts/run_qemu_ztest.py

# 仓库卫生与严格文档同步检查
python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync
```
> 注：若直接手工执行 `west build -b qemu_cortex_m3 -d build/ztest_p4 tests/ztest -t run`，QEMU 执行完 `PROJECT EXECUTION SUCCESSFUL` 后不会主动退出，交互终端中需按 `Ctrl+a` 再按 `x` 退出。

---

## 4. 首次启用与单 MZM 闭环锁定操作流程

通过串口终端（`115200 8N1`，以 `\r` 或 `\n` 结尾）连接后，应看到提示符 `Phase 7 CLI ready. Type 'help'.` 与 `> `。

### 4.1 首次初始化子板 EEPROM（新模拟板或更换参数时执行）
```text
> eeprom init
eeprom init: ok crc=0x1E71EE41
```
向 `U21 (P24C02A)` `0..147 B` 写入与 `20261110-MZM` 原理图匹配的默认板卡描述符（`±10 V` 量程、`47 kΩ` TIA）。

### 4.2 双遍扫压校准并保存（连接完整光路后执行）
1. 确认激光器已开启、手动 PC 偏振已对准，且 `10%` 监测支路已接入 `H1 (PD IN)`；通过 `status` 确认 `VA` 处于空闲 `0 mV` 且监测 `dc` 位于线性未削波区间（`< 1.02 V`）。
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
| `ctrl cfg` | 打印当前闭环参数配置 | 任意 | `pilot=... mV wait=...` |
| `ctrl cfg <param> <val>` | 配置 `pilot`(mV)/`wait`(0..25)/`avg`(1..16)/`gain`/`kp`/`ki`/`recal`(0\|1)/`rho_thresh`/`hold_ticks` | `IDLE` | `ctrl cfg: ok` |
| `start` | 启动 `64 kSPS` 采集与配置周期的闭环锁定（默认 `159.2 mV` 导频、`T_wait=3` 块） | `IDLE` 且 `cal=1` | `start: ok` |
| `stop` | 停止闭环与实时采集，并将 `VA` 归零 | `SCAN/CAL/RUNNING` | `stop: ok` |
| `save` | 将当前 `MZM2` 校准记录写入 `U21`（字节 `160..223`） | `IDLE` 且 `cal=1` | `save: ok` |
| `load` | 从 `U21` 重新读取并校验描述符及校准记录 | `IDLE` | `load: ok` |
| `reinit` | 重新初始化模拟子板、`rt_pipeline` 与故障状态 | `IDLE` 或 `FAULT` | `reinit: ok` / `uncalibrated` |
| `eeprom init` | 写入 `20261110-MZM` 默认板卡描述符至 `U21` | 非运行/非扫描态 | `eeprom init: ok crc=0x...` |

### 5.2 `status` 输出字段解释
```text
state=RUNNING init=1 target=QUAD(90.000 deg) cal=1 locked=1 hold_assist=1 frozen=0 recal=0
bias=-340 mV scan_vpi=0 mV vpi=5382 mV null=-3029 mV updates=46 pass=0 step=0
harmonics h1=355.394 mV@-2.287 rad h2=0.311 mV@2.447 rad dc=418.561 mV
drops=0 crc_errs=0 fault=0 cleanup_err=0 load_err=0 rt_fault=0
```
- `state`：`IDLE`（空闲）/ `SCAN`（仅测 Vpi）/ `CAL`（两遍校准）/ `RUNNING`（闭环锁定）/ `EXP_HOLD`（实验保持态）/ `FAULT`（已触发安全切断）。
- `locked=1`：同时满足 `observer_ok`、`radius_ok`（`r >= 0.15`）、`error_ok`、`phase_ok` 四项锁定判据；连续 25 次满足后 `hold_assist=1`。热漂移不再受 `0.30*Vpi` 偏置窗误判限制。
- `frozen=1`：当回拉半径 `r < 0.15`（接近 $J_0(m_{\mathrm{RF}})=0$ 零点或光强跌落）时冻结偏压与积分器，保持输出不进 `FAULT`；`recal` 记录自动重校准次数。
- `harmonics`：`h1`/`h2` 为 `ADS131M02 CH0`（交流通道）`1 kHz`/`2 kHz` 幅值与相位；`dc` 为 `CH1` 监测通道直流均值（仅用于校准回归与监测，不直接充当闭环反馈）。
- `drops` / `crc_errs` / `fault` / `rt_fault`：正常运行时四项必须持续为 `0`。

---

## 6. OLED 显示、`LED1` 与常见故障排查

### 6.1 `OLED1`（`0x3C`）与 `LED1`（`PC13`）
- **屏幕第 1 行**：固定显示 `BIASCTRL H503 P7`。
- **屏幕第 2–4 行**：
  - 正常/校准中：`ST:<IDLE|SCAN|CAL|RUN|FLT> CAL:<0|1> LCK:<0|1>`、`TGT:<QUAD|MIN|MAX|CUST> VA:<mV>`、`VPI:<mV> HA:<0|1>`（未校准时显示 `UNCAL LOAD:<err>`，故障时显示 `FLT:<err> CLN:<err>`）。
  - 初始化失败：`BRD:<rc> CTRL:<load_err>`、`KEYS:<rc> CLI:<rc>`、`RUN EEPROM INIT`。
- **`LED1` 状态**：正常周期翻转 `500 ms`；发生启动故障或进入 `CTRL_STATE_FAULT` 时变为 `150 ms` 快闪。

### 6.2 常见错误码与处置
| 错误码 | 含义 | 常见原因与排查措施 |
| --- | --- | --- |
| `-77` (`-EBADMSG`) | CRC 校验失败或记录未写入 | 1. 开机 `ctrl_thread init: -77`：新板仅有描述符但未校准，运行 `cal bias` 后执行 `save`；2. 若 `eeprom init` 前报错，先执行 `eeprom init`；3. 运行时 `crc_errs > 0`：检查 `FPC2` 排线接触与 `ADS131M02` 供电。 |
| `-33` (`-EDOM`) | 数学域/观测器收敛失败 | 未接通光路、光功率过弱导致观测半径不足（`< 0.10`），偏振未对准（PC 失调导致消光比退化），或光功率过高触发 `CH1` 削波 / `D1` 钳位导致单遍扫压未能识别极值点。对照[光学设计](design/optical.md)核对监测光功率与 `status` 中 `dc` 读数后执行 `reinit` 重试。 |
| `-16` (`-EBUSY`) | 通道/资源忙 | 控制线程非 `IDLE` 时重复发命令（先执行 `stop`），或实时流水线 SPI 未在周期内完成。 |
| `-116` (`-ETIMEDOUT`) | 通信或异步写入超时 | 检查模拟板 `+5V` / `DAC8568` / `ADS131M02` 供电及 `FPC2` 连接后执行 `reinit`。 |
| `-19` (`-ENODEV`) | 外设未就绪或未初始化 | 检查 `I2C1`（`0x3C/0x50`）上拉及排线连接，通过 `reinit` 恢复。 |
| `-22` (`-EINVAL`) | 命令参数非法 | 检查 `target custom <deg>` 数值格式或命令拼写，运行 `help` 核对语法。 |

进入 `CTRL_STATE_FAULT` 后系统已自动切断导频并将 `VA` 置回 `0 V`，排查物理原因后发送 `reinit` 恢复至 `IDLE`。

---

## 7. 实验串口采集

以 `115200 8N1` 发送一条命令并等待同名的 `ok` 或 `err=<errno>` 结束行。以下三项诊断从空闲态执行。

| 命令 | 返回值 |
| --- | --- |
| `hw info` | 系统时钟、实际 ADC 寄存器、U21 描述符与校准记录 CRC、校准校验结果及固件版本。H503 对应 250000000 Hz、ID 高字节 0x22、MODE 0x1111、CLOCK 0x0322。新板缺少有效校准时，仍输出已读寄存器和 CRC，结束行报告错误。 |
| `adc stat 100000` | 允许 1000–1000000 个周期。导频归零后读取 n+1 个 DRDY 回调，以 n 个周期的总时长计算采样率。ADC_STAT 同时给出最短和最长周期（内核周期数）、丢块和 CRC 错误计数。 |
| `adc dump 16384` | 允许 1–16384 个 CH0 原始有符号计数。每个 DUMP_SEG 给出从 0 起的段号、实际样本数和首样本的 32 位周期时间戳。DUMP_DATA 每行最多 16 点。每段至多 1280 点，最后一段可较短；FFT 按段计算，满段的频率分辨率为 50 Hz。 |

`meas <bias_mv> <pilot_mv> <wait_blocks> <n_blocks>` 在块边界更新 VA 与导频，丢弃 0–250 个完整块后返回 1–2000 个连续块。VA 范围取自 U21 描述符，导频范围为 0–250 mV 峰值，支持小数。例如 `meas -3029 159.2 7 3` 在提交后等待 140 ms，再输出三个 20 ms 块。159.2 mV 使用标称 0.10 V/V 注入系数，对应 DAC 端 1.592 V 峰值。

`step <v0_mv> <v1_mv> <pilot_mv> <n_blocks>` 保持 v0 共 15 个完整块（300 ms），在块边界提交 v1，立即输出随后 1–200 个块。两个电压均须在描述符范围内。两种命令结束后均进入或保持 `EXP_HOLD`，继续采样并维持最后的 VA 与导频。下一条测量可直接改值。执行 `stop` 结束采样并将 VA 恢复至 0 V。

```text
MEAS,seq,t_ms,bias_mv,code,pilot_mv,h1_i,h1_q,h2_i,h2_q,dc_mv,ch0_min_mv,ch0_max_mv,ch1_min_mv,ch1_max_mv,drops,crc
STEP,seq,t_ms,bias_mv,code,pilot_mv,h1_i,h1_q,h2_i,h2_q,dc_mv,ch0_min_mv,ch0_max_mv,ch1_min_mv,ch1_max_mv,drops,crc
TEL,k,t_ms,bias_mv,phi_hat_rad,err_rad,radius,h1_i,h1_q,h2_i,h2_q,dc_mv,locked,frozen,recal_count,drops,crc
```

以上为字段顺序，设备直接发送数据行。`seq` 从本次启动采样开始计数，`t_ms` 是块末的 32 位开机毫秒数。四个 I/Q 字段均为有符号整数 µV 峰值；`dc_mv` 是 CH1 均值，四个极值字段以 0.1 mV 输出。`code` 是实际提交的 VA DAC 码。每条数据行含 CRLF 最多 135 字节，在 115200 8N1 下占线最多 11.72 ms。

`telemetry on` 启用闭环每次更新的 TEL 行，`telemetry off` 关闭。导出时终端暂停普通提示符和输入回显。`bias_mv` 是本观测窗口的偏压，I/Q、DC 取该控制窗口均值。`phi_hat_rad` 是固件暗零点坐标中的连续相位，`err_rad` 是目标减观测相位的圆周差。`frozen` 指示 `r < 0.15` 偏压冻结保护状态，`recal_count` 统计 `recal=1` 触发的自动重校准成功次数。

## 8. 文档变更记录

| 日期 | 版本 | 修改内容 | 对应阶段/任务 |
| --- | --- | --- | --- |
| 2026-11-26 | v1.0 | 新建用户使用手册，涵盖实板飞线基线、编译烧录测试、Phase 4 CLI 全流程、OLED/LED 指示及故障排查 | Phase 4 收尾 / 文档建档 |
| 2026-11-26 | v1.1 | 补齐光路分束与监测功率核查要点、`H1 (PD IN)` 接线、`scripts/run_qemu_ztest.py` 测试入口及 `-33` 光饱和/偏振排查项 | 文档光-电一致性核查 |
| 2026-11-26 | v1.2 | 在初始化与测试章节补充 `.githooks/pre-commit` 自动激活与 `--working-tree --strict-doc-sync` 文档同步检查说明 | Agent 文档同步规范化 |
| 2026-11-26 | v1.3 | 增加实验诊断、同步测量、阶跃和闭环遥测的命令、范围、字段单位及保持态退出方法。 | Phase 7 / t1_fw_exp_cli |
| 2026-11-26 | v1.4 | 新增 `ctrl cfg` 命令与 `RUNNING` 态 `T_wait` 丢块、冻结保护、自动重校准及 `MZM2` 存储说明。 | Phase 7 / t2_fw_thesis_affine |
