# 系统架构

## 光-电一体化闭环系统架构与双轴可扩展原则

本系统是面向电光调制器工作点稳态控制的光-电一体化闭环平台，物理闭环贯穿 **外部光路与调制器、模拟信号调理前端、数字控制底板与实时固件** 四个域：`1550 nm` 激光器经手动偏振控制器（PC）进入 MZM / DPMZM 调制器，输出经 `1:9` 光耦合器分出 `10%` 监测光至 `InGaAs PIN PD`；光电流由前面板 `H1 (PD IN)` 接入 `20261110-MZM` 模拟子板（`U12A` `47 kΩ` 反相 TIA 交流通道 `CH0` 与 `R16` 直流监测通道 `CH1` -> `ADS131M02` `64 kSPS` 同步采样），跨越 `30-pin FPC2` 送入 `STM32H503CBT6` 数字底板执行 `DRDY#` 中断 Goertzel 谐波解调与 `5 Hz` 仿射观测器 / 防饱和 PI 控制；控制量经 `DAC8568 + OPA4192` 与独立交流导频叠加，通过后面板 `CN1 (VA/VB/VC)` 返回调制器电极完成闭环。数字控制底板与可替换模拟子板之间通过两条正交抽象边界实现解耦（遗留单板硬件明确不纳入兼容范围）：

![光-电一体化闭环系统架构与双轴抽象边界](figures/system-architecture.svg)

1. **MCU 轴可扩展性**：主控相关时钟树、引脚复用与 DMA 分配全部封装在 `boards/xidian/<board>/`，更换主控 MCU 时无需改动 `drivers/`、`lib/` 与 `app/`。
2. **模拟子板轴可扩展性**：所有偏压范围、跨阻增益与导频路由表均通过 `analog_board_ops_t` 接口与子板 `U21` EEPROM 描述符动态描述。

### 跨域接口与设计文档映射

| 系统域 | 核心载体与接口边界 | 关键跨域耦合约束 | 设计文档 |
| --- | --- | --- | --- |
| 光路与被控对象 | `1550 nm DFB + PC -> MZM/DPMZM -> 1:9 耦合器 -> PIN PD` | 监测光功率名义 `-9 dBm`、峰值 `≤ 344.4 µW`（`I_max ≤ 310 µA`），保证 `CH1 ≤ 1.023 V` 低于 `ADS131M02 ±1.2 V` 满量程；电极 $V_\pi$ 与导频深度 $m$ 决定弱轴信噪比 | [光学设计](design/optical.md) |
| 模拟前端与数字硬件 | `H1 (PD IN)`、`CN1 (VA/VB/VC)`、`30-pin FPC2` | `U12A` 反相跨阻与 `2 kHz` 模拟/ADC 相移共同决定固件极性 `null_q_sign = -1`；`FPC2` 全引脚强制 `low-speed`；`+5V` 旁路基线 `VSYS = 5.339 V` | [电气设计](design/electrical.md) |
| 机械与外壳接口 | 双层导槽外壳、`H1`/`CN1`/`U18`/`OLED1` 面板开孔 | `H1` 与 `CN1/CN3` 为光-电耦合边界；装配前需按 `FPC KEEP-OUT` 与面板公差核对排线、针尾与开孔 | [机械与外壳设计](design/mechanical.md) |
| 解调与闭环算法 | `64 kSPS` `DRDY#` ISR + `5 Hz` `ctrl_thread` | `N = 1280` 零泄漏 Goertzel、两遍 `O(1)` 仿射标定与单位圆归一化观测器；`CH1` 不入主相位环，仅作直流辅助与监测 | [算法设计](design/algorithm.md) |
| 实板勘误与返修 | `20261110-底板` 与 `20261110-MZM` 原型改造项 | `ERR-01..06` 记录 `U7` 旁路、导频直连飞线、`VF` 待修、H503 替换、配合公差及低速时序 | [PCB 勘误](design/pcb-errata.md) |
| 实验验证台账 | 光路条件 + 固件镜像 + 串口/仪器实测 | 按标准模板同步登记光功率、分束比、消光比、`Vpi`、零点、锁定指标及故障计数 | [实验台账](experiments/README.md) |

---

## 实时调度、中断隔离与故障安全模型

`64 kSPS` 周期仅有 **`15.625 µs`**（`3906` 个 CPU 周期 @ `250 MHz`），软件严格隔离实时中断路径与慢速外设线程：

| 执行上下文 | 职责 | 实时与安全约束 |
| --- | --- | --- |
| **`DRDY#` EXTI + DMA RX 回调（[rt_pipeline.c](../app/src/rt_pipeline.c)）** | 1. 接收 `ADS131M02` `CH0/CH1` 帧并校验 CRC；<br>2. 生成下一步 `DITHER1` 码字，在计算前提交 `SPI1` 传输以掩盖总线延迟；<br>3. 更新 `1 kHz / 2 kHz` Goertzel 与 `CH1` 直流累加器；<br>4. 在 `8 µs` 轮询窗口后执行最多 1 次 `dac8568_set_dc_code_async()`（`5 Hz` `VA` 偏压更新）；<br>5. 每 `1280` 点打包投递 `rt_pipeline_blocks` `k_msgq`（深度 10，`50 Hz`）。 | 运行期间 **`DRDY#` ISR 独占 `SPI1` 与 `SPI2`**，采用无 DMA 中断寄存器轮询与单精度 `goertzel_result_fast()`；严禁调用 `I2C`、`UART`、互斥锁或休眠；任何 CRC 错误、SPI 超时或队列满立即调用 `trip()` 进入 `FAULT`。 |
| **`ctrl_thread`（优先级 `5`，栈 `2560 B`，[ctrl_thread.c](../app/src/ctrl_thread.c)）** | 以 `DSP_CONTROL_DECIMATION = 10` 抽取 `rt_pipeline` 块，组成 `0.2 s`（`5 Hz`）控制周期：驱动两遍 `cal bias` 状态机或 `ctrl_mzm_strategy` PI 闭环更新，并检查序号连续性与幅度一致性。 | 通过 `rt_pipeline_set_dc_code()` 同步握手委托 ISR 发包；出现任何错误即调用 `shutdown()` 切断导频路由、停止采样并把 `VA` 复位到 `0 V`。 |
| **`cli_service`（优先级 `8`，栈 `2048 B`，[cli_service.c](../app/src/cli_service.c)）** | UART 中断收字节写入 `128 B` 环形队列，线程内行解析与 `status` 打印。 | 全异步接收，不阻塞 `DRDY#` 与控制线程。 |
| **`main` 低优先级循环（[main.c](../app/src/main.c)）** | 每 `500 ms` 差分刷新 `OLED1`（仅数据行变化时写 `I2C1`），驱动 `LED1` 正常 500 ms / 故障 150 ms 翻转。 | 通过 `bias_i2c1_mutex` 与 `U21` 读写互斥；与实时流水线无共享阻塞锁。 |

---

## `U21 (P24C02A, 256 B)` EEPROM 存储分区

描述符与记录的序列化定义见 [EEPROM 描述符](../include/bias/eeprom_schema.h) 和 [校准存储服务](../app/src/cal_service.c)。

| 地址范围 | 大小 | 结构体 / 内容 | 校验与绑定规则 |
| --- | :---: | --- | --- |
| `0x00 .. 0x93` | `148 B` | `struct bias_eeprom_descriptor_v1`（magic `0x53414942` `"BIAS"`，version `1`，`board_id="20261110-MZM"`，`tia_gain_ohms=47000`，`6` 通道增益/偏置/`vpi_uv`） | 尾部 `4 B` `CRC-32/ISO-HDLC`，由 `eeprom init` 或校准保存写入 |
| `0x94 .. 0x9F` | `12 B` | 保留页对齐间隔 | — |
| `0xA0 .. 0xDF` | `64 B` | `struct cal_mzm_record_v2`（magic `0x324D5A4D` `"MZM2"`，`descriptor_crc`，`dsp_affine_fit_t fit`，`null_v`，`vpi_v`，`cal_pilot_v`，`h1_ref_phase`，`h2_ref_phase`，`has_ref_phases`，`reserved`） | 尾部 `4 B` `CRC-32/ISO-HDLC`，且 `descriptor_crc` 必须匹配当前描述符 CRC |
| `0xE0 .. 0xFF` | `32 B` | 预留扩展区 | — |

---

## 当前资源占用与实测性能基线（Phase 4 终版）

- **内存与存储（硬预算：Flash `<= 110,592 B`，RAM `<= 24,576 B`）**：
  - `zephyr.elf`：`text=79,576 B, data=1,152 B, bss=14,662 B`
  - **Flash**：**`80,728 / 110,592 B`（73.0%）**，剩余 `29,864 B`
  - **静态 RAM**：**`15,814 / 24,576 B`（64.3%）**，剩余 `8,762 B`
- **自动化验证**：7 个核心 C 文件 `-Wall -Wextra -Werror` `0` 警告；QEMU `ztest` `5/5 PASS`；Host CTest `13/13 PASS`。
- **真机光学闭环实测（[phase4-closed-loop-verify.md](experiments/bringup/phase4-closed-loop-verify.md)）**：
  - `Vpi = 5382 mV`，`null = -3029 mV`；掉电复位重载 `ctrl_thread init: 0`；
  - `QUAD(90)` `dc=418.561 mV`（`53.2%`），`MAX(180)` `dc=787.601 mV`（`100.0%`），`MIN(0)` `dc=-0.992 mV`（`0.0%`），`CUSTOM(135)` `dc=678.135 mV`（`86.1%`），全过程 `drops=0, crc_errs=0, fault=0`。

---

## Phase 5 架构演进规划（适配无开关固定双导频实板）

依据修订后的 [05-tdm-multichannel-and-ui](../openspec/changes/05-tdm-multichannel-and-ui/design.md)，后续演进围绕三部分展开：
1. **`p5_tdm_scheduler`（单 MZM 无缝切换 + 无开关 DPMZM 固定双导频）**：
   - 运行态 `QUAD <-> MIN <-> MAX <-> CUSTOM` 动态切点不中断实时流，按连续相位跟踪重算半周期边界与观测器锚点；
   - 基于固定飞线（`DITHER1 -> VA` 调制子臂 I、`DITHER2 -> VC` 调制子臂 Q、`VB` 纯直流控制主臂、提取 $f_1 \pm f_2$ 互调分量）实现 `ctrl_mod_dpmzm`，在 `64 kSPS` 下采用整周期正交频率组合并满足 `15.625 µs` ISR 时序裕量，通过宿主机 3 臂光学闭环仿真（误差 `< 1.0 deg`）。
2. **`p5_ui_oled_dial`（本地 `OLED1` + `WS-001CB` 拨轮 UI）**：
   - 在最低线程优先级（栈 `<= 1.5 KiB`）实现 `app/src/ui_service.{c,h}`，完成状态监控与免串口切换目标/启停/校准/保存，保持 `drops == 0`。
3. **`p5_full_system_verify`（整机系统联调与资源验收）**：
   - 完成固件集成，确认新增代码后仍满足 `Flash <= 110,592 B` 与 `RAM <= 24,576 B`，通过真机与全量回归验收。

---

## Phase 7 单 MZM 实验采集

在 64 kSPS 下，每 1280 个有效帧构成一个 20 ms 块。DRDY 回调以整数比较更新 CH0、CH1 的极值。块末将极值转为伏特，从原有 Goertzel 的实部、虚部直接得到 H1、H2 的有符号峰值 I/Q，并保留幅相和 CH1 均值。控制线程从同一消息队列取得带序号、块末时间及实际导频幅度的块。

`CTRL_STATE_EXP_HOLD` 与扫描、校准和闭环互斥。实验命令在块末通过现有 SPI1 DMA 提交 VA，下一块从新的导频幅度开始。`step` 的第二次提交由采样序号触发，恰好间隔 15 个块。控制线程检查序号，在等待主机下一条命令期间持续排空队列。UART 格式化和发送全部在线程中完成。停止或故障沿用切断路由、停 ADC、复位 VA 的清理路径。

空闲诊断独占 ADC。`adc stat` 在 DRDY 回调内取 DWT CYCCNT，相邻读数按无符号 32 位求差，累加器为 64 位。测速期间保持内核运行，避免 WFI 暂停 CYCCNT。`adc dump` 复用单个 3840 B 静态缓冲区，以三字节保存每个 CH0 样本。每段停采后才打印，段内连续，段间包含串口输出和重新启动 ADC 所需时间。

### Phase 7 复相量仿射校准与控制环升级

单 MZM 校准与闭环升级为论文第 5 章复相量仿射形式：`cal bias` 在第一遍累加 12 个双精度复协方差统计量提取主轴相位 `(h1_ref_phase, h2_ref_phase)`，并在第二遍执行 `O(1)` 内存 `6×6` Fitzgibbon 椭圆拟合、环绕反射判定与 `CH1` 同步直流回归（`dc_span > 1 mV` 时代数锁定规范旋转与暗谷零点；`dc_span <= 1 mV` 时保留零直流仿真的 `dsp_online_lsq` 回退路径），持久化升级为 `U21` `64 B` `cal_mzm_record_v2`（`"MZM2"`）。`RUNNING` 闭环移除 `spring` 惩罚、`obs_dc` 自减与 `bias_delta <= 0.30` 锁定门限，改用纯圆周误差、`T_wait` 写后丢块（默认 `3` 块 = `60 ms`）、多块平均（默认 `1` 块）、默认 `159.2 mV` 导频、无量纲增益 `G` 换算、`r < 0.15` 冻结保护及圆残差 EMA `rho_ema` 自动重校准（通过 `ctrl cfg` 配置）。

### Phase 7 资源基线与 C1–C6 实板实验归档

- **STM32H503CBT6 资源占用（`build/h503_p7/zephyr/zephyr.elf`）**：`text=94,016 B, data=1,224 B, bss=19,406 B`；**Flash `95,240 / 110,592 B`（`86.1%`）**、**静态 RAM `20,630 / 24,576 B`（`83.9%`）**，全套真机实验全程 `drops=0, crc_errs=0`。
- **C1–C6 实验归档与论文映射**：完整实测汇总、口径核对（`CLOCK=0x0322`、`20 MHz` 射频激励、余弦相位换算）及台架排除项见 [EXP-012 C1–C6 实板总汇与论文表格映射](experiments/reports/exp-012-thesis-c1-c6-summary.md)。

## 文档变更记录

| 日期 | 版本 | 修改内容 | 对应阶段/任务 |
| --- | --- | --- | --- |
| 2026-11-26 | v1.0 | 新建系统设计文档，完整梳理分板架构、硬件信号链与实板改造基线、64 kSPS ISR 隔离、DSP/控制算法、U21 分区、Phase 4 实测基线及 Phase 5 演进规划 | Phase 4 收尾 / Phase 5 规划同步 |
| 2026-11-26 | v1.1 | 按光学、电气、算法及 PCB 勘误拆分设计资料，归档三份验证报告。 | Phase 6 文档重组 |
| 2026-11-26 | v1.2 | 新增外壳与机械装配文档，导入物理/仿真图表，将架构图替换为 LaTeX TikZ 矢量 SVG。 | Phase 6 图形与结构补全 |
| 2026-11-26 | v1.3 | 补齐系统架构中的光路域与光-电一体化闭环视角，更新架构矢量图并同步核查全仓设计文档与实验模板的跨域接口说明。 | 文档光-电一致性核查 |
| 2026-11-26 | v1.4 | 建立五层 Agent 文档同步工作流：升级 `check_repo_hygiene.py` 严格检查，新增 `.agents/hooks.json`（`agent_doc_gate.py`）、`.githooks/pre-commit` 与 CI 严格门禁，同步规范 `AGENTS.md` 与 OpenSpec 规则。 | Agent 文档同步规范化 |
| 2026-11-26 | v1.5 | 增加块级 I/Q、双通道极值、DWT 周期统计、分段原始采集与实验保持态。 | Phase 7 / t1_fw_exp_cli |
| 2026-11-26 | v1.6 | 升级单 MZM 为第 5 章复相量仿射校准与纯圆周控制环，更新 `U21` `MZM2` `64 B` 布局、`T_wait` 丢块、`r < 0.15` 冻结及 `rho_ema` 自动重校准。 | Phase 7 / t2_fw_thesis_affine |
| 2026-11-27 | v1.7 | 完成 C1–C6 实板实验全套采集与数据归档（EXP-007..EXP-012），更新 Phase 7 资源基线与论文第 3/5 章实测对照总汇。 | Phase 7 / t4..t9 |
