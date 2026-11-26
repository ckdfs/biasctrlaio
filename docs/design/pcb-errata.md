# PCB 勘误与返修记录

本表适用于 20261110 数字底板及 MZM 子板原型。测点坐标和原焊盘网络见[电气设计](electrical.md)，机械配合尺寸见[机械与外壳设计](mechanical.md)，闭环复测条件见 [EXP-003](../experiments/bringup/phase4-closed-loop-verify.md)。

| 编号 | 位置与原因 | 原型处理与状态 | 验证依据或待检项 |
| --- | --- | --- | --- |
| ERR-01 | MZM 子板 U7（TPS7A2050PDQNR，X2SON-4）所在 +5V 支路需旁路。反馈电阻 R9 位于底板 U3（LMR51430），BOM 原值 88.7 kΩ。 | 已拆除子板 U7 并飞线旁路，底板调整 R9，使 VSYS 直供 +5V。 | 记录值 5.339 V，DAC8568 供电上限 5.5 V。U7 封装及焊盘的失效原因待核查。 |
| ERR-02 | MZM 子板四颗 TS5A23159RSER（U14/U16/U17/U20，UQFN-10）的切换路径已拆除。 | 固定 U10.7 → C48.1（DITHER1 → VA），U10.10 → C56.1（DITHER2 → VC），VB 纯直流。八个开关 GPIO 保持低速、低电平。 | EXP-003 验证 VA 单路闭环。三臂双导频验证列入 EXP-006。 |
| ERR-03 | MZM 子板 VF 通道 R56 实装 1 kΩ，BOM 为 30 kΩ。 | 待返修。当前 CN1 使用 VA/VB/VC，VF 闲置。 | 启用 VF 前核对阻值并复测偏压增益和零点。 |
| ERR-04 | 底板 BOM 为 STM32H523CET6（272 KiB SRAM / 512 KiB Flash），原型实装 STM32H503CBT6（32 KiB / 128 KiB）。 | 固件以 H503 为目标，采用常数内存校准，限制静态 RAM ≤ 24,576 B、Flash ≤ 110,592 B。 | Phase 4 镜像静态 RAM 15,814 B、Flash 80,728 B。 |
| ERR-05 | OLED1 背面邻近 USB1 外壳焊脚。H1 的 HX PZ1.27-1x3P WT 与机壳 PM1.27 窗口（3.10 mm）需核对。 | OLED 使用 2.5 mm 垫柱和聚酰亚胺胶带，USB1 针尾 ≤ 1.50 mm。H1 配合尺寸待装配核对。 | 合盖前检查针尾高度、绝缘及插头可达性。 |
| ERR-06 | FPC2 快边沿引起排线振铃。64 kSPS 时，原 GPDMA1 TC 中断等待与线程侧 SPI1 写入发生重叠，触发 rt_fault=-16（-EBUSY）。 | DTS 强制 low-speed。SPI1 使用 12.5 MHz，由 DRDY# ISR 独占，轮询 DMA 完成，直流码通过 dac8568_set_dc_code_async() 提交。 | EXP-003 实测 drops=0、crc_errs=0、fault=0。 |

## 下版核对

- [ ] ERR-01 核对 U7 封装、散热焊盘、反馈 BOM 和 +5V 容差。
- [ ] ERR-02 将两个导频固定连接纳入原理图及通断测试。
- [ ] ERR-03 将 VF 的 R56 纳入贴片阻值与输出增益检验。
- [ ] ERR-04 统一 MCU 采购料号、板级配置及镜像预算。
- [ ] ERR-05 核对 OLED 垫高、USB 焊脚和 H1 窗口公差。
- [ ] ERR-06 保留低速边沿、串联端接和 SPI 独占时序，复测连续采样错误计数。
