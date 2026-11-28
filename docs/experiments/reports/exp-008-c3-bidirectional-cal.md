# EXP-008：C3 双向全周期仿射标定、交叉验证与板载 MZM2 持久化验证

- **实验日期**：2026-11-26
- **关联任务**：`openspec/changes/07-affine-thesis-experiments/`（`t5_exp_c3`）
- **硬件与固件条件**：`20261110-底板`（`STM32H503CBT6` @ `250 MHz`）+ `20261110-MZM`（`64 kSPS`、`159.2 mV` 导频、`T_wait = 3` 块 = `60 ms`、每点 `3` 块平均、扫描起点与反向终点预稳定 `2.0 s`），固件 `P7 rev=b4bd711-dirty`
- **外部仪器状态**：`RIGOL DG922 Pro` 射频关闭（`:OUTP1? = 0`）
- **原始日志与归档数据**：`.agent-bridge/exp_runs/c3_live_v2/`；归档文件见 [c3_summary.json](../data/c3/c3_summary.json)、[c3_sessions.csv](../data/c3/c3_sessions.csv)、[c3_session_0.csv](../data/c3/c3_session_0.csv)、[c3_session_1.csv](../data/c3/c3_session_1.csv)、[c3_session_2.csv](../data/c3/c3_session_2.csv)、[c3_cv_comparison.png](../data/c3/c3_cv_comparison.png)、[onboard_mzm2_persistence.json](../data/c3/onboard_mzm2_persistence.json)

## 1. 双向全周期标定与五折交叉验证（3 轮次 × 181 点正反向）

扫描电压覆盖 `[-5.820, +5.140] V`（全周期 $2V_\pi$），同步采集 `(H1, H2)` 复相量与 `CH1` 直流监测值。在初始无预稳定测试（`.agent-bridge/exp_runs/c3_live/`）中定位到两点机理与软件问题并完成修正后获得本组正式数据：
1. 在双向合并数组 `fwd + rev` 中，铌酸锂调制器正反向迟滞（`~0.044 rad`）大于单步电压增量（`~0.035 rad`），导致旧版 `mergesort` 相邻点靴带面积判定在反向与前向交错点倒退从而使合并模型手性翻转；已在 `scripts/experiments/algorithms.py` 中改用按唯一偏压聚合排序并通过唯一电压相位单调性 $\mathrm{d}\hat{\phi}/\mathrm{d}V > 0$ 强制锁定手性。
2. 偏压大幅跳变至扫频起点 `-5.820 V` 后存在约 `1 s` 的铌酸锂介质弛豫，`run_c3.py` 在每程起点增加 `2.0 s`（`100` 块）预稳定等待，消除边界点弛豫伪差。

### 1.1 各轮次汇总与五折分块交叉验证

| 轮次 | 拟合内误差中位数 `(mrad)` | `CH1 DC` vs `cos(φ_aff)` RMSE `(mV)` | 条件数 `κ(A_cal)` | `σ_min (mV)` | `Vpi (V)` | `null_thesis (V)` | `null_fw (V)` | `512`-code INL 杂散 `(mrad)` |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Session 1 | `9.242` | `0.800` | `50.776` | `8.588` | `5.4124` | `+2.1908` | `-3.2216` | `0.409` |
| Session 2 | `8.614` | `0.778` | `50.774` | `8.589` | `5.4085` | `+2.2048` | `-3.2037` | `0.356` |
| Session 3 | `8.385` | `0.770` | `50.773` | `8.588` | `5.4084` | `+2.2134` | `-3.1950` | `0.620` |
| 三轮均值/中位 | `8.614`（中位） | `0.782` | `50.774` | `8.588` | `5.4098` | `+2.2030` | `-3.2067` | `0.461` |

| 轮次 | 有标签 2D 仿射 CV RMS `(mrad)` | 椭圆+DC 无监督仿射 CV RMS `(mrad)` | 对角 2D 基线 CV RMS `(mrad)` | 无监督较对角降幅 |
| --- | --- | --- | --- | --- |
| Session 1 | `7.838` | `9.802` | `15.679` | `-37.5%` |
| Session 2 | `6.854` | `8.772` | `14.638` | `-40.1%` |
| Session 3 | `6.867` | `8.622` | `14.953` | `-42.3%` |
| 三轮均值 | `7.186` (`0.412°`) | `9.065` (`0.519°`) | `15.090` (`0.865°`) | `-39.9%` |

## 2. 双向零点迟滞、参考重启一致性与 `512`-Code INL 核查

| 检验项目 | Session 1 | Session 2 | Session 3 | 物理说明 |
| --- | --- | --- | --- | --- |
| 正向 vs 反向零点差 `(mV / °)` | `85.07 / 2.829°` | `89.02 / 2.963°` | `88.84 / 2.957°` | 铌酸锂电极扫频真实电光迟滞（三轮均值 `87.65 mV` / `2.92°`） |
| `10` 次参考点重启 RMS / Max `(mrad)` | `8.965 / 20.435` | `8.699 / 20.927` | `8.898 / 19.718` | 含约 `10 s` 串口间隔内铌酸锂慢漂 |
| DAC `512`-code 周期 INL 相关杂散 `(mrad)` | `0.409` | `0.356` | `0.620` | `< 0.04°`，无显著周期性非线性畸变 |

## 3. 跨轮次冻结重测与多通道一致性

冻结 Session 1 标定参数对后续 Session 2 与 Session 3 前向扫频样本直接重构：

| 跨轮次冻结检验指标 | 实测值 | 判定基准 |
| --- | --- | --- |
| 相对单向线性电压 `φ_ref` 冻结 RMS / P95 | `8.471 mrad` / `0.809°` | `P95 <= 1.0°` 通过 |
| 纯仿射参数跨轮次预测差异 RMS / P95（$\hat{\phi}_{S1}(z) - \hat{\phi}_{Sk}(z)$） | `0.419 mrad` (`0.024°`) / `0.061°` | 核心解算矩阵高度时不变 |
| 冻结 S1 谐波预测相位重构独立 `CH1 DC` 残差 RMSE | `0.817 mV` | `CH1` 满幅约 `783 mV`，一致性优于 `0.11%` |

## 4. 板载固件 `MZM2` 校准与掉电冷启动重载验证

执行板载 `cal bias -> save -> pyocd reset -> hw info / status`，并与 C3 离线 Session 3 双向结果对比：

| 项目 | 实测结果 |
| --- | --- |
| 板载 `MZM2` 校准输出 | `vpi_mv = 5378 mV`，`null_mv_fw = -3145 mV` |
| `U21` 描述符与校准 CRC | `desc_crc = 0x61DC009F`，`cal_crc = 0xEB285013`，`cal_status = 0` |
| `pyocd reset` 冷启动重加载 | `ctrl_thread init: 0`，`load_err = 0`，`drops = 0`，`crc_errs = 0`，重载参数完全一致（`5378 / -3145 mV`） |
| 与 C3 Session 3 离线双向标定偏差 | `ΔVpi = -30.39 mV`（`-0.56%`），`Δnull = +50.03 mV`（位于双向 `87.65 mV` 铌酸锂迟滞带内，源于固件 `-9.8 ~ +9.8 V` 单向扫描跨度差异） |

## 5. 结论

- 修复合并扫频手性判定与起点 `2.0 s` 预稳定后，无标签 Fitzgibbon+DC 仿射模型五折 CV RMS 均值达 `9.065 mrad`（较对角基线降低 `39.9%`），跨轮冻结重测 `P95 = 0.809° <= 1.0°`，纯仿射映射模型跨轮稳定性 `P95 = 0.061°`，板载 `MZM2` 掉电重载零差错通过。
- 门禁验证：`python3 -m scripts.experiments.analyze_and_plot --exp c3 --run-dir .agent-bridge/exp_runs/c3_live_v2 --out-dir docs/experiments/data/c3`、`python3 -m unittest discover -s tests/experiments -v` 与 `python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync` 均退出 `0`。
