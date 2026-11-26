# biasctrlaio

电光调制器偏压控制系统（10% 监测光路 + 模拟前端 + 数字控制底板 + Zephyr RTOS 实时固件）。

## 当前项目状态

- **Phase 1–4 已完成并通过真机光学闭环验证**：实现 `64 kSPS` 实时采集与导频输出、两遍 `O(1)` 内存扫压校准、`U21 (P24C02A)` 掉电重载、串口 CLI 控制以及单 MZM `QUAD / MIN / MAX / CUSTOM` 闭环锁定（Flash `73.0%`、RAM `64.3%`，QEMU `5/5 PASS`，Host CTest `13/13 PASS`）。
- **Phase 5 进行中**：适配无开关固定双导频实板（`DITHER1 -> VA`、`DITHER2 -> VC`、`VB` 纯直流），开发单 MZM 运行时无缝切点、3 臂 DPMZM 固定双导频闭环算法与 `OLED1 / WS-001CB` 拨轮本地 UI。

## 工程文档索引

- **用户使用说明**：[docs/user-guide.md](docs/user-guide.md)（硬件接线与飞线基线、编译烧录、串口 CLI 命令、`OLED1/LED1` 说明及故障排查）
- **系统设计文档**：[文档索引](docs/README.md) 与 [系统架构](docs/architecture.md)（分板架构、光路/电气/机械外壳/算法设计、PCB 勘误、`64 kSPS` 实时隔离模型、`U21` 存储布局及 Phase 5 架构演进）
- **[实验台账](docs/experiments/README.md)**：
  - [Phase 1 底板 Bring-Up 验证报告](docs/experiments/bringup/phase1-h503-bringup.md)
  - [Phase 2 外设与分板集成验证报告](docs/experiments/bringup/phase2-peripherals-verify.md)
  - [Phase 4 单 MZM 闭环光学验证报告](docs/experiments/bringup/phase4-closed-loop-verify.md)
- **OpenSpec 规格与活跃规划**：主规格位于 `openspec/specs/`，Phase 5 活跃规划位于 [openspec/changes/05-tdm-multichannel-and-ui/](openspec/changes/05-tdm-multichannel-and-ui/)

## 快速初始化环境（克隆后一键配置，同时启用 `.githooks/pre-commit` 文档门禁）

### macOS
```bash
brew bundle
./scripts/bootstrap.sh
source scripts/env.sh
```

### Windows 原生开发（PowerShell，无需 WSL）
1. 安装宿主机依赖并初始化 Zephyr 工作区与最小化 SDK：
   ```powershell
   .\scripts\bootstrap.ps1 -InstallDeps
   .\scripts\bootstrap.ps1
   ```
2. 激活环境、编译、内存门禁校验与 USB CMSIS-DAP 烧录：
   ```powershell
   . .\scripts\env.ps1
   west build -b biasctrl_h503 -d build\h503_p4 app --pristine
   python scripts\check_mem_budget.py --elf build\h503_p4\zephyr\zephyr.elf
   west flash -d build\h503_p4
   ```

### Linux / DevContainer
使用 `.devcontainer/devcontainer.json`（自动执行 `./scripts/bootstrap.sh`），进入后执行：
```bash
source scripts/env.sh
```

## 常用编译、验证与协作命令

```bash
# 1. 编译 Phase 4 正式固件、检查 Flash/RAM 门禁并烧录（macOS / Linux）
source scripts/env.sh
west build -b biasctrl_h503 -d build/h503_p4 app --pristine
python3 scripts/check_mem_budget.py --elf build/h503_p4/zephyr/zephyr.elf
west flash -d build/h503_p4

# 2. 宿主机 DSP / 控制 / 物理光学仿真测试（13 项）
cmake -B build/host-tests -S tests/host
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure

# 3. Zephyr QEMU 集成测试（5 项，自动识别工具链并检测通过标识）
python3 scripts/run_qemu_ztest.py

# 4. 仓库卫生、严格文档同步检查与矢量图重新编译
python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync
python3 scripts/render_docs_figures.py

# 5. 查看 OpenSpec 与 SQLite 任务协同看板
python3 ~/.gemini/config/plugins/codex-mission-control/skills/codex-openspec-orchestrator/scripts/bridge_ctl.py status --project .
```
