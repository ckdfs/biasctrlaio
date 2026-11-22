# biasctrlaio

基于 Zephyr RTOS 的分板式电光调制器偏压控制器固件工程（`STM32H503CBT6` 数字控制底板 + `20261110-MZM` 模拟子板）。

## 快速初始化环境（克隆后一键配置）

### macOS
```bash
brew bundle
./scripts/bootstrap.sh
source scripts/env.sh
```

### Windows 原生开发（PowerShell，无需 WSL）
1. 一键安装系统依赖（`cmake`、`ninja`、`gperf`、`dtc`、`7z`、`uv`、`git`）并初始化 Zephyr 工作区与最小化 SDK：
   ```powershell
   # 首次使用可自动调用 winget 安装缺失的宿主机工具
   .\scripts\bootstrap.ps1 -InstallDeps

   # 若已安装上述工具，直接初始化工程环境
   .\scripts\bootstrap.ps1
   ```
   > 若通过 `winget` 新装了工具，按提示重开一次 PowerShell 后再次运行 `.\scripts\bootstrap.ps1` 即可。
2. 激活开发环境：
   ```powershell
   . .\scripts\env.ps1
   ```
3. 原生编译、内存预算校验与 USB CMSIS-DAP 烧录：
   ```powershell
   west build -b biasctrl_h503 -d build\h503_bringup app --pristine
   python scripts\check_mem_budget.py --elf build\h503_bringup\zephyr\zephyr.elf
   west flash -d build\h503_bringup
   ```

### Linux / DevContainer
直接使用项目内 `.devcontainer/devcontainer.json`（基于 `ghcr.io/zephyrproject-rtos/ci:latest`），容器创建时自动执行 `./scripts/bootstrap.sh`，之后激活环境即可：
```bash
source scripts/env.sh
```

### 常用验证与协作命令
```bash
# 宿主机单元测试（macOS / Linux）
.venv/bin/pytest

# 宿主机单元测试（Windows PowerShell）
.venv\Scripts\pytest.exe

# 查看 OpenSpec 与 SQLite 任务协同看板
python3 ~/.gemini/config/plugins/codex-mission-control/skills/codex-openspec-orchestrator/scripts/bridge_ctl.py status --project .
```
