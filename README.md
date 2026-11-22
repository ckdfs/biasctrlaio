# biasctrlaio

基于 Zephyr RTOS 的分板式电光调制器偏压控制器固件工程（`STM32H503CBT6` 数字控制底板 + `20261110-MZM` 模拟子板）。

## 快速初始化环境（克隆后一键配置）

### macOS
```bash
brew bundle
./scripts/bootstrap.sh
source scripts/env.sh
```

### Linux / DevContainer
直接使用项目内 `.devcontainer/devcontainer.json`（基于 `ghcr.io/zephyrproject-rtos/ci:latest`），容器创建时自动执行 `./scripts/bootstrap.sh`，之后激活环境即可：
```bash
source scripts/env.sh
```

### 常用验证命令
```bash
# 宿主机单元测试
.venv/bin/pytest

# 查看 OpenSpec 与 SQLite 任务协同看板
python3 ~/.gemini/config/skills/codex-openspec-orchestrator/scripts/bridge_ctl.py status --project .
```
