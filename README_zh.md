# OEA ReKep 真机插件

[English](README.md) | 中文

`oea-rekep-real-plugin` 是 **OpenEmbodiedAgent (OEA)** 的外部 HAL 插件仓库，用于为 OEA 提供 `rekep_real` 真机执行能力，同时保持 OEA 主仓库本身简洁、干净、易维护。

本仓库包含：

- `rekep_real` HAL 驱动
- ReKep 真机运行时
- 真机执行所需的 Dobot/XTrainer 最小第三方代码
- 新机器人适配入口与文档

## 插件能力

- 为 `hal/hal_watchdog.py` 提供 `rekep_real` 驱动
- 提供 `preflight`、`execute`、`execute_background`、`longrun_*` 等真机 bridge 能力
- 支持 `xtrainer_zmq`、`xtrainer_sdk`、`mock` 等运行形态
- 支持扩展新的机器人 family
- 提供 `oea_plugin.toml`，供 OEA 动态安装和注册

## 前置要求

- 已安装 OEA 主仓库 `OpenEmbodiedAgent`
- Python `>=3.11`
- 建议为 ReKep 运行时单独准备 Python 或 conda 环境
- 根据实际硬件准备 RealSense、机器人通讯服务等运行条件

## 接入 OEA

进入 OEA 主仓库后执行：

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url https://github.com/baiyu858/oea-rekep-real-plugin.git
```

如果还要安装 solver 额外依赖：

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url https://github.com/baiyu858/oea-rekep-real-plugin.git \
  --with-solver
```

本地联调时，也可以直接从本地目录安装：

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url ../oea-rekep-real-plugin
```

安装完成后，在 OEA 中启动：

```bash
python hal/hal_watchdog.py --driver rekep_real --workspace ~/.OEA/workspace
```

## 本地泄密扫描

先安装开发依赖：

```bash
python -m pip install -e ".[dev]"
```

启用提交前检查：

```bash
pre-commit install
```

对当前工作区执行一次完整 hook 扫描：

```bash
pre-commit run --all-files
```

对整个 git 历史执行一次手动泄密扫描：

```bash
./scripts/scan_secrets.sh
```

仓库里已经包含：

- `.pre-commit-config.yaml`：本地提交前保护
- `.gitleaks.toml`：仓库级 gitleaks 配置
- `scripts/scan_secrets.sh`：整仓历史扫描脚本

## 快速开始

执行预检：

```bash
python runtime/dobot_bridge.py preflight --pretty
```

执行一次 dry-run：

```bash
python runtime/dobot_bridge.py execute \
  --instruction "pick up the red block and place it on the tray" \
  --pretty
```

执行真机动作：

```bash
python runtime/dobot_bridge.py execute \
  --instruction "pick up the red block and place it on the tray" \
  --execute_motion \
  --pretty
```

## 仓库结构

- `oea_rekep_real_plugin/driver.py`：外部 HAL 驱动实现
- `oea_rekep_real_plugin/profiles/rekep_real.md`：由 OEA 安装到工作区的本体档案
- `runtime/`：ReKep 真机运行时与 bridge 脚本
- `runtime/third_party/`：随插件分发的最小第三方运行时代码
- `runtime/docs/robot_adaptation.md`：英文适配文档
- `runtime/docs/robot_adaptation_zh.md`：中文适配文档
- `oea_plugin.toml`：供 OEA 读取的插件清单

## 新机器人适配

建议从这些文件开始：

- `runtime/cellbot_adapter.py`
- `runtime/robot_factory.py`
- `runtime/docs/robot_adaptation.md`
- `runtime/docs/robot_adaptation_zh.md`

bridge 已支持通用机器人参数：

- `--robot_family`
- `--robot_driver`
- `--robot_host`
- `--robot_port`
- `--robot_move_port`

## 运行时说明

常用环境变量包括：

- `REKEP_PYTHON`
- `REKEP_TOOL_ROOT`
- `REKEP_REAL_STATE_DIR`
- `REKEP_EXECUTION_MODE`
- `REKEP_ROBOT_FAMILY`
- `REKEP_ROBOT_DRIVER`
- `REKEP_ROBOT_HOST`
- `REKEP_ROBOT_PORT`

更完整的运行时说明见 [runtime/README.md](runtime/README.md)。

## 第三方代码与许可证

本仓库仅打包真机执行所需的最小第三方代码集合，相关许可证与第三方声明文件保留在 `runtime/third_party/` 下。

如需完整训练栈或数据采集栈，请单独准备对应上游仓库。

## 许可证

本仓库采用 MIT License，详见 [LICENSE](LICENSE)。
