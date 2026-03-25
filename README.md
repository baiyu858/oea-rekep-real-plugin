# OEA ReKep Real Plugin

English | [中文](README_zh.md)

`oea-rekep-real-plugin` is an external HAL plugin for **OpenEmbodiedAgent (OEA)**. It adds real-world ReKep execution capability to OEA through the `rekep_real` driver, while keeping the OEA core repository clean and lightweight.

This repository includes:

- the `rekep_real` HAL driver
- the migrated real-world ReKep runtime
- the minimum required third-party Dobot/XTrainer runtime code
- extension points for adapting additional robot families

## What This Plugin Provides

- `rekep_real` driver for `hal/hal_watchdog.py`
- real-world bridge entrypoints such as `preflight`, `execute`, `execute_background`, and `longrun_*`
- support for Dobot/XTrainer runtime variants such as `xtrainer_zmq`, `xtrainer_sdk`, and `mock`
- pluggable robot adaptation flow for new robot families
- a plugin manifest that allows OEA to install and register this repository dynamically

## Requirements

- OEA installed from the main `OpenEmbodiedAgent` repository
- Python `>=3.11`
- a dedicated Python or conda environment is recommended for the ReKep runtime
- hardware-side dependencies as needed, such as RealSense and robot communication services

## Install Into OEA

From the OEA repository root:

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url https://github.com/baiyu858/oea-rekep-real-plugin.git
```

To install optional solver dependencies as well:

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url https://github.com/baiyu858/oea-rekep-real-plugin.git \
  --with-solver
```

For local development, you can also install from a local checkout:

```bash
python scripts/deploy_rekep_real_plugin.py \
  --repo-url ../oea-rekep-real-plugin
```

After installation, start OEA with the external driver:

```bash
python hal/hal_watchdog.py --driver rekep_real --workspace ~/.OEA/workspace
```

## Local Secret Scanning

Install the development tooling:

```bash
python -m pip install -e ".[dev]"
```

Enable commit-time checks:

```bash
pre-commit install
```

Run the configured hooks on the current working tree:

```bash
pre-commit run --all-files
```

Run a full git-history secret scan:

```bash
./scripts/scan_secrets.sh
```

The repository includes:

- `.pre-commit-config.yaml` for local commit-time protection
- `.gitleaks.toml` for repository-specific gitleaks settings
- `scripts/scan_secrets.sh` for a manual full-history scan

## Quick Start

Run a preflight check:

```bash
python runtime/dobot_bridge.py preflight --pretty
```

Run a dry-run execution:

```bash
python runtime/dobot_bridge.py execute \
  --instruction "pick up the red block and place it on the tray" \
  --pretty
```

Run a real execution:

```bash
python runtime/dobot_bridge.py execute \
  --instruction "pick up the red block and place it on the tray" \
  --execute_motion \
  --pretty
```

## Repository Layout

- `oea_rekep_real_plugin/driver.py`: external HAL driver implementation
- `oea_rekep_real_plugin/profiles/rekep_real.md`: embodiment profile installed by OEA
- `runtime/`: ReKep real-world runtime and bridge scripts
- `runtime/third_party/`: bundled minimum third-party runtime dependencies
- `runtime/docs/robot_adaptation.md`: robot adaptation guide in English
- `runtime/docs/robot_adaptation_zh.md`: robot adaptation guide in Chinese
- `oea_plugin.toml`: plugin manifest consumed by OEA

## Adapting Another Robot

Start here:

- `runtime/cellbot_adapter.py`
- `runtime/robot_factory.py`
- `runtime/docs/robot_adaptation.md`
- `runtime/docs/robot_adaptation_zh.md`

The bridge supports generic robot parameters such as:

- `--robot_family`
- `--robot_driver`
- `--robot_host`
- `--robot_port`
- `--robot_move_port`

## Runtime Notes

Common environment variables:

- `REKEP_PYTHON`
- `REKEP_TOOL_ROOT`
- `REKEP_REAL_STATE_DIR`
- `REKEP_EXECUTION_MODE`
- `REKEP_ROBOT_FAMILY`
- `REKEP_ROBOT_DRIVER`
- `REKEP_ROBOT_HOST`
- `REKEP_ROBOT_PORT`

For detailed runtime usage, see [runtime/README.md](runtime/README.md).

## Third-Party Code and Licenses

This repository bundles only the minimum third-party code required for real-world execution. License files and third-party notices are preserved under `runtime/third_party/`.

If you need a full training or data collection stack, prepare the upstream repositories separately.

## License

This repository is released under the MIT License. See [LICENSE](LICENSE).
