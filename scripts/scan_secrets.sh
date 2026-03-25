#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if ! command -v gitleaks >/dev/null 2>&1; then
  echo "gitleaks is not installed."
  echo "Install it first, then re-run this script."
  echo "Official repo: https://github.com/gitleaks/gitleaks"
  exit 1
fi

echo "Running full git-history secret scan..."
gitleaks git --config "$ROOT_DIR/.gitleaks.toml" --redact --verbose "$@"
