#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WUJI2_CONDA="${WUJI2_CONDA:-$HOME/miniconda3}"
WUJI2_ENV="${WUJI2_ENV:-wuji2}"
WUJI2_PYTHON="${WUJI2_PYTHON:-$WUJI2_CONDA/envs/$WUJI2_ENV/bin/python}"

if [[ ! -x "$WUJI2_PYTHON" ]]; then
  echo "ERROR: wuji2 Python not found: $WUJI2_PYTHON" >&2
  exit 1
fi
if [[ ! -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]]; then
  echo "ERROR: ROS workspace is not built; follow DEPLOY.md step 5 first" >&2
  exit 1
fi

source "$PROJECT_ROOT/env_ros.sh" >/dev/null

exec "$WUJI2_PYTHON" "$PROJECT_ROOT/scripts/04_manus_wuji2.py" "$@"
