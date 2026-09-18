# 用法: source <仓库根>/env_ros.sh
# 仅用于 colcon build 和系统 ROS 节点；不要在 wuji2 Python 终端中 source。
export WUJI2_ROOT="${WUJI2_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"

# env_ros.sh deliberately removes conda from PATH.  Also remove conda's prompt
# marker so the shell cannot misleadingly display "(wuji2)" while `python`
# actually points nowhere/on the system PATH.
if [ -n "${CONDA_DEFAULT_ENV:-}" ] && [ -n "${PS1:-}" ]; then
  PS1="${PS1#\(${CONDA_DEFAULT_ENV}\) }"
fi

export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE CONDA_PROMPT_MODIFIER
unset CONDA_SHLVL CONDA_EXE _CE_CONDA _CE_M
unset PYTHONHOME PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset LD_LIBRARY_PATH PKG_CONFIG_PATH

source /opt/ros/jazzy/setup.bash
if [ "${WUJI2_NO_OVERLAY:-0}" != 1 ] && [ -f "$WUJI2_ROOT/ros2_ws/install/setup.bash" ]; then
  source "$WUJI2_ROOT/ros2_ws/install/setup.bash"
fi
export ROS_DOMAIN_ID=30

if [ "$(command -v python3)" != /usr/bin/python3 ]; then
  echo "[ros] ERROR: python3 仍不是 /usr/bin/python3" >&2
  return 1 2>/dev/null || exit 1
fi
echo "[ros] python=$(command -v python3) ROS_DISTRO=$ROS_DISTRO DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[ros] conda 已退出 PATH；运行 scripts/01..03 前请重新 source $WUJI2_ROOT/env.sh"
