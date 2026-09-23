# 用法: source <仓库根>/env_ros.sh      （bash 或 zsh）
# 仅用于 colcon build 和系统 ROS 节点；不要在 wuji2 Python 终端中 source。
if [ -n "${ZSH_VERSION:-}" ]; then
  eval '_wuji2_src=${(%):-%x}'; _wuji2_sh=zsh
else
  _wuji2_src=${BASH_SOURCE[0]}; _wuji2_sh=bash
fi
export WUJI2_ROOT="${WUJI2_ROOT:-$(cd "$(dirname "$_wuji2_src")" && pwd)}"

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
# 屏蔽 ~/.local 下 pip --user 装的包：它们会顶掉系统/conda 的同名包
# （Ubuntu 22.04 上常见的是新版 setuptools 让 colcon 编 rosidl 包时报 canonicalize_version）。
export PYTHONNOUSERSITE=1

# ROS 发行版：Ubuntu 24.04 → jazzy，22.04 → humble。可用 WUJI2_ROS_DISTRO 强制指定。
if [ -z "${WUJI2_ROS_DISTRO:-}" ]; then
  for _d in jazzy humble; do
    if [ -f "/opt/ros/$_d/setup.bash" ]; then WUJI2_ROS_DISTRO=$_d; break; fi
  done
  unset _d
fi
if [ ! -f "/opt/ros/${WUJI2_ROS_DISTRO:-none}/setup.bash" ]; then
  echo "[ros] ERROR: 找不到 /opt/ros/{jazzy,humble}/setup.bash，先装 ROS 2" >&2
  return 1 2>/dev/null || exit 1
fi
export WUJI2_ROS_DISTRO
# ROS 的 setup.bash 在 zsh 里会找错路径，按当前 shell 选 setup.bash / setup.zsh。
source "/opt/ros/$WUJI2_ROS_DISTRO/setup.$_wuji2_sh"
if [ "${WUJI2_NO_OVERLAY:-0}" != 1 ] && [ -f "$WUJI2_ROOT/ros2_ws/install/setup.$_wuji2_sh" ]; then
  source "$WUJI2_ROOT/ros2_ws/install/setup.$_wuji2_sh"
fi
unset _wuji2_src _wuji2_sh
export ROS_DOMAIN_ID=30

if [ "$(command -v python3)" != /usr/bin/python3 ]; then
  echo "[ros] ERROR: python3 仍不是 /usr/bin/python3" >&2
  return 1 2>/dev/null || exit 1
fi
echo "[ros] python=$(command -v python3) ROS_DISTRO=$ROS_DISTRO DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[ros] conda 已退出 PATH；运行 scripts/01..03 前请重新 source $WUJI2_ROOT/env.sh"
