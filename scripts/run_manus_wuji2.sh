#!/usr/bin/env bash
# 启动 Hand 2 遥操作控制节点。
#
#   ./scripts/run_manus_wuji2.sh                     右手（默认）
#   ./scripts/run_manus_wuji2.sh --side left         左手
#   ./scripts/run_manus_wuji2.sh --side both         双手，各起一个进程
#   ./scripts/run_manus_wuji2.sh --side both --dry-run
#
# ⚠️ 不加 --dry-run 就会直接连接并驱动实体手。
#
# --side both 刻意起**两个独立进程**，而不是在一个进程里管两只手：
#   - 一只手 fail-safe（关节故障、输入超时）不会带走另一只
#   - SDK 的连接/使能是阻塞调用，同进程会让一只手卡住另一只的控制环
#   - 每只手有各自的 watchdog、限速和失能路径
# 代价是本脚本要自己转发信号并等两个子进程干净退出（它们在 finally 里失能）。

set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WUJI2_CONDA="${WUJI2_CONDA:-$HOME/miniconda3}"
WUJI2_ENV="${WUJI2_ENV:-wuji2}"
WUJI2_PYTHON="${WUJI2_PYTHON:-$WUJI2_CONDA/envs/$WUJI2_ENV/bin/python}"
NODE="$PROJECT_ROOT/scripts/04_manus_wuji2.py"

if [[ ! -x "$WUJI2_PYTHON" ]]; then
  echo "ERROR: wuji2 Python not found: $WUJI2_PYTHON" >&2
  exit 1
fi
if [[ ! -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]]; then
  echo "ERROR: ROS workspace is not built; follow DEPLOY.md step 5 first" >&2
  exit 1
fi

# 先把 --side both 摘出来，其余参数原样透传给 04_manus_wuji2.py。
SIDE=""
PASSTHROUGH=()
while (($#)); do
  case "$1" in
    --side)
      SIDE="${2:-}"
      if [[ -z "$SIDE" ]]; then
        echo "ERROR: --side 需要一个值 (right|left|both)" >&2
        exit 2
      fi
      shift 2
      ;;
    --side=*)
      SIDE="${1#--side=}"
      shift
      ;;
    *)
      PASSTHROUGH+=("$1")
      shift
      ;;
  esac
done
SIDE="${SIDE:-right}"

source "$PROJECT_ROOT/env_ros.sh" >/dev/null

if [[ "$SIDE" != "both" ]]; then
  exec "$WUJI2_PYTHON" "$NODE" --side "$SIDE" "${PASSTHROUGH[@]}"
fi

# ---------- --side both ----------
# 逐手的参数（topic / address）此时无法区分，必须由各进程按 --side 取默认值。
for arg in "${PASSTHROUGH[@]}"; do
  case "$arg" in
    --topic | --topic=* | --address | --address=*)
      echo "ERROR: --side both 不能带 ${arg%%=*}；它对两只手含义不同。" >&2
      echo "       请分别启动：--side right ${arg%%=*} ... / --side left ${arg%%=*} ..." >&2
      exit 2
      ;;
  esac
done

declare -A PIDS=()
CLEANED=0

# 收尾：给还活着的子进程发 SIGINT，等它们在 finally 里失能并断开。
# 幂等 —— INT/TERM 与 EXIT 两条路径都会调它。
cleanup() {
  if ((CLEANED)); then
    return 0
  fi
  CLEANED=1
  if ((${#PIDS[@]} == 0)); then
    return 0
  fi
  local side
  echo "" >&2
  echo "[both] 正在停止剩余控制节点（等待它们失能并断开）…" >&2
  for side in "${!PIDS[@]}"; do
    kill -INT "${PIDS[$side]}" 2>/dev/null || true
  done
  for side in "${!PIDS[@]}"; do
    wait "${PIDS[$side]}" 2>/dev/null || true
  done
  echo "[both] 已全部退出。" >&2
}

# EXIT 也必须收尾，不只是 INT/TERM。脚本一旦因为别的原因退出（errexit、语法
# 错误、被 SIGHUP 带走），还在驱动实体手的子进程就会被 init 收养：那时它已经
# 脱离终端的前台进程组，Ctrl+C 根本打不到它，只能靠 kill 去找 pid。
# 这里踩过一次 —— 循环里的裸 wait 触发 errexit，把左手丢成了孤儿。
trap 'trap - INT TERM EXIT; cleanup; exit 130' INT TERM
trap cleanup EXIT

for side in right left; do
  "$WUJI2_PYTHON" "$NODE" --side "$side" "${PASSTHROUGH[@]}" &
  PIDS[$side]=$!
  echo "[both] $side hand -> pid ${PIDS[$side]}"
done

# 任一进程退出都要报出来，但**不**连坐停掉另一只：
# 遥操作中途一只手 fail-safe 时，把还在正常工作的那只也停掉可能让它松开已抓的东西。
# 需要全停就 Ctrl+C。
FAILED=0
while ((${#PIDS[@]})); do
  if ! wait -n "${PIDS[@]}" 2>/dev/null; then
    FAILED=1
  fi
  for side in "${!PIDS[@]}"; do
    if ! kill -0 "${PIDS[$side]}" 2>/dev/null; then
      # `|| status=$?` 不能省：裸 wait 会把子进程的非零退出码交给 errexit，
      # 脚本当场就死，下面这些消息一句都打不出来。
      status=0
      wait "${PIDS[$side]}" 2>/dev/null || status=$?
      if ((status == 0)); then
        echo "[both] $side hand 已正常退出。" >&2
      else
        echo "[both] !! $side hand 异常退出 (exit=$status) —— 另一只仍在运行。" >&2
        echo "[both]    需要全停请 Ctrl+C。" >&2
        FAILED=1
      fi
      unset 'PIDS[$side]'
    fi
  done
done

exit "$FAILED"
