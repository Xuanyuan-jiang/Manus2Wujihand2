#!/usr/bin/env bash
# 恢复 MANUS SDK 运行库到 ros2_ws/src/ManusSDK/lib/。
#
# 这些 .so 是 MANUS 的闭源商业件（使用需带 SDK feature 的 license key），
# 因此不入 git。clone 之后、colcon build 之前先跑一次这个脚本。
#
# 来源目录应当是官方分发包里的 ManusSDK/lib（含 amd64/ 与 aarch64/ 两个子目录），
# 例如 "MANUS_Core_3.2.0_SDK_Linux/ROS2 package/ManusSDK/lib"。
#
# 查找顺序：
#   1. 已就位且校验通过  -> 直接返回
#   2. $MANUS_SDK_LIB    -> 从该目录复制
#   3. 脚本第一个参数    -> 从该目录复制
#   4. 若干默认位置      -> 从中复制
# 都找不到就打印获取说明后退出。

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$REPO_ROOT/ros2_ws/src/ManusSDK/lib"
SUMS="$REPO_ROOT/tools/manus_sdk_lib.sha256"
LIBS=(amd64/libManusSDK-amd64.so aarch64/libManusSDK-aarch64.so)

verify() {
  local dir="$1"
  local lib
  for lib in "${LIBS[@]}"; do
    [[ -f "$dir/$lib" ]] || return 1
  done
  (cd "$dir" && sha256sum --quiet --check "$SUMS") >/dev/null 2>&1
}

if verify "$DEST"; then
  echo "OK: MANUS SDK 已就位且校验通过 -> $DEST"
  exit 0
fi

CANDIDATES=()
[[ -n "${MANUS_SDK_LIB:-}" ]] && CANDIDATES+=("$MANUS_SDK_LIB")
[[ $# -ge 1 ]] && CANDIDATES+=("$1")
CANDIDATES+=(
  "$REPO_ROOT/MANUS_Core_3.2.0_SDK_Linux/ROS2 package/ManusSDK/lib"
  "$HOME/.local/share/manus-sdk/lib"
  "/opt/manus/lib"
)

for src in "${CANDIDATES[@]}"; do
  [[ -d "$src" ]] || continue
  if verify "$src"; then
    for lib in "${LIBS[@]}"; do
      mkdir -p "$DEST/$(dirname "$lib")"
      cp -f "$src/$lib" "$DEST/$lib"
      chmod 644 "$DEST/$lib"
    done
    echo "OK: 已从 $src 恢复 MANUS SDK -> $DEST"
    exit 0
  fi
  echo "跳过 $src（缺文件或校验不符）" >&2
done

cat >&2 <<EOF

ERROR: 找不到可用的 MANUS SDK 运行库。

需要这些文件（sha256 见 tools/manus_sdk_lib.sha256）：
$(printf '  %s\n' "${LIBS[@]}")

获取方式：从 MANUS 官方分发包中取出 "ROS2 package/ManusSDK/lib" 整个目录
（当前对应版本 MANUS Core 3.2.0 SDK for Linux）。

然后指定来源目录重跑：
  MANUS_SDK_LIB=/path/to/ManusSDK/lib $0
  # 或
  $0 /path/to/ManusSDK/lib

EOF
exit 1
