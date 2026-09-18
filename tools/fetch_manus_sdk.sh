#!/usr/bin/env bash
# 恢复 MANUS SDK 运行库到 ros2_ws/src/manus_ros2/ManusSDK/lib/。
#
# 这两个 .so 共约 250 MB，是 MANUS 的闭源商业件，源树内没有随附 license/EULA，
# 因此不入 git。clone 之后、colcon build 之前先跑一次这个脚本。
#
# 查找顺序：
#   1. 已就位且校验通过  -> 直接返回
#   2. $MANUS_SDK_LIB    -> 从该目录复制
#   3. 脚本第一个参数    -> 从该目录复制
#   4. 若干默认位置      -> 从中复制
# 都找不到就打印获取说明后退出。

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$REPO_ROOT/ros2_ws/src/manus_ros2/ManusSDK/lib"
SUMS="$REPO_ROOT/tools/manus_sdk_lib.sha256"
LIBS=(libManusSDK.so libManusSDK_Integrated.so)

verify() {
  local dir="$1"
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
  "$HOME/.local/share/manus-sdk/lib"
  "/opt/manus/lib"
)

for src in "${CANDIDATES[@]}"; do
  [[ -d "$src" ]] || continue
  if verify "$src"; then
    mkdir -p "$DEST"
    for lib in "${LIBS[@]}"; do
      cp -f "$src/$lib" "$DEST/$lib"
      chmod 755 "$DEST/$lib"
    done
    echo "OK: 已从 $src 恢复 MANUS SDK -> $DEST"
    exit 0
  fi
  echo "跳过 $src（缺文件或校验不符）" >&2
done

cat >&2 <<EOF

ERROR: 找不到可用的 MANUS SDK 运行库。

需要这两个文件（sha256 见 tools/manus_sdk_lib.sha256）：
$(printf '  %s\n' "${LIBS[@]}")

获取方式，任选其一：
  1. 从 MANUS 官方 SDK 安装包中取出 lib 目录；
  2. 从另一台已部署好的机器上复制。

然后指定来源目录重跑：
  MANUS_SDK_LIB=/path/to/lib $0
  # 或
  $0 /path/to/lib

EOF
exit 1
