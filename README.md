# Manus2Wujihand2

```
MANUS Metagloves Pro ── USB dongle 3325:0049
  → manus_data_publisher   C++，链 libManusSDK（闭源，不入库）
  → /manus_glove_0, _1     ManusGlove.msg，含 raw_nodes 骨架（编号是创建顺序，不是左右）
  → manus_input            按 msg.side 分流，转成 21 点 MediaPipe
  → /hand_input_right      Float32MultiArray，63 floats = 21×3（米），约 120 Hz
    /hand_input_left       两只手各发各的，互不阻塞
  → scripts/04_manus_wuji2.py --side right|left     每只手一个独立进程
        wuji_sdk.RetargetSession → 20 轴限位 clamp → 每轴限速 → watchdog
  → 网线 / Zenoh → Wuji Hand 2
```

| | 序列号 | 地址 |
|---|---|---|
| 右手 | `WH2KA01260817029` | `192.168.1.111:7447` |
| 左手 | `WH2JA01260813009` | `192.168.1.110:7447` |

地址是 `04/05/06` 脚本的默认值，换了手用 `--address HOST:PORT` 覆盖。

> ⚠️ **控制节点默认就会连接、使能并驱动实体手**，只有加 `--dry-run` 才只打印不下发。
> 这两只手的固件软限位是关闭的（`soft_limit_enabled = 0`），越界保护全靠本仓库的软件。
> 换标定、改 `manus_input_py`、升级 SDK 之后，第一次必须先跑 `--dry-run`。

---

## 0. 准备

**硬件**

- MANUS Metagloves Pro + USB dongle（`lsusb` 显示 `3325:0049`）。dongle 的 license 必须带 **SDK（Integrated）** feature，否则 Linux 侧无法使用。
- Wuji Hand 2（一只或两只），网线接到本机的有线网口（两只手经交换机或两个网口）。
- x86_64 主机，Ubuntu 24.04 或 22.04：

| Ubuntu | ROS 2 | `wuji2` 环境的 Python | 说明 |
|---|---|---|---|
| 24.04 | Jazzy | 3.12 | 原部署机的配置，全链路实测 |
| 22.04 | Humble | 3.10 | 已验证编译与 dry-run，实体手未在此组合上跑过 |

`wuji2` 的 Python 小版本必须和 ROS 的 rclpy 一致，因为控制节点在同一个进程里同时 import `rclpy` 和 `wuji_sdk`。

**文件**

- MANUS 官方分发包 **MANUS Core 3.2.0 SDK for Linux**（目录名 `MANUS_Core_3.2.0_SDK_Linux`，约 867 MB，闭源，不入库），从 MANUS 官方获取。

**约定**

- 本文所有命令都在**仓库根目录**执行，bash 和 zsh 都可以。`env.sh` / `env_ros.sh` 两种 shell 都支持，并从自身位置推导仓库根，所以仓库可以放在任意路径。zsh 下粘贴带 `#` 注释的命令需要开启 `setopt interactivecomments`（oh-my-zsh 默认已开）。
- 代码块里写成 `VAR=...   # 改成...` 的变量，先改成你自己机器上的值，再粘贴执行。
- 两套 shell 环境，不要混用：
  - `source env_ros.sh`：把 conda 移出 PATH，加载系统 ROS 2，用于 `colcon build` 和 ROS 节点。它还会设置 `PYTHONNOUSERSITE=1` 和 `ROS_DOMAIN_ID=30`。
  - `source env.sh`：激活 conda `wuji2`，只用于跑 `scripts/01..03`。
  - `scripts/run_manus_wuji2.sh` 会自己处理两套环境，任意终端都能直接运行。
- conda 默认装在 `~/miniconda3`；不在这里的话，先 `export WUJI2_CONDA=/你的/conda 前缀`。

## 1. 拉取仓库

```bash
git clone git@github.com:Xuanyuan-jiang/Manus2Wujihand2.git
cd Manus2Wujihand2
```

仓库不用 Git LFS 和 submodule。闭源的 MANUS 运行库在第 4 步单独恢复。

## 2. 安装 ROS 2

```bash
DISTRO=$(. /etc/os-release; case $VERSION_CODENAME in noble) echo jazzy;; jammy) echo humble;; esac)
echo "DISTRO=$DISTRO"     # 为空说明系统不是 24.04 / 22.04
```

已经有 `/opt/ros/$DISTRO/setup.bash` 的，跳到下面的 `apt install` 补依赖即可。否则先加 ROS apt 源：

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb
sudo apt update
```

安装 ROS 和编译依赖（用 `ros-base` 即可，不需要 `desktop`）：

```bash
sudo apt install -y ros-$DISTRO-ros-base ros-dev-tools \
  ros-$DISTRO-rclcpp ros-$DISTRO-std-msgs ros-$DISTRO-geometry-msgs \
  ros-$DISTRO-rosidl-default-generators ros-$DISTRO-ament-cmake ros-$DISTRO-ament-index-cpp \
  python3-empy python3-catkin-pkg python3-numpy python3-yaml \
  build-essential cmake git libncurses-dev
```

> 不要把 `source /opt/ros/.../setup.bash` 写进 `~/.bashrc`：它会污染每个 conda 终端的 `PYTHONPATH`。需要 ROS 时用 `source env_ros.sh`。

## 3. 创建 `wuji2` Python 环境

```bash
PYVER=$([ "$DISTRO" = jazzy ] && echo 3.12 || echo 3.10)
conda create -y -n wuji2 python=$PYVER
~/miniconda3/envs/wuji2/bin/python -s -m pip install 'wuji-sdk==2026.8.31' 'numpy>=1.26' pyyaml
```

- `-s` 让 pip 忽略 `~/.local` 里 `pip --user` 装的包。不加的话，pip 会把 `~/.local` 里已有的 `pyyaml` 等当成“已满足”而跳过安装，等到 `env_ros.sh` 屏蔽 `~/.local` 后运行就会报 `No module named 'yaml'`。
- `numpy` 不是 `wuji-sdk` 的硬依赖，pip 不会自动安装，但 `RetargetSession` 运行时必须有。
- `wuji-sdk` 固定用 2026.8.31，这是实测版本。2026.8.3 及更早版本的 `RetargetSession` 不在包顶层（breaking change）；更新的版本没有验证过。
- 始终用 `wuji2` 的**绝对路径解释器**。conda base 里可能装着旧版 `wuji-sdk`，`conda run` 又会吞掉 stdout（退出码 0、没有输出）。

## 4. 恢复 MANUS SDK 运行库

把官方分发包整个放到仓库根目录（`.gitignore` 已忽略它），然后运行：

```bash
ls -d MANUS_Core_3.2.0_SDK_Linux        # 放在仓库根
./tools/fetch_manus_sdk.sh
```

> 验收：`OK: 已从 ... 恢复 MANUS SDK -> .../ros2_ws/src/ManusSDK/lib`。
> 分发包放在别处也可以：`./tools/fetch_manus_sdk.sh '/path/to/MANUS_Core_3.2.0_SDK_Linux/ROS2 package/ManusSDK/lib'`。
> 脚本按 `tools/manus_sdk_lib.sha256` 校验，版本不对会拒绝。

## 5. MANUS dongle 的 udev 规则

```bash
sudo cp config/99-manus-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# 拔插一次 dongle
lsusb -d 3325:
```

> 验收：出现 `3325:0049 Manus VR ... Sensor Dongle`。
> 看到的是 `3325:b00e Manus VR Bootloader` 时，设备处于 Bootloader（固件更新）模式，发布器用不了它：先重新拔插；还是 `b00e` 的话，用 MANUS Core 完成固件更新。
> Hand 2 走网线，不需要任何 udev 规则。

## 6. 编译 ROS 2 包

在子 shell 里编译，出错立即停：

```bash
(
  set -eo pipefail
  export WUJI2_NO_OVERLAY=1
  source env_ros.sh
  cd ros2_ws
  colcon list          # 应恰好是 manus_input_py / manus_ros2 / manus_ros2_msgs
  colcon build --symlink-install \
    --packages-select manus_ros2_msgs manus_ros2 manus_input_py \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
)
```

> 验收：`Summary: 3 packages finished`，0 failed。
> `-DPython3_EXECUTABLE=/usr/bin/python3` 不能省：CMake 自动探测可能选中 conda 的 Python，那里没有 `empy`，rosidl 生成消息时报 `No module named 'em'`。
> 仓库是从别的机器拷过来（而不是 clone）的，或者换过 ROS 发行版，先 `rm -rf ros2_ws/build ros2_ws/install ros2_ws/log`，旧缓存里存的是旧路径。

编译后自检：

```bash
(
  source env_ros.sh >/dev/null
  ros2 pkg list | grep -E '^manus_'
  ldd ros2_ws/install/manus_ros2/lib/manus_ros2/manus_data_publisher | grep -i manus
  python3 tests/test_skeleton_mapping.py | tail -1
  ~/miniconda3/envs/wuji2/bin/python -c "import rclpy, yaml, numpy, wuji_sdk; print('wuji2 OK', wuji_sdk.__version__)"
)
```

> 验收：
> - 三个 `manus_*` 包都在；
> - `libManusSDK-amd64.so => ...` 已解析，不是 `not found`；
> - 测试打印 `全部断言通过`；
> - 打印 `wuji2 OK 2026.8.31`（前面可能有一段 Wuji SDK 的 Privacy Notice，正常）。

## 7. 连上 Hand 2（只读）

### 7.1 网络

两只手是固定 IP：右 `192.168.1.111`、左 `192.168.1.110`。本机要能从接手的那块网卡发包到 `.110` / `.111`，且本机地址不能是 `.110` / `.111`。两块网卡都在这个网段时，发往手的包可能走错网卡（见接法 C）。先看现状：

```bash
ip -4 -br addr
nmcli -g NAME,DEVICE con show --active
```

按手的接法三选一。

**A. 手插在实验室路由器或交换机上，本机经同一局域网访问（有线或 Wi-Fi 都行）**

本机在这个网络上要用固定 IP，免得 DHCP 恰好分到 `.110` / `.111`。下面以 `.120` 为例，DNS 沿用当前 DHCP 下发的值：

```bash
CON="PINE_LAB_5G 1"                  # 改成本机连这个网络的连接名（上面 --active 的输出）
DNS=$(nmcli -g IP4.DNS con show "$CON" | tr -s ' |' ' ')
sudo nmcli con mod "$CON" ipv4.method manual ipv4.addresses 192.168.1.120/24 \
  ipv4.gateway 192.168.1.1 ipv4.dns "$DNS" connection.autoconnect-priority 10
sudo nmcli con up "$CON"
```

- `autoconnect-priority 10`：同一个网络存了多份连接配置时，保证优先用这一份，否则重连后可能又回到 DHCP。
- `con up` 会让这块网卡断开几秒。远程 SSH / VPN 如果走这块网卡，会跟着掉线；网关或 DNS 填错时，这块网卡就上不了网。
- 最好在路由器上把 `.110` / `.111` / `.120` 移出 DHCP 地址池（或做静态保留），否则以后可能被分配给别的设备。

**B. 手直连本机一个空闲的有线网口**

```bash
IFACE=enp5s0                         # 改成接 Hand 2 的网口名（ip -br link）
sudo nmcli con add type ethernet ifname "$IFACE" con-name wuji-hand \
  ipv4.method manual ipv4.addresses 192.168.1.2/24 ipv4.never-default yes
sudo nmcli con up wuji-hand
```

- 此时其它网卡（Wi-Fi 等）不能也在 `192.168.1.0/24`，否则用 C。
- 一块网卡同时只有一个活动连接。如果这个网口已经在用（比如接着机械臂网络），`con up wuji-hand` 会把原来的连接顶掉，也改用 C。

**C. 手接在有线网口，但这个网口已有别的网络，或者 Wi-Fi 也在 `192.168.1.0/24`**

不新建连接，而是在网口原有的连接上追加一个 `/32` 地址，只把 `.110` / `.111` 两个地址路由到这个网口。局域网的其它地址照旧走 Wi-Fi，网口原有的网络也不受影响：

```bash
CON="有线连接 2"                      # 改成接 Hand 2 的那个网口上原有的连接名
IFACE=enp4s0                         # 改成该网口名
sudo nmcli con mod "$CON" +ipv4.addresses 192.168.1.2/32 \
  +ipv4.routes "192.168.1.110/32 src=192.168.1.2" \
  +ipv4.routes "192.168.1.111/32 src=192.168.1.2"
sudo nmcli dev reapply "$IFACE"
ip route get 192.168.1.111           # 应显示 dev <该网口> src 192.168.1.2
```

撤销时把上面的三个 `+` 改成 `-` 再执行一遍。

> 症状对照：`scan()` 能扫到手，但连接时报 `No reply from Zenoh queryable`、ping 不通。原因是扫描靠广播能收到手的回复，而单播包按路由表走了另一块网卡。

最后都验证一下：

```bash
ip route get 192.168.1.111           # 必须走接手的那块网卡
ping -c 2 -W 1 192.168.1.111 && ping -c 2 -W 1 192.168.1.110
```

### 7.2 关掉 Wuji Studio

Wuji Studio 会占住 Hand 2 的 SDK bridge，SDK 就连不上手。

```bash
pgrep -af 'wuji-studio|wuji-hand-hmi' && pkill -x wuji-studio
pgrep -x wuji-studio || echo "OK: studio 已退出"
```

被 systemd 或开机自启拉起的，用 `systemctl --user list-units | grep -i wuji` 找到后 disable。

### 7.3 扫描与体检

```bash
~/miniconda3/envs/wuji2/bin/python -s - <<'PY'
import wuji_sdk as w
for d in w.SdkManager.instance().scan():
    side = {"J": "LEFT", "K": "RIGHT"}.get(d.sn[3:4], "?")
    kind = f"Wuji Hand 2 ({side})" if d.sn.upper().startswith("WH2") else "gen1 / other"
    print(f"{d.sn:<20} {str(d.transport_type).split('.')[-1]:<4} {d.address}  -> {kind}")
PY

~/miniconda3/envs/wuji2/bin/python -s scripts/05_clear_faults.py --side both
```

> 验收：
> - 扫描能看到两台 `WH2*`，地址分别是 `.111` / `.110`；
> - `05_clear_faults.py` 退出码 0，20 个关节全部 Ready、无故障码。它默认**只读**，加 `--clear` 才会清故障。
>
> 判断是不是 Hand 2 要看 **SN 前缀 `WH2`**，不看 `device_type`：USB 枚举出的设备一律报 `WujiHand`（一代）。SN 第 4 位 `J` = 左手，`K` = 右手。
> 扫描发现不了（多网卡时组播发现不稳定）的话，直接用地址，比如 `scripts/05_clear_faults.py --side right --address 192.168.1.111:7447`。

## 8. 首次联调

### 终端 A：MANUS 发布器

```bash
source env_ros.sh
ros2 run manus_ros2 manus_data_publisher 2>&1 | tee /tmp/manus.log
```

### 终端 B：骨架 → 21 点 MediaPipe

```bash
source env_ros.sh
ros2 run manus_input_py manus_input --config config/manus_input_both_hands.yaml
# 只有右手手套时改用 config/manus_input_right_only.yaml
```

### 终端 C：链路验收

```bash
source env_ros.sh
ros2 topic list | grep -E 'manus_glove|hand_input'
timeout 12 ros2 topic hz /manus_glove_0
timeout 12 ros2 topic hz /hand_input_right
timeout 12 ros2 topic hz /hand_input_left
for s in right left; do
  timeout 12 ros2 topic echo /hand_input_$s --once --qos-reliability best_effort > /tmp/hi_$s.yaml
  python3 -c "import yaml; m=next(x for x in yaml.safe_load_all(open('/tmp/hi_$s.yaml')) if x); print('$s', len(m['data']))"
done
```

> 验收：各 topic 约 120 Hz，最后打印 `right 63` / `left 63`。
> `ros2 topic echo` **必须带 `--qos-reliability best_effort`**：链路上的 publisher 全是 BEST_EFFORT，不带这个参数会“订阅成功但没有数据”。

标定质量检查：把手完全摊平，运行 `python3 tools/check_glove_live.py`（左手加 `--topic /manus_glove_1`，以 `side` 字段为准）。出现 FAIL 就先做第 10 节的标定。

### 终端 D：

```bash
./scripts/run_manus_wuji2.sh --side both
```

节点会先等 30 帧新鲜有效输入，再检查 20 轴在线状态、错误码、当前位置和限位，全部通过后才 enable；enable 之后还要再等到一帧新输入才开始下发。日志出现 `HAND ENABLED: sn=...` 表示已接管。

`--side both` 启动的是**两个独立进程**：SDK 的连接/使能是阻塞调用，放在同一进程里会互相卡住。一只手 fail-safe 时，另一只**继续运行**（免得松开已经抓住的东西），脚本会打出提示；要全部停下就按 Ctrl+C。`--side both` 不接受 `--topic` / `--address`，要单独指定时分两个终端启动：`--side right ...` / `--side left ...`。

## 9. 日常启动与停止

启动（每次开机）：

```bash
# 前置：wuji-studio 已关，dongle 已插，两只手已上电，ping 得通 .110 / .111
# 终端 A
source env_ros.sh && ros2 run manus_ros2 manus_data_publisher 2>&1 | tee /tmp/manus.log
# 终端 B
source env_ros.sh && ros2 run manus_input_py manus_input --config config/manus_input_both_hands.yaml
# 终端 C（任意终端都可以；想只看数值就加 --dry-run）
./scripts/run_manus_wuji2.sh --side both
```

停止时**逆序**：

1. 终端 C 按 Ctrl+C，等每只手都打印 `hand disabled`（dry-run 不会打印这一行，因为它从没连过手）；
2. 终端 B 按 Ctrl+C；
3. 终端 A 按 Ctrl+C；
4. 确认没有残留进程：`pgrep -af '04_manus_wuji2|manus_data_publisher|manus_input' || echo OK`。

## 10. 手套标定

仓库自带的 `ros2_ws/src/manus_ros2/calibration/{Right,Left}MetaglovePro.mcal` 是**官方默认标定**（别人的手），只够验证链路是否打通。换人佩戴时必须重新标定。

1. 停掉占用手套的进程。标定期间 MANUS Core 要独占手套：
   ```bash
   pkill -f manus_data_publisher; pkill -f manus_input
   ```
2. 启动 MANUS Core Dashboard。分发包自带 Linux 版，需要图形会话；也可以用 Windows 版 MANUS Core。
   ```bash
   cd "MANUS_Core_3.2.0_SDK_Linux/MANUS Core Dashboard (Linux build)"
   chmod +x MANUS_Core && ./MANUS_Core
   ```
   完成标定流程，并在 Raw Skeleton Data 视图检查五指伸直、屈曲和逐指捏合。然后导出 `.mcal`。
3. 用导出的文件覆盖同名文件（文件名是写死的；原文件在 git 里，`git checkout -- <文件>` 可以还原）：
   ```bash
   cp /导出路径/RightMetaglovePro.mcal ros2_ws/src/manus_ros2/calibration/RightMetaglovePro.mcal
   python3 tools/check_mcal.py ros2_ws/src/manus_ros2/calibration/RightMetaglovePro.mcal
   ```
   `check_mcal.py` 必须输出“自洽性检查全部通过”。SDK **不校验**文件里的几何属于哪一侧：一份标着 `right`、内容却是左手的文件会被静默接受，结果是关节角离谱、手指朝腕部折回。
4. 重启终端 A（标定只在连接时加载）和终端 B。用 `--symlink-install` 编译过的不需要重新编译。然后摊平手运行 `python3 tools/check_glove_live.py`，应输出“全部接近零位”。最后重新 dry-run。

> `.mcal` 只含手型几何（`cmcPosition`、`fingerLength`、`wristPosition`、`wristRotationOffset`），**不含**弯曲传感器原始值到角度的映射。摊平手时 PIP 读到上百度（原部署机实测 `IndexPIPStretch` 达 +143°）属于传感器标定问题，换 `.mcal` 修不好，只能在 MANUS Core 里重做标定流程。

**手套伸直、Hand 2 却向侧面偏**：先 dry-run，看四指侧摆轴（`index/middle/ring/pinky` 每组的第 2 个值）。如果目标值本身就同向偏离 0（原部署机上实测伸直时 `index_S2 ≈ +0.44 rad`），问题出在手套标定，按上面的流程重新标定。只有在确认“20 轴命令全为 0 时，Hand 2 的机械姿态本身就歪”之后，才考虑 Hand 2 的零点。**不要随手调用 `set_origin()`**：它会把当前物理姿态永久定义为零点。`scripts/02_wuji2_init.py` 也不是标定脚本，它会以 1.5 A 把 20 轴拉回零位。

## 11. 控制参数与安全机制

`run_manus_wuji2.sh` 会把参数透传给 `04_manus_wuji2.py`。默认值面向实时跟随：

| 参数 | 默认 | 允许范围 / 说明 |
|---|---|---|
| `--rate` | 120 Hz | 与 `/hand_input_*` 对齐；≤ 1000 |
| `--max-speed` | 8.0 rad/s | 每轴限速。每 tick 约 3.8°，正常手速下几乎无感，但能拦住手套跟丢造成的跳变 |
| `--kp` | 4.0 | ≥ 3.0（设备出厂值 1.5） |
| `--kd` | 0.02 | 0.01–0.05 |
| `--effort-limit` | 1.5 A | ≤ 1.5 |
| `--watchdog` | 0.2 s | 输入中断超过这个时间就 fail-safe |
| `--warmup-frames` | 30 | enable 前需要的有效输入帧数 |

调参方向：发闷、跟不上 → 先加 `--max-speed`，再加 `--kp`；嗡嗡响、抖 → 降 `--kp` 或加 `--kd`。

运行时的保护：

- 输入：BEST_EFFORT/depth=1、长度必须是 63、拒收 NaN/Inf、检查 MediaPipe 米制尺度；
- 骨架：首帧检查腕→MCP 距离、MCP 展宽、指节长度；手性持续监视，握拳时连续 10 帧一致才给结论，先前的镜像告警会被之后的正常握拳撤销（`manus_input` 日志）；
- 输出：按 Hand 2 URDF 的 20 轴限位 clamp，每轴限速；
- 时序：输入 watchdog；enable 后等到一帧新输入才开始下发；
- 故障：按固件的四级 severity 处理。`Warning`（如 `Enc1BitRate`）只限流记录、不停机；`DeferredStop` / `ImmediateStop` / `Fatal` 以及无法识别的非零故障码一律 fail-safe；
- 退出：任何异常或退出路径都会失能并断开。

环境变量：`WUJI2_CONDA`（conda 前缀）、`WUJI2_ENV`（环境名，默认 `wuji2`）、`WUJI2_PYTHON`（直接指定解释器）、`WUJI2_ROS_DISTRO`（强制指定 `jazzy` / `humble`）。

## 12. 排障

**分段定位**

```
/manus_glove_0 没有数据          → MANUS 侧：license / dongle 被占用 / 手套没配对 / Bootloader 模式
有 glove 但 /hand_input_* 没数据  → manus_input：看终端 B 的报错；echo 时是否带了 best_effort
run_manus_wuji2.sh 报导入失败     → wuji2 环境（第 3 步）或 ROS overlay（第 6 步）
有关键点但 qpos 乱                → 手套标定（第 10 节）；tools/check_hand_input.py --live --topic /hand_input_right
qpos 正常但手不动                 → 没 enable / 关节有故障（05_clear_faults.py）/ effort_limit 太低
```

| 现象 | 原因与处理 |
|---|---|
| colcon 报 `No module named 'em'` | CMake 选中了 conda 的 Python。用第 6 步的完整命令，带上 `-DPython3_EXECUTABLE=/usr/bin/python3`，并先清掉 `ros2_ws/build` |
| colcon 报 `canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'` | `~/.local` 里的新版 setuptools 在捣乱。必须先 `source env_ros.sh`（它会设置 `PYTHONNOUSERSITE=1`）再编译 |
| 链接时报 `cannot find -lManusSDK-amd64` | 没有做第 4 步 |
| `Failed to initialize the SDK. Are you sure the correct ManusSDKLibary is used?` | `.so` 加载失败：重跑第 4 步，并用 `ldd` 检查 |
| `MANUS data publisher could not connect, trying again in a second.` 反复刷屏 | dongle 被 MANUS Core 或另一个发布器占用、udev 规则未生效，或 `lsusb` 显示的是 `3325:b00e` |
| `don't have a valid SDK Integrated license` | dongle 的 license 缺少 SDK feature，只能找 MANUS 处理 |
| `ros2 topic echo` 订阅上了但没有数据 | 没带 `--qos-reliability best_effort` |
| 两个终端互相看不到 topic | 有一个终端没 `source env_ros.sh`（`ROS_DOMAIN_ID` 必须都是 30） |
| 扫不到 Hand 2 | 先 ping；再确认 Wuji Studio 已关；多网卡时改用 `--address`；检查 VPN 是否接管了 `192.168.1.0/24` 的路由 |
| wuji2 里 `import yaml` 失败，但 pip 显示已安装 | 当初装进的是 `~/.local`。用 `python -s -m pip install ...` 重装（第 3 步） |
| 关节报故障、手不能使能 | `scripts/05_clear_faults.py --side right`（只读查看）→ 确认手指能自由屈伸后加 `--clear`。清完一使能又报同一个码时，用 `scripts/06_isolate_joint.py` 做零负载隔离实验；`ImmediateStop` 类故障只能断电重启 |

**不要运行**旧的 `wujihand_driver` / `wujihand_controller` launch（来自 `wuji-hand-teleop` / `wujihandros2`）。它们是给一代 USB 手用的，不能驱动网口上的 Hand 2。`lsusb` 里的 `0483:2000` 也是一代手，不是 Hand 2。

## 13. 仓库结构与上游差异

```
env.sh / env_ros.sh   两套 shell 环境（见第 0 节）
config/               manus_input 配置（双手 / 仅右手）、MANUS udev 规则
scripts/
  01_read_only.py     只读：连右手、打印 20 轴角度
  02_wuji2_init.py    ⚠ 会 enable 并把 20 轴拉回零位
  03_wuji2_test.py    ⚠ 预设手势动作测试
  04_manus_wuji2.py   主遥操作节点（通过 run_manus_wuji2.sh 启动）
  05_clear_faults.py  关节故障查看 / 清除（默认只读）
  06_isolate_joint.py 单关节隔离使能实验
  07_tactile_qp_wuji2.py  触觉 QP 实验（可选；需 --urdf 指向 Hand 2 URDF，wuji-sdk 包里不含；插件说明见 finger_qp/README.md）
ros2_ws/src/
  ManusSDK/           MANUS SDK 头文件；lib/ 由 tools/fetch_manus_sdk.sh 恢复
  manus_ros2/         官方 ROS 2 节点 + 本地补丁，含 calibration/*.mcal
  manus_ros2_msgs/    消息定义
  manus_input_py/     骨架 → 21 点 MediaPipe（非官方）
tests/                录制的 /hand_input 帧与取点逻辑测试
tools/                SDK 恢复脚本；标定、骨架、手套读数检查工具
finger_qp/            独立的触觉 QP 插件
```

**代码来源**：`ManusSDK/`、`manus_ros2/`、`manus_ros2_msgs/` 取自 MANUS Core 3.2.0 SDK for Linux 分发包里的 `ROS2 package/`，版权归 MANUS 所有，使用需要带 SDK feature 的 license。`manus_input_py/` 来自 `wuji-technology/wuji-hand-teleop` commit `3fa58481e971bf1588b57751ea44d99abe1d95b5`（官方 main 分支已删除该包）。vendored 代码首次提交时是逐字未改的副本，之后的本地修改都是独立提交：`git log -p ros2_ws/src/` 就是相对上游的完整改动清单。

`manus_ros2` 里有两处标了 `LOCAL PATCH` 的本地修改，**升级 MANUS SDK 时必须一起带走**：

1. **`JointTypeToString` 输出骨头名**。上游把 SDK 按骨头命名的枚举翻译成关节名时整体错了一位：

   | SDK 枚举（骨头） | 节点实际位置 | 上游标成 | 应为 |
   |---|---|---|---|
   | `_Metacarpal` | 掌骨根 ≈ 腕 | `MCP` | 腕 / 掌骨根 |
   | `_Proximal` | 近节指骨根 | `PIP` | **MCP** |
   | `_Intermediate` | 中节指骨根 | `IP` | **PIP** |
   | `_Distal` | 远节指骨根 | `DIP` | DIP |
   | `_Tip` | 指尖 | `TIP` | TIP |

   照字面取点会让每根手指变成 `[掌骨根, MCP, PIP, DIP]`，丢掉指尖，retarget 输出四指同向外摆。`manus_input_py` 按 `(chain_type, 骨头)` 选点，并带旧标签兼容表。拇指链是 `[Metacarpal, Proximal, Distal, Tip]`（没有中节）。
2. **自动加载 `.mcal`**。上游 3.2.0 删掉了这个功能，改由 MANUS Core 持有标定；保留它之后，遥操作时不必开着 Core。

**坐标系手性**：`manus_input_py` 的 `_node_position` **不做** y 取反（这里曾被改错两次）。手性判据是 `dot(curl, normalize(cross(u, r)))`，其中 `u` = 腕→中指 MCP，`r` = 小指 MCP→食指 MCP，`curl` = 中指尖相对中指 MCP、垂直于 `u` 的分量。屈曲时右手为负、左手为正，基准来自 Hand 2 的 `right.urdf` / `left.urdf`，已固化在 `tests/test_skeleton_mapping.py` 中。`manus_input` 只采信绝对值 ≥ 5 cm（握拳级别）的帧：手指伸直时默认标定会读出约 3 cm 的反向“后翘”，没戴在手上的手套可以读出任意值，门槛低了既会误报镜像，也可能在真镜像时放过。
