# MANUS → Wuji Hand 2 部署手册

> 生成于 2026-09-18。以下事实已在本机**独立复核**（非仅来自文档）：
>
> - `WH2KA01260818006 @ 192.168.1.111:7447` 确为 `DeviceType.WujiHand2`，ping 0% loss (~0.09ms)
> - `lsusb` 里两只 `0483:2000` 确为**一代** `DeviceType.WujiHand`
> - `wuji_sdk 2026.8.31` 具备 `RetargetSession` 与 `HandModel.WujiHand2`
> - `wuji2` env 已于 15:48 升级至 **Python 3.12.14**，`wuji_sdk` 导入正常
>
> 待办的环境清理（不影响功能）：
>
> ```bash
> # 升级残留：25M 孤儿目录 + 两个野生软链
> rm -rf /home/mzsun/miniconda3/envs/wuji2/lib/python3.10
> rm -f  /home/mzsun/miniconda3/envs/wuji2/lib/python3.1.c~
> ```
>
> 注：正文 2.1 节写的 "conda base 是 python 3.14 + 旧版 wuji-sdk" 仍然成立 ——
> **始终用绝对路径 `/home/mzsun/miniconda3/envs/wuji2/bin/python`**，不要依赖 `conda activate`。

---

# MANUS Metagloves Pro → Wuji Hand 2 部署指南

## 0. 路线结论(先读这 5 行)

1. **你要控的 Wuji Hand 2 不在 USB 上,在网口上**:一对双手,右 `WH2KA01260818006 @ 192.168.1.111:7447`、左 `WH2JA01260813009 @ 192.168.1.110:7447`(Zenoh/UDP,各 20 关节全在线;handedness 已逐台向设备核实)。`lsusb` 里那两只 `0483:2000 WUJIHAND` 被 SDK 判定为 **第一代 WujiHand**,不是 Hand 2。
2. **手侧不需要 Docker、不需要 wujihandcpp/wujihandros2**。Hand 2 的官方控制栈是 `wuji-sdk`(pip),而你的 **conda `wuji2` env 里已经装好了 2026.8.31**。当前 `numpy`、`PyYAML`、Jazzy `rclpy` 已验证可在该环境中同时导入。
3. **`wuji2` env 不是白建的 —— 它是手侧的主力环境**。整条「关键点 → retarget → 驱动实体手」用它一个包就能跑通(`RetargetSession.for_hand(HandModel.WujiHand2, ...)`)。
4. **只有 MANUS 侧需要 ROS2**:MANUS 在 Linux 上只有 C++ SDK(3.2.0 起为单个 `libManusSDK-amd64.so`),官方只通过 ROS2 节点 `manus_ros2` 暴露。装**系统 ROS2 Jazzy** 原生编即可——实测该 `.so` 最高只需 `GLIBC_2.29`/`GLIBCXX_3.4.26`(本机 2.39/3.4.33),`ldd` 零 not found；构建还需要 `ament_index_cpp`、rosidl 生成器和 `libncurses-dev`。**不需要 Docker。**
5. **`manus_ros2` / `manus_ros2_msgs` / `ManusSDK` 用 MANUS 官方 3.2.0 分发包的 `ROS2 package/`**(已 vendored 进 `ros2_ws/src/`,并带两处标了 `LOCAL PATCH` 的本地修正)。`manus_input_py` 不在官方包里,取自 `wuji-hand-teleop` commit `3fa5848`(经 `Fanlinfeng23/Wuji_Retargeting` 转手),upstream main 已删除该包。**只编这 3 个包。**

**架构**:

```
[系统 ROS2 Jazzy]                        [conda wuji2, Python 3.12.14]
manus dongle(USB)
   → manus_data_publisher (C++)
   → /manus_glove_0
   → manus_input_py
   → /hand_input_right, /hand_input_left (各 63 floats = 21×3 MediaPipe,互不阻塞)
        │
        ↓ scripts/04_manus_wuji2.py（默认 dry-run）
        RetargetSession.step() → 限位/限速/watchdog → (20,) JointCommand
                  ↓
        192.168.1.111:7447 → 右手 Hand 2 / 192.168.1.110:7447 → 左手 Hand 2
```

---

## 1. 必须先纠正的 4 条事实

| 你/文档的假设 | 实测真相 |
|---|---|
| lsusb 那两只 0483:2000 = 两只 Hand 2 | 是 **一代 WujiHand**(`DeviceType.WujiHand`),且日志显示它们最近连接一直 `USB I/O error` |
| Hand 2 走 USB,需要 `99-wujihand.rules` | Hand 2 走 **RJ45 / 静态 IP / Zenoh**,**零 udev 需求**。你现有的 udev 规则只对一代手有意义 |
| 需要装 wujihandcpp deb + ROS2 + wujihandros2 | 这 4 个官方仓库(`wujihandpy`/`wujihandros2`/`wuji-retargeting`/`wuji-hand-teleop`)**已于 2026-08-31 全部 archive**,功能并入 `wuji-sdk`。全库 grep 不到任何 `hand2`/`WujiHand2` |
| Ubuntu 24.04 上装 ROS2 Humble | noble 源里 `ros-humble-*` = **0 个**(jazzy 3673 / kilted 2861)。原生路线不可行 |

---

## 2. 【先做这个】最短硬件验证路径(不装 ROS2、不用 Docker)

这一节全部在宿主机跑,纯 Python,**5 分钟内能确认手在线、算法可用、dongle 被识别**。默认不发送任何运动指令；跑不通就别往下走。

### 2.0 关掉 Wuji Studio(它占着 Hand 2 的 SDK bridge)

```bash
ps -eo pid,user,comm,args | grep -E 'wuji-studio|wuji-hand-hmi' | grep -v grep
pkill -x wuji-studio
for i in $(seq 10); do pgrep -x wuji-studio >/dev/null || break; sleep 1; done
pgrep -x wuji-studio >/dev/null && echo "STILL RUNNING -> pkill -9 -x wuji-studio" || echo "OK: studio 已退出"
```

> 验收:`OK: studio 已退出`。
> 日志实证:`~/.wuji/logs/studio_*.log` 里有 `Bridge up: device WH2KA01260818006`,Studio 不关,SDK 抢不到手。
> ⚠️ 若被 systemd/autostart 拉起会自动重启:`systemctl --user list-units | grep -i wuji` 后 disable。

### 2.1 验证 numpy 与 wuji-sdk

```bash
PY=/home/mzsun/miniconda3/envs/wuji2/bin/python
"$PY" -c "import numpy" 2>/dev/null || "$PY" -m pip install 'numpy>=1.26'
"$PY" -c "import numpy,wuji_sdk;print(numpy.__version__, wuji_sdk.__version__)"
```

> 验收:当前实测为 `2.5.3 2026.8.31`；不要把 numpy 小版本写成固定验收值。
> 原因:`wuji-sdk` 的 METADATA 里 `Requires-Dist: numpy ; extra == 'dev'` —— numpy **不是**硬依赖,pip 不会带,但 `RetargetSession.step()` 运行时必须有它。
> **全程用绝对路径解释器**:conda base 自动激活的是 python 3.14 + 旧版 wuji-sdk 2026.8.3(有 breaking change),`conda run` 还会吞掉 stdout。

### 2.2 扫描 + 确认网络

```bash
ip -4 addr show enp132s0 | grep -w inet
ping -c 2 -W 1 192.168.1.111

/home/mzsun/miniconda3/envs/wuji2/bin/python - <<'PY' 2>/dev/null
import wuji_sdk as w
for d in w.SdkManager.instance().scan():
    sn=d.sn; side={"J":"LEFT","K":"RIGHT"}.get(sn[3] if len(sn)>3 else "","?")
    tag = f"WUJI HAND 2 (side={side})" if sn.upper().startswith("WH2") else "gen1 / other"
    print(f"sn={sn:<22} type={str(d.device_type).split('.')[-1]:<11} "
          f"transport={str(d.transport_type).split('.')[-1]:<4} addr={d.address}   -> {tag}")
PY
```

> 验收(实测输出):
> ```
> inet 192.168.1.2/24
> 0% packet loss  (~0.14ms)
> sn=WH2KA01260818006     type=WujiHand2   transport=UDP  addr=192.168.1.111:7447  -> WUJI HAND 2 (side=RIGHT)
> sn=306735773434         type=WujiHand    transport=USB  addr=usb:0483:2000:...   -> gen1 / other
> sn=367C39563134         type=WujiHand    transport=USB  addr=usb:0483:2000:...   -> gen1 / other
> ```
> **判据是 SN 前缀 `WH2*`,不是 `device_type`** —— USB 通道枚举出来的一律打 `DeviceType.WujiHand`,按 device_type 判会把分叉走反。
> ping 不通 → Hand 2 会直接从 scan 里消失,会被误判成"只有一代手"。

### 2.3 只读体检(动手之前的最后一道关)

```bash
W=/home/mzsun/.local/bin/wuji
S=WH2KA01260818006

$W ping --sn $S
$W get node_online   --sn $S
$W get input_voltage --sn $S
$W get board_temperature --sn $S
$W get effort_limit  --sn $S
$W get mit_params    --sn $S

V=$($W get node_online --sn $S | awk '{print $3}')
python3 -c "v=$V;b=[i for i in range(24) if v>>i&1];print('popcount',len(b),'bits',b)"

$W sub joint_diagnostics --sn $S --count 1 --json 2>/dev/null | python3 -c "
import sys,json; j=json.load(sys.stdin)['value']['joints']
N={0:'Init',1:'Ready',2:'Enabled',3:'Stopped'}
print('joints',len(j))
print('error_code_current:',sorted({x['error_code_current'] for x in j}))
print('ext_state:',{N.get(x['status_word']&0xF,'?') for x in j})
print('vbus %.2f..%.2f V'%(min(x['vbus_v_fb'] for x in j),max(x['vbus_v_fb'] for x in j)))
"
```

> 验收:
> - `Status=ok, Firmware=2.6.0`
> - `node_online = 16236015` → `popcount 20`
> - `input_voltage ≈ 12.2`(合法区间 11–13V)、`board_temperature 40–50`
> - `effort_limit` 20 个 `1.5`、`mit_params` 20 组 `{kp:1.5, kd:0.1}`
> - `error_code_current: [0]`、`ext_state: {'Ready'}`
>
> ⚠️ **安全:实测 `soft_limit_enabled = 0`,`soft_pos_min/max` 全 0** —— 固件不挡超程,越界保护 100% 靠软件。任何自写脚本必须 clamp 或只做相对当前位置的小增量。
>
> ⚠️ `wuji doctor --sn <Hand2>` 返回 `no diagnostic recipe for device` —— CLI 的体检只覆盖手套,Hand 2 只能用上面这组手动命令。
> ⚠️ `wuji doctor` 会报 `Interface count: 4 interfaces — multiple interfaces may cause routing issues`(enp132s0 / wlp131s0 / tun0 / docker bridge)。发现不到设备时用 `--address 192.168.1.111:7447` 绕过组播发现。

### 2.4 控制前只读安全门(**不 enable、不发运动指令**)

当前设备的 `soft_limit_enabled = 0`,固件不会替应用层限制位置。旧版文档在未读取真实关节限位的情况下直接发送 `base + 0.15 rad`,与安全要求矛盾,因此不再把微动脚本作为部署前置步骤。

继续部署前只确认以下只读条件:

```bash
W=/home/mzsun/.local/bin/wuji
S=WH2KA01260818006

$W get node_online --sn "$S"
$W get soft_limit_enabled --sn "$S"
$W sub joint_diagnostics --sn "$S" --count 1 --json
```

> 验收:`node_online` 的 popcount 为 20、20 个关节 `error_code_current` 都为 0。
>
> 第一次实际运动推迟到第 8 步:必须先 `--dry-run` 验证关键点和 qpos,再使用具备**逐关节上下限 clamp、超时失能、异常 finally-disable、限流和速率限制**的正式控制节点。没有这些保护时不要调用 `hand.enable()`。

### 2.5 确认 MANUS dongle 被识别

```bash
lsusb -d 3325:
MANUS_DEV=$(lsusb -d 3325:0049 | awk 'NR==1 {gsub(":","",$4); printf "/dev/bus/usb/%s/%s",$2,$4}')
test -n "$MANUS_DEV" && ls -l "$MANUS_DEV"
lsusb -d 1915:
```

> 验收:`3325:0049 Manus VR ... Sensor Dongle`,动态解析出的节点为 `crw-rw-rw-`。USB Device 编号会在重插后改变,不能硬编码 `/dev/bus/usb/003/021`。
> `lsusb -d 1915:` 无输出是**当前状态**;若你的 Metagloves Pro 需要 `1915:83fd` 无线收发器,现在硬件配置就不完整(见第 5 节)。

### 2.6 确认 retarget 算法在本机可跑(不接 MANUS)

```bash
/home/mzsun/miniconda3/envs/wuji2/bin/python - <<'PY'
import numpy as np, wuji_sdk as w
s = w.RetargetSession.for_hand(w.HandModel.WujiHand2, w.Handedness.Right)
kp = np.zeros((21,3), dtype=np.float32)
for f,x in enumerate([-0.04,-0.03,-0.01,0.01,0.03]):
    for k in range(4): kp[1+f*4+k] = [x, 0.03*(k+1), 0.0]
kp[1] = [-0.03, 0.02, 0.01]
q = s.step(kp)
print("qpos", q.shape, q.dtype); print(np.round(q,3))
PY
```

> 验收:`qpos (20,) float32` + 20 个弧度值。
> **到这里手侧基础接口已通。** 完整 MANUS → Hand 2 节点见步骤 8；它已实现，但实体运动前仍必须先完成 dry-run 人工核对。

---

## 3. 主线:MANUS 侧(**原生 ROS2 Jazzy,不用 Docker**)

```
[系统 ROS2 Jazzy]                         [conda wuji2, Python 3.12.14]
manus dongle(USB)
  → manus_data_publisher (C++)
  → /manus_glove_0
  → manus_input_py  → /hand_input_{right,left} (各 63 floats)
                            │
                            ↓ scripts/04_manus_wuji2.py
                        RetargetSession.step()
                            ↓ 逐关节限位 + 每周期限速 + watchdog
                        (20,) → JointCommand
                                      ↓
                        .111 → 右手 Hand 2 / .110 → 左手 Hand 2
```

**已采用方案**:`wuji2` 是 Python 3.12.14,与 Jazzy 的 rclpy 同 ABI；本机已实测同一进程可同时接收 `/hand_input_*`、调用 `RetargetSession`。左右手各跑一个进程。因此直接运行 `scripts/04_manus_wuji2.py`,**不需要 UDP 桥**。

> `env_ros.sh` 会刻意把 conda 从 PATH 移除，只供构建和运行系统 ROS 节点；它现在也会移除残留的 `(wuji2)` 提示符。运行 `scripts/01_read_only.py`、`02_wuji2_init.py`、`03_wuji2_test.py` 前必须重新 `source /home/mzsun/Projects/Wuji2_xyj/env.sh`，或直接使用 `/home/mzsun/miniconda3/envs/wuji2/bin/python`。`scripts/run_manus_wuji2.sh` 已自动处理两套环境，不受此限制。

### 步骤 1 — 加 ROS2 apt 源并装 Jazzy

**目的**:本机 `/etc/apt/sources.list.d/` 目前**没有任何 ROS 源**(实测),必须先加。

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb

sudo apt update
apt-cache policy ros-jazzy-ros-base | head -3
```

> 验收:`apt-cache policy` 的 `Candidate:` 不是 `(none)`。
> 用 `ros-base` 不用 `desktop`:省 ~2GB,且本机 `DISPLAY` 为空,rviz 之类用不上。

```bash
sudo apt install -y ros-jazzy-ros-base ros-dev-tools \
  ros-jazzy-rclcpp ros-jazzy-std-msgs ros-jazzy-geometry-msgs \
  ros-jazzy-rosidl-default-generators ros-jazzy-ament-cmake ros-jazzy-ament-index-cpp \
  python3-empy python3-catkin-pkg python3-numpy python3-yaml \
  build-essential cmake git libncurses-dev

source /opt/ros/jazzy/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"; ros2 --help >/dev/null && echo "ros2 CLI OK"
```

> 验收:`ROS_DISTRO=jazzy` + `ros2 CLI OK`。
> ⚠️ **不要**把 `source /opt/ros/jazzy/setup.bash` 写进 `~/.bashrc` —— 它会污染每个 conda 终端的 `PYTHONPATH`/`AMENT_PREFIX_PATH`。用下面的 `env_ros.sh` 按需加载。

### 步骤 2 — LFS 与源码就位(**你已完成**)

```bash
P=/home/mzsun/Projects/Wuji2_xyj
MI=$P/ros2_ws/src   # 仓库化后三个包已在此
file $MI/ManusSDK/lib/amd64/libManusSDK-amd64.so   # 3.2.0 起 SDK 移到 src/ManusSDK/,单库分架构
ls -d $MI/manus_input_py $MI/manus_ros2 $MI/manus_ros2_msgs
ls -l $MI/manus_ros2/calibration/
```

> 验收:`.so` 是 `ELF 64-bit LSB shared object`(**已确认**,143M/117M);三个包目录齐;`calibration/` 有两个 `.mcal`。
> ❗ 只有 `manus_input_py` 来自 `wuji-hand-teleop`(upstream main 已删除该包);其余三项来自 MANUS 官方 3.2.0 分发包。

### 步骤 3 — MANUS udev 规则

```bash
sudo tee /etc/udev/rules.d/99-manus.rules > /dev/null <<'EOF'
SUBSYSTEM=="usb",  ATTR{idVendor}=="3325", MODE="0666", TAG+="uaccess"
KERNEL=="hidraw*", ATTRS{idVendor}=="3325", MODE="0666"
SUBSYSTEM=="usb",  ATTR{idVendor}=="1915", ATTR{idProduct}=="83fd", MODE="0666"
KERNEL=="hidraw*", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="83fd", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --subsystem-match=hidraw
# 拔插一次 dongle
lsusb -d 3325:
```

> ❗ **`1915:83fd` 那两行不能省**:仓库 CHANGELOG 明写 "without it the BLE skeleton stream was silently empty" —— 手套枚举正常但骨架数据为空。
> 说明:现有 `99-wuji-studio-usb.rules` 是全局 `MODE="0666"`,当前已覆盖一切,这步是防它被卸载。
> 验证规则真的命中(而非被 studio 那条顶掉):
```bash
DEV=$(lsusb -d 3325: | awk '{gsub(":"," ");printf "/dev/bus/usb/%s/%s",$2,$4}')
sudo udevadm test "/sys$(udevadm info -q path -n $DEV)" 2>&1 | grep -E '99-manus|99-wuji-studio'
```
> 必须看到 `99-manus.rules:` 那一行。

### 步骤 4 — 隔离用的 ROS 环境脚本

**目的**:主动移除自动激活的 conda base,再加载系统 ROS。只“开新终端”不够,因为本机新终端仍会自动进入 base。

```bash
cat > /home/mzsun/Projects/Wuji2_xyj/env_ros.sh <<'EOF'
# 用法: source /home/mzsun/Projects/Wuji2_xyj/env_ros.sh
# 仅用于 colcon build 和系统 ROS 节点；不要在 wuji2 Python 终端中 source。
export WUJI2_ROOT=/home/mzsun/Projects/Wuji2_xyj
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
EOF
echo written
```

### 步骤 5 — 编译三个包

> **已变更(仓库化)**:三个包现在是 `ros2_ws/src/` 下的**真实目录**,由本仓库直接管理,
> 不再是指向 `Wuji_Retargeting/` 的软链接。本步骤原有的 `ln -sfn` 准备工作已删除。
> clone 之后只需先恢复 MANUS SDK 运行库:
>
> ```bash
> ./tools/fetch_manus_sdk.sh '/path/to/MANUS_Core_3.2.0_SDK_Linux/ROS2 package/ManusSDK/lib'
> ls ros2_ws/src/        # 应为 ManusSDK manus_input_py manus_ros2 manus_ros2_msgs
> ```

在一个子 shell 中构建。即使外层自动激活 conda base,该命令也会清理它；`cd` 或包集合不符合预期时立即退出:

```bash
(
  set -eo pipefail
  export WUJI2_NO_OVERLAY=1
  source /home/mzsun/Projects/Wuji2_xyj/env_ros.sh
  cd /home/mzsun/Projects/Wuji2_xyj/ros2_ws

  test "$PWD" = /home/mzsun/Projects/Wuji2_xyj/ros2_ws
  test "$(command -v python3)" = /usr/bin/python3
  python3 -c "import em, catkin_pkg; print('ROS build Python OK')"

  colcon list
  FOUND=$(colcon list | awk '{print $1}' | sort | paste -sd ' ' -)
  test "$FOUND" = "manus_input_py manus_ros2 manus_ros2_msgs"

  colcon build --symlink-install \
    --packages-select manus_ros2_msgs manus_ros2 manus_input_py \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
)
```

> 验收:`ROS build Python OK`、`colcon list` 恰好三包、`Summary: 3 packages finished [...] 0 failed`。
> ⚠️ 已知上游 bug:`manus_ros2/CMakeLists.txt` 用了 `find_package(ament_index_cpp)` 但 `package.xml` 漏了该 depend。当前本机已安装 `ros-jazzy-ament-index-cpp`。
> ⚠️ 若看到 `cd: /ros2_ws: No such file or directory`,说明变量已丢失；必须停止,不能继续在项目根目录运行 colcon。不要通过给 conda base 安装 `empy`/`catkin_pkg` 掩盖路径错误。

```bash
(
  set -eo pipefail
  source /home/mzsun/Projects/Wuji2_xyj/env_ros.sh
  ros2 pkg list | grep -E '^manus_(ros2|ros2_msgs|input_py)$'
  ldd /home/mzsun/Projects/Wuji2_xyj/ros2_ws/install/manus_ros2/lib/manus_ros2/manus_data_publisher | grep -i manus
)
```

> 验收:三个包都在;`ldd` 里 `libManusSDK-amd64.so => ...` **已解析**,不是 `not found`(CMakeLists 设了 `INSTALL_RPATH "$ORIGIN"`)。
>
> ⚠️ `-DPython3_EXECUTABLE=/usr/bin/python3` 不能省:CMake 的自动探测会挑中 conda 的 `wuji2/bin/python3`,那个环境没有 `empy`,`rosidl` 生成消息时报 `ModuleNotFoundError: No module named 'em'`。旧的 build 缓存里存着正确值,所以清空 build 目录后才会暴露。
> ❗ 只有 `.so` 拷到位**不算数**,必须 `manus_data_publisher` 真编出来。

### 步骤 6 — 起 MANUS 发布器,判定 license(生死关口)

```bash
source /home/mzsun/Projects/Wuji2_xyj/env_ros.sh
ros2 run manus_ros2 manus_data_publisher 2>&1 | tee /tmp/manus.log
```

另开终端:

```bash
source /home/mzsun/Projects/Wuji2_xyj/env_ros.sh
ros2 node list
ros2 topic list | grep manus
timeout 12 ros2 topic hz /manus_glove_0

if grep -Fq "don't have a valid SDK Integrated license" /tmp/manus.log; then
  echo "LICENSE FAIL: 缺少 SDK Integrated license"
else
  echo "日志中暂未发现 license 错误（这本身不等于 license 已验证）"
fi
grep -E "Manus Core connected|Calibration loaded successfully|publishes in the last" /tmp/manus.log
```

> 验收必须同时满足:`/manus_data_publisher` 节点存在、日志含 `Manus Core connected` 和 calibration/publish 正向记录、`/manus_glove_0` 约 120 Hz,并且日志中不存在 SDK Integrated license 错误。
>
> ❗ **不能用“grep 不到错误”单独判 LICENSE OK**。发布器未连接、回调尚未发生时也不会出现该错误,会形成假阳性。`ManusDataPublisher.cpp:225` 还有一次性错误闩锁,所以必须 `tee` 完整落盘并同时检查正向状态。
> ⚠️ 跑之前确保没有别的 `manus_data_publisher` 或 Windows 端 MANUS Core 在抢 dongle——dongle 一次只接受一个客户端。
> ⚠️ `/manus_glove_0` 是**懒创建**的,手套没真正推数据前 topic 不存在；此时 license 状态仍是“未验证”,不能判成功或失败。

### 步骤 6.5 — 手套标定(MANUS Core Dashboard,Linux 原生)

**没标定过、或 `tools/check_glove_live.py` 报 FAIL,就必须先做这一步。**
用别人的标定只能验证链路通不通,谈不上精度。⚠️ 控制节点现在**默认就驱动实体手**,标定没做好时务必加 `--dry-run`。

3.2.0 的官方分发包自带 **Linux 版 Core Dashboard**(Unity 应用)。注意包里
`Getting started.md` 仍写着 "MANUS Core itself is only available on Windows",
那是没更新的旧文案,与它自己 ship 的 Linux 构建矛盾。

标定期间 **Core 要独占手套连接**,必须先停掉 `manus_data_publisher`。

```bash
# 1. 停掉所有占用手套的进程
pkill -f manus_data_publisher; pkill -f manus_input
pgrep -af 'manus_data_publisher|manus_input' || echo "OK: 已全部退出"

# 2. 启动 Core Dashboard（包里没带执行位,首次要 chmod）
cd "$WUJI2_ROOT/MANUS_Core_3.2.0_SDK_Linux/MANUS Core Dashboard (Linux build)"
chmod +x MANUS_Core
DISPLAY=:1 ./MANUS_Core          # DISPLAY 取本机实际的图形会话号
```

在 Dashboard 里完成右手标定,导出 `.mcal`,覆盖到仓库:

```bash
cp <导出的文件> "$WUJI2_ROOT/ros2_ws/src/manus_ros2/calibration/RightMetaglovePro.mcal"
```

**两道校验,都过了才算标定成功:**

```bash
# a) 文件自洽性：side 标签 vs 几何手性、腕旋转偏移、各指长度
python3 "$WUJI2_ROOT/tools/check_mcal.py" \
  "$WUJI2_ROOT/ros2_ws/src/manus_ros2/calibration/RightMetaglovePro.mcal"

# b) 传感器读数：重启发布器后,摊平手
source "$WUJI2_ROOT/env_ros.sh"
ros2 run manus_ros2 manus_data_publisher &
python3 "$WUJI2_ROOT/tools/check_glove_live.py"
```

> 验收:`check_mcal.py` 输出「自洽性检查全部通过」;`check_glove_live.py` 输出
> 「全部接近零位」。`manus_data_publisher` 终端里应出现
> `Calibration loaded successfully for Right glove (ID: ...)`。
>
> ⚠️ 若报 `VersionError`,说明 `.mcal` 格式与当前 SDK 不匹配 —— 检查是否用了
> 不同版本的 Core 导出。
>
> ⚠️ `CoreSdk_SetGloveCalibration` **不校验**文件内的几何属于哪一侧:一份
> `side` 标着 `right`、内容却是左手的文件会被静默接受,下游表现为关节角离谱、
> 手指朝腕部折回。`check_mcal.py` 就是挡这个的,别跳过。

另一条路是官方 `C++/SDKClient`(FTXUI 终端向导,菜单第 3 项 Calibration),
`install-dependencies.sh` 和 CMakeLists 都在包里。Integrated 模式的依赖只需
`build-essential libusb-1.0-0-dev zlib1g-dev libudev-dev libncurses5-dev gdb`。

---

### 步骤 7 — 起 manus_input,拿到 21 点 MediaPipe

```bash
source /home/mzsun/Projects/Wuji2_xyj/env_ros.sh
ros2 run manus_input_py manus_input \
  --config "$WUJI2_ROOT/config/manus_input_both_hands.yaml"   # 单手用 manus_input_right_only.yaml
```

另开终端:

```bash
source /home/mzsun/Projects/Wuji2_xyj/env_ros.sh
timeout 12 ros2 topic hz /hand_input_right
timeout 12 ros2 topic echo /hand_input_right --once \
  --qos-reliability best_effort > /tmp/hand_input_once.yaml

/usr/bin/python3 - <<'PY'
import yaml
msg = next(x for x in yaml.safe_load_all(open("/tmp/hand_input_once.yaml")) if x)
n = len(msg["data"])
print("hand_input_right length:", n)
assert n == 63, f"expected 63 floats for right hand, got {n}"
PY
```

> 验收:`/hand_input_right` ≈ 120 Hz,脚本打印 `hand_input_right length: 63`。双手时 `/hand_input_left` 同样 63。
> ✅ **已改为每只手各发各的 topic,各 63 floats**。上游把两只手拼成 126 floats 且任一只手缺数据就整体不发(左手掉线会连带停掉右手),本仓库拆开了。
> ❗ `ros2 topic echo` **必须带 `--qos-reliability best_effort`** —— 链路里所有 publisher 都是 BEST_EFFORT,否则会出现“订阅成功但无数据”的假故障。

### 步骤 8 — 启动 Hand 2 重定向节点（先 dry-run）

本仓库现已提供：

- `scripts/04_manus_wuji2.py --side right|left`：`/hand_input_<side>` → 官方 `RetargetSession` → 对应的 Hand 2；
- `scripts/run_manus_wuji2.sh`：加载 ROS overlay 并固定使用 `wuji2` Python 的启动器。

实现依据：[Wuji Hand 2 控制指南](https://docs.wuji.tech/docs/zh/wuji-hand/latest/control-guide/)、[Wuji Hand 2 SDK 接口](https://docs.wuji.tech/docs/zh/wuji-hand/latest/sdk-reference/) 和 [Wuji SDK 手部重定向](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/retargeting/)。

先验证同进程依赖：

```bash
source /opt/ros/jazzy/setup.bash
source /home/mzsun/Projects/Wuji2_xyj/ros2_ws/install/setup.bash
/home/mzsun/miniconda3/envs/wuji2/bin/python -c \
  "import yaml,rclpy,wuji_sdk,numpy; print('rclpy OK:',rclpy.__file__,'yaml:',yaml.__version__)"
```

然后启动 dry-run：

```bash
cd /home/mzsun/Projects/Wuji2_xyj
./scripts/run_manus_wuji2.sh --dry-run
```

dry-run **不会连接 Hand 2、不会 `enable()`、不会发送命令**。本机已用实时 `/hand_input` 验证：能稳定接收约 120 Hz 输入，日志中的 `age` 约 0–7 ms，并按 `thumb/index/middle/ring/pinky` 打印全部 20 个 qpos（rad），静止时 `clamped_now=0`。

节点已实现以下安全门：BEST_EFFORT/depth=1、63 长度与 NaN/Inf 检查、MediaPipe 米制尺度检查、首帧骨架尺寸与手性自检、20 轴运动范围 clamp、每轴限速、输入 watchdog、关节故障按固件四级 severity 分级（`Warning` 不停机）、以及所有退出/异常路径失能并断开。

> ⚠️ **控制节点已改为默认驱动实体手**，`--dry-run` 才是只打印。原先的 `--control` + 确认短语那道闸已移除。

> 官方 SDK 文档明确规定 `RetargetSession.step()` 接受 `(21,3)` MediaPipe 顺序、单位米，返回可直接下发的 `(20,)`。当前 `/hand_input` 实测腕到最远点约 `0.18 m`，顺序和尺度均符合；但方向和动作语义仍必须由操作者在 dry-run 中逐指核对。
>
> 若 `qpos` 不随对应手指变化、频繁触发 clamp，或开掌/握拳方向反了，**不要进入实体模式**。先修正坐标转换/标定。

#### 8.1 手套伸直但 Hand 2 向侧面偏：先判断哪一侧需要标定

不要直接调用 `hand.set_origin()`。按以下顺序判断：

1. 停止实体控制，只运行 `./scripts/run_manus_wuji2.sh --side right --dry-run`，戴着手套并拢、伸直五指。
2. 观察四指侧摆轴：`index/middle/ring/pinky` 每组的第 2 个值，即索引 `5/9/13/17`（`*_S2`）。若这些目标本身明显同向偏离 0，说明偏斜来自 MANUS 骨架/retarget 输入，而不是 Hand 2 擅自偏转。
3. 当前现场实测伸直姿态仍得到 `index_S2≈+0.444 rad`、`middle_S2≈+0.339 rad`，足以产生明显侧摆；仓库加载的又是通用 `RightMetaglovePro.mcal`。因此应先在 Windows MANUS Core 中为当前佩戴者重新完成 Metaglove Pro 标定，并在 Raw Skeleton Data 视图检查五指伸直、屈曲与逐指捏合。官方建议每次使用前重新标定。
4. 从 MANUS Core 导出右手 `.mcal`，备份并替换：

```bash
CAL=/home/mzsun/Projects/Wuji2_xyj/ros2_ws/src/manus_ros2/calibration/RightMetaglovePro.mcal
cp -an "$CAL" "${CAL}.factory-backup"
cp /你的导出路径/RightMetaglovePro.mcal "$CAL"
```

5. 重启终端 A 的 `manus_data_publisher`（标定文件只在连接时加载），再重启 B，并重新 dry-run。先看 qpos，不要直接进入实体模式。
6. 只有在已确认“命令 20 轴全为零时，Hand 2 的机械姿态本身仍然歪”的情况下，才进入 Hand 2 用户零点诊断。`set_origin()` 会把**当下物理姿态**定义为零，姿态没人工摆准时调用会把错误永久引入；没有机械基准和现场确认时不要执行。

`scripts/02_wuji2_init.py` 不是标定脚本：它会 enable 手并以 `1.5 A` 把全部 20 轴移动到零位，不能修复 MANUS 的侧摆目标偏置，也不应用来代替上述判断。

### 步骤 9 — 每次开机后的完整启动与停止顺序

当前完整链路为：

```text
MANUS dongle → manus_data_publisher → /manus_glove_0, /manus_glove_1
  → manus_input_py → /hand_input_right, /hand_input_left
  → 04_manus_wuji2.py --side right → RetargetSession → 安全控制器 → 右手 Hand 2 (.111)
  → 04_manus_wuji2.py --side left  → RetargetSession → 安全控制器 → 左手 Hand 2 (.110)
```

下面的 A–E 是每次开机后的完整顺序。A/B 是常驻进程，C 是一次性验收，D/E 分别驱动右手和左手。

> ⚠️ **D/E 默认就会连接并驱动实体手。** 换过标定、改过取点逻辑、升过 SDK 之后，
> 第一次务必先加 `--dry-run` 过一遍，确认 qpos 合理、无轴顶限位，再去掉该参数。

#### 9.0 首次启动前的一次性条件

- 步骤 1–5 已完成,且 `ros2_ws/install/setup.bash` 存在。
- `colcon build` 的结果为三包成功、零失败。
- 已确认没有 `wuji-studio`、旧 `manus_data_publisher` 或 Windows MANUS Core 抢设备。
- MANUS dongle 已插入；两只 Hand 2 的网线、电源和 `192.168.1.110` / `.111` 路由正常。

```bash
test -f /home/mzsun/Projects/Wuji2_xyj/ros2_ws/install/setup.bash || {
  echo "ERROR: 先完成步骤 5 的 colcon build" >&2
  exit 1
}

pgrep -af 'wuji-studio|wuji-hand-hmi|manus_data_publisher' || true
ping -c 2 -W 1 192.168.1.111   # 右手
ping -c 2 -W 1 192.168.1.110   # 左手
lsusb -d 3325:0049
```

如果第一行 `pgrep` 找到占用进程,先正常关闭它；不要在不清楚 PID 归属时直接 `pkill -9`。

#### 9.1 终端 A:启动 MANUS C++ 发布器

```bash
source /home/mzsun/Projects/Wuji2_xyj/env_ros.sh
ros2 run manus_ros2 manus_data_publisher 2>&1 | tee /tmp/manus.log
```

保持此终端运行。必须看到 `Manus Core connected`；随后按步骤 6 同时检查 license 错误和 calibration/publish 正向日志。

#### 9.2 终端 B:启动关键点转换（双手）

```bash
source /home/mzsun/Projects/Wuji2_xyj/env_ros.sh
ros2 run manus_input_py manus_input \
  --config "$WUJI2_ROOT/config/manus_input_both_hands.yaml"   # 单手用 manus_input_right_only.yaml
```

保持此终端运行。它订阅 `/manus_glove_0`/`_1`,按消息的 `side` 字段分流,
分别发布 63 个 float 到 `/hand_input_right` 与 `/hand_input_left`。两只手互不阻塞:
一只手套掉线不会连带停掉另一只。

启动时每只手各打一行骨架自检:

```text
[骨架自检 Right] 腕->中指MCP=9.4cm MCP展宽=8.4cm 食指近节=4.4cm 手性=-6.0e-02(可信度6.1cm)
```

> 手性判据在**手指伸直时会退化**,此时会打 WARN 说「无法判定」,那不是故障 ——
> 弯曲手指后重启本节点即可看到确定结论。**右手应为负、左手应为正。**

#### 9.3 终端 C:统一运行状态验收

```bash
source /home/mzsun/Projects/Wuji2_xyj/env_ros.sh

ros2 node list
ros2 topic list -t | grep -E 'manus_glove|hand_input'
timeout 12 ros2 topic hz /manus_glove_0
timeout 12 ros2 topic hz /hand_input

timeout 12 ros2 topic echo /hand_input --once \
  --qos-reliability best_effort > /tmp/hand_input_once.yaml
/usr/bin/python3 - <<'PY'
import yaml
msg = next(x for x in yaml.safe_load_all(open("/tmp/hand_input_once.yaml")) if x)
assert len(msg["data"]) == 63, len(msg["data"])
print("OK: /hand_input contains 63 floats")
PY

if grep -Fq "don't have a valid SDK Integrated license" /tmp/manus.log; then
  echo "FAIL: SDK Integrated license 无效"
else
  echo "未发现 license 错误；继续核对下面的连接/标定/publish 正向日志"
fi
grep -E 'Manus Core connected|Calibration loaded successfully|publishes in the last' /tmp/manus.log
```

数据链路可用于 dry-run 的最低条件：

- `/manus_data_publisher` 和 `/manus_input` 都存在；
- `/manus_glove_0` 与 `/hand_input` 持续有频率；
- `/hand_input` 长度严格为 63；
- 日志有连接、标定及持续 publish 的正向记录。

`SDK Integrated license` 必须单独判定。当前现场日志是“持续发布约 120 Hz，**同时报告 license 无效**”：这证明数据当前可读，但授权健康检查仍为失败，不能写成 license OK；应联系 MANUS 修正 license，尤其不能据此做无人值守运行。

#### 9.4 终端 D/E：右手与左手

> ⚠️ **控制节点默认就会连接、使能并驱动实体手。** 早期版本默认 dry-run、实体控制
> 需 `--control` 加确认短语，那道闸已移除。换过标定、改过取点逻辑、升过 SDK 之后，
> 第一次务必先加 `--dry-run` 过一遍。

先各跑至少 30 秒 dry-run，依次做开掌、单指弯曲、握拳：

```bash
cd /home/mzsun/Projects/Wuji2_xyj
./scripts/run_manus_wuji2.sh --side both --dry-run    # 一个终端管两只手
```

也可以分两个终端起，那样能给每只手不同参数：

```bash
./scripts/run_manus_wuji2.sh --side right --dry-run   # 终端 D
./scripts/run_manus_wuji2.sh --side left  --dry-run   # 终端 E
```

验收要求：

- 日志持续显示 `[dry-run] frames=... age=... qpos(rad) thumb=... index=...`；
- 戴着手套动作时 qpos 平滑变化，静止时不持续漂移；
- 对应手指与运动方向正确，**左手要单独核对一遍，不能假定和右手镜像就一定对**；
- `clamped_now` 正常为 0，不出现尺度或 NaN/Inf 拒收；
- 这一阶段 Hand 2 不会连接、不会使能、不会运动。

通过后清空手的运动空间，确认无人接触、皮肤层已装好、急停方案可用，去掉 `--dry-run`：

```bash
./scripts/run_manus_wuji2.sh --side both     # 双手，直接驱动
```

`--side both` 起**两个独立进程**（不是一个进程管两只手 —— SDK 的连接/使能是阻塞
调用，同进程会互相卡住；一只手 fail-safe 也会带走另一只）。脚本转发 Ctrl+C 并等
两个子进程失能退出。**一只手异常退出时另一只继续运行**，脚本会打出提示；要全停
就 Ctrl+C。`--side both` 不接受 `--topic` / `--address`，因为它们对两只手含义不同。

`--side` 决定重定向手别、默认 topic（`/hand_input_<side>`）、ROS 节点名和 Hand 2 地址
（右 `192.168.1.111:7447`、左 `192.168.1.110:7447`）。两个进程完全独立，一只手
fail-safe 不会影响另一只。

默认参数面向实时跟随：`rate=120Hz`、`max_speed=8.0 rad/s`、`kp=4.0`、`kd=0.02`、
`effort_limit=1.5A`、`watchdog=0.2s`。首次上电或换了标定建议先用保守值：

```bash
./scripts/run_manus_wuji2.sh --side right \
  --max-speed 2.0 --kp 3.0 --kd 0.05 --effort-limit 0.5
```

程序先等 30 帧有效新鲜输入，再检查 20 轴在线、错误码、当前位置和限位，然后才 enable。
SDK 连接期间 ROS 单线程回调会暂停，因此 enable 后会重新等待一帧真正的新输入再下发。
正常控制期间输入超时、关节报出 `DeferredStop` 及以上级别故障、离开 Enabled 状态或
任意异常，都会关闭 publisher、失能并断开。`Warning` 级故障（如 `Enc1BitRate`）
只限流记录、不停机。

仍然**不要**运行以下旧 launch 文件：

```text
Wuji_Retargeting/launch/manus_wuji_right.launch.py
Wuji_Retargeting/wuji-hand-teleop/src/wuji_teleop_bringup/launch/wuji_teleop_hand.launch.py
```

它们会启动 `wujihand_driver`/`wujihand_controller`,走的是一代 USB 手和
`wujihandpy/wujihandros2` 栈,不是网口上的 Wuji Hand 2。

最终启动顺序：

```text
终端 A MANUS publisher
  → 终端 B manus_input（双手）
  → 终端 C 验收 /hand_input_right 与 /hand_input_left
  → --side both --dry-run
  → 人工逐手检查 qpos/限位/方向（左手要单独核对，不能假定镜像就对）
  → 去掉 --dry-run，进入实体控制
```

#### 9.5 停止顺序

必须逆序停止:

1. 若已启动终端 D/E，先分别 `Ctrl+C` 停止两个控制节点，确认各自日志出现 `hand disabled`（dry-run 没有该行，因为它从未连接手）。
2. 在终端 B `Ctrl+C` 停止 `manus_input`。
3. 在终端 A `Ctrl+C` 停止 `manus_data_publisher`。
4. 最后检查没有残留节点或进程:

```bash
source /home/mzsun/Projects/Wuji2_xyj/env_ros.sh
ros2 node list
pgrep -af '04_manus_wuji2|manus_data_publisher|manus_input' || echo "OK: 遥操作进程已退出"
```

A–C 从不 enable Hand 2；只有带双重确认的终端 D 实体模式有权 enable，并负责 disable。


## 4. 别踩的坑(校验判定为 WRONG 的原始步骤)

| # | 错误做法 | 为什么错 |
|---|---|---|
| 1 | **把 lsusb 的两只 `0483:2000` 当成 Wuji Hand 2** | SDK 判为 `DeviceType.WujiHand`(一代)。而且 `~/.wuji/logs` 显示它们最近每次 connect 都报 `USB I/O error: transport I/O error` —— 即使想退回一代方案,这两只现在也连不上。真正的 Hand 2 在 192.168.1.111 |
| 2 | **`docker compose up -d` 起官方 wuji-hand-teleop**(本方案已不用 Docker,此条仅供参考) | (a) main 分支 tree 里**零个 submodule gitlink**,`Dockerfile:189-190` 的 `COPY src/wuji-retargeting` 和 `COPY src/wujihandros2/external/...` 必然 `failed to compute cache key: not found`;(b) 镜像硬编码 `USER_UID=1000` 而你是 1002,`entrypoint.sh` 往 bind-mount 的 src 里 `cp` yaml 会 EACCES,配合 `restart: unless-stopped` 变成无限重启;(c) 你还不在 docker 组 |
| 3 | **把 entrypoint 自检里的 `[OK] Manus SDK` 当作 MANUS 可用** | 那行只判断 `[ -f /usr/local/lib/libManusSDK.so ]`,而该文件是无脑 `cp` 过去的。它只证明"文件拷到了",**不证明 `manus_ros2` 编译成功**。真正的验收是 `ros2 pkg list \| grep manus_ros2` |
| 4 | **原生 apt 装 ROS2 Humble + pinocchio** | noble 源里 `ros-humble-*` = 0 个;`python3-colcon-common-extensions` / `python3-rosdep` / `ros-kilted-pinocchio` 在**没加 ROS apt 源**时全都 `Unable to locate package`,而 `apt install -y` 遇到一个定位不到的包会**整条命令原子失败,一个包都装不上**。另外 `requirements.txt` 漏了 `pinocchio`(`robot.py:6` 硬依赖),`check_environment.sh` 的 required 列表也没查它 → 假通过,炸在后面 |
| 5 | **`ros2 run manus_data_publisher` 后靠 `ros2 topic hz` 或“grep 不到错误”判 license** | `static bool s_LicenseErrorShown` 闩锁:错误只打一次；未连接时又可能根本不打印。必须 `tee` 日志,同时检查错误和连接/标定/publish 正向记录 |
| 6 | **Hand 2 单关节测试打 `flat=5`,并给全部 20 个关节发 `position=0`** | (a) `flat=5` 是 `index_S2` = **食指侧摆**(行程仅 ±0.37),不是宣称的 MCP 屈伸(那是 `flat=4`);(b) 给 20 个关节都发 0 = **整手抢零位**,实测当前姿态离零位很远(拇指 -0.229、小指 +0.441),enable 后第一帧就是大幅运动 —— 正好是它声称要避免的;(c) 完全删掉了参考实现里的 `np.clip` 限位钳制,而这台手 `soft_limit_enabled = 0`,固件不挡越界 |
| 7 | **`git submodule update --init --recursive`** | 官方 `.gitmodules` 只声明了 `src/wujihandros2` 却没有对应 gitlink,`src/wuji-retargeting` 连声明都没有。这条命令 exit 0、无输出、什么都不创建 —— 给你一个假的成功信号 |
| 8 | **`conda run -n wuji2 python - <<'PY'`** | `conda run` 不转发 stdin 且默认吞 stdout:退出码 0、**零输出**。全程用绝对路径 `/home/mzsun/miniconda3/envs/wuji2/bin/python`,或加 `--no-capture-output` |
| 9 | **在 conda base 里跑任何 wuji 命令** | base 是 python 3.14 + 旧版 `wuji-sdk 2026.8.3`,且排在 PATH 最前。2026.8.3 之后有 breaking change(`RetargetSession` 从 `wuji_sdk.retargeting` 移到包顶层),混用必炸 |
| 10 | **新终端里直接执行 `cd $P/ros2_ws; colcon build`** | `$P` 不会跨终端保存；为空时会尝试进入 `/ros2_ws`。若 `cd` 后没有失败即停,colcon 会在当前目录误编整个仓库。必须用绝对路径、`set -e` 并先核对 `colcon list` |

---

## 5. 仍需你确认 / 提供的东西

| 项 | 状态 | 说明 |
|---|---|---|
| **MANUS dongle 的 SDK license** | ❌ 当前日志判定无效 | `3325:0049`(serial `082AC6A4`)硬件已识别，且当前仍能发布约 120 Hz 数据，但日志明确出现 `don't have a valid SDK Integrated license`。license 存在一个**必须全程插着 USB 的物理 key** 上；修复需按 MANUS 授权流程处理，不能把“当前有数据”误记为 license 通过。 |
| **MANUS SDK 从哪下** | ✅ 已有 3.2.0 | 官方分发包 `MANUS_Core_3.2.0_SDK_Linux`(约 867MB,含 Linux 版 Core Dashboard、C++/Python 示例、ROS2 包)。**不入库**(见 `.gitignore`),运行库用 `tools/fetch_manus_sdk.sh` 恢复。3.2.0 起 SDK 合并为单个分架构的 `libManusSDK-<arch>.so`(amd64 约 20.7MB),取代旧的两库共 261MB |
| **`.mcal` 个人标定** | ⚠️ 需要 Windows | 仓库自带通用 `LeftMetaglovePro.mcal` / `RightMetaglovePro.mcal`(各 ~3.8KB),**可以先用它跑通链路**,但那是别人的手型,精度明显下降。个人标定只能在 **Windows 的 MANUS Core** 里做完导出。文件名硬编码,必须精确覆盖同名文件 |
| **`1915:83fd` 无线收发器** | ❓ | 当前 `lsusb -d 1915:` 无输出。若你的 Metagloves Pro 靠它通信而非 dongle 直连,现在硬件不完整,会出现"手套枚举正常但骨架数据为空" |
| **手的序列号** | ✅ 已确认 | Hand 2:`WH2KA01260818006`(SN[3]=`K` → 右手;`A01` → Beta 2 硬件),IP `192.168.1.111`。一代手:`367C39563134`(sysfs `3-7`)/ `306735773434`(`3-8`) |
| **左手 Hand 2** | ❓ | 网上只发现**一只右手**。若有左手应在 `192.168.1.110` —— 是没上电/没接网线,还是根本没有?决定单手还是双手遥操 |
| **那两只一代 USB 手还要不要用** | ❓ | 只做 Hand 2 的话可以直接拔掉,也就完全不需要 wujihandcpp/wujihandros2/ROS2 那一整套。要用的话先解决它们的 `USB I/O error` |
| **`/home/mzsun/Projects/tianji_teleop/`** | ❓ 值得复用 | 你自己已经写过一整套 `hand_teleop.py` + `RetargetSession` + `keypoint_io.py`(UDP 关键点总线),注释里有 2026-08 的现场实测记录。在那套基础上接 MANUS 输入,可能比按官方 ROS2 方案重来工作量小得多 |
| **固件 v2.6.0 与 SDK 的配套** | ⚠️ | 官方文档写 Beta 2 当前 `v2.5.1`,你的手是 `v2.6.0`,更新。目前实测功能正常,遇到怪问题时版本配套是第一嫌疑 |

---

## 6. 验收 Checklist

### 宿主机(纯 Python,第 2 节)

```
[ ] pkill -x wuji-studio 后 pgrep 无输出
[ ] numpy 装上,wuji_sdk 显示 2026.8.31
[ ] ping 192.168.1.111 → 0% loss (~0.14ms)
[ ] scan 输出含  sn=WH2KA01260818006  type=WujiHand2  transport=UDP  addr=192.168.1.111:7447
[ ] wuji ping → Status=ok, Firmware=2.6.0
[ ] node_online = 16236015 → popcount 20
[ ] input_voltage 11–13V (实测 ~12.2),board_temperature 40–50
[ ] joint_diagnostics: error_code_current [0],ext_state {'Ready'}
[ ] 只读安全门:20 关节在线、error_code_current 全 0；未运行 enable/send
[ ] lsusb -d 3325: → Manus VR Sensor Dongle,节点 crw-rw-rw-
[ ] RetargetSession 自测:qpos (20,) float32
```

### 系统 ROS2 Jazzy(MANUS,第 3 节)

```
[ ] apt-cache policy ros-jazzy-ros-base  →  Candidate 不是 (none)
[ ] source /opt/ros/jazzy/setup.bash  →  ROS_DISTRO=jazzy
[ ] env_ros.sh 后 python3=/usr/bin/python3；colcon list 恰好三包；Summary: 3 packages finished / 0 failed
[ ] ros2 pkg list | grep manus  →  manus_ros2 / manus_ros2_msgs / manus_input_py 三行
[ ] ldd manus_data_publisher | grep manus  →  libManusSDK-amd64.so => ... (已解析,非 not found)
[ ] conda python 能同时 import yaml/rclpy/std_msgs/wuji_sdk/numpy
[ ] ./scripts/run_manus_wuji2.sh --dry-run 持续输出 20 维 qpos，方向正确且 clamped_now=0
[ ] 实体模式前已清空工作区，并明确输入 --control 与完整确认短语
```

### 运行时 ROS2 topic

| topic | 类型 | 期望 |
|---|---|---|
| `/manus_glove_0` | `manus_ros2_msgs/msg/ManusGlove` | ≈120 Hz,`side` 字段为 `right`,含 25 个 `raw_nodes` |
| `/manus_glove_1` | 同上 | 双手时才有;**编号是创建顺序不是左右**,靠 `side` 字段区分 |
| `/hand_input` | `std_msgs/msg/Float32MultiArray` | ≈120 Hz,`data` 长度恰好 **63** |

```bash
ros2 node list
ros2 topic list -t
timeout 12 ros2 topic hz /manus_glove_0
timeout 12 ros2 topic hz /hand_input
ros2 topic echo /manus_glove_0 --once --qos-reliability best_effort | head -20
```

> ⚠️ `ros2 topic echo` **必须带 `--qos-reliability best_effort`**,链路里所有 publisher 都是 BEST_EFFORT,否则会出现"订阅上了但一直没数据"的假故障。

### 日志行

```
[manus_data_publisher] Starting MANUS Data publisher!
[manus_data_publisher] MANUS Data publisher initialized.
[manus_data_publisher] MANUS data publisher is running in integrated mode.
[manus_data_publisher] Autoconnecting to the first host found.
[manus_data_publisher] Manus Core connected.
[manus_data_publisher] Calibration loaded successfully for Right glove (ID: N)
[manus_data_publisher] Glove ID: ..., publishes in the last 10 seconds: ~1200
[manus_wuji2] DRY-RUN mode: the physical hand will not be connected or commanded
[manus_wuji2] [dry-run] frames=N age=Nms qpos(rad) thumb=[...] ... clamped_now=0
[manus_wuji2] connecting to physical Wuji Hand 2 at 192.168.1.111:7447
[manus_wuji2] HAND ENABLED: sn=WH2KA01260818006, ...
[manus_wuji2] hand disabled
```

**绝对不能出现**:
```
It looks like you don't have a valid SDK Integrated license.   ← license 问题(或 .so 还是 LFS 指针)
Failed to initialize the SDK. Are you sure the correct ManusSDKLibary is used?   ← .so 加载失败
MANUS data publisher could not connect, trying again in a second.  (反复刷)   ← 卡在构造函数死循环,查 udev / dongle 是否被抢
```

### 分段定位断点

```
/manus_glove_0 无数据          → MANUS 侧(license / dongle 被抢 / 手套没配对)
有 glove 但 /hand_input 无数据  → manus_input_py 转换问题
run_manus_wuji2 导入失败         → 必须用启动器，确认 ROS overlay 与 wuji2 Python
已有关键点但 qpos 乱            → RetargetSession 输入约定不匹配(见步骤 8 的 ⚠️)
qpos 正常但手不动               → 没 enable / effort_limit 太低 / ext_state != 2
```

---

## 7. Blockers(按严重度)

### P0 — 会让整个方案作废

1. **MANUS dongle 的 license 必须含 `SDK`(Integrated)feature**。license 存在一个必须全程插 USB 的物理 key 上,写 `.lic` 只能在 **Windows 的 MANUS Core** 里做。当前机器上**看不到第二个 MANUS 设备**。license 不过关 → Linux 侧无解,只能找 MANUS。
2. **MANUS Core 本体在 Linux 上不存在**(官方原文:"Currently MANUS Core only runs on windows")。这意味着**配对、标定、固件升级、license 上传这四件事在这台机器上都做不了**。SDK 虽然暴露了 `CoreSdk_PairGlove` / `CoreSdk_GloveCalibrationStart` 等 API,但 `manus_data_publisher` **一个都没调**,它只会加载现成的 `.mcal`。
3. **旧的一代手参考链路已作废**:`wujihandpy` / `wujihandros2` / `wuji-retargeting` / `wuji-hand-teleop` 四个仓库 **2026-08-31 全部 archive**,且旧 launch 只支持一代手。Hand 2 没有官方 ROS2 驱动；本仓库现在用 `scripts/04_manus_wuji2.py` 直接把 ROS 关键点接入官方 `wuji-sdk`。不要再启动旧 launch。

### P1 — 会让步骤失败,但可绕过

4. ~~**Ubuntu 24.04 装不了 ROS2 Humble → 只能 Docker**~~ **已推翻**。noble 原生使用 Jazzy即可；MANUS SDK 的 glibc/libstdc++ 要求已满足。除消息和 rclcpp 外,构建还显式需要 `ament_index_cpp`、rosidl 生成器及 `libncurses-dev`,步骤 1 已列全。**不需要 Docker。**
5. **官方仓库缺 submodule gitlink**,`Dockerfile:189-190` 的两个 `COPY` 必挂,README 教的 `git submodule update --init --recursive` 是空操作。→ 本方案改用 `Wuji_Retargeting` 的 vendored 源码、只编 3 个包,绕开了这个问题。
6. ~~UID 不匹配 / 不在 docker 组~~ **已不适用**(不走 Docker)。
7. **colcon 必须使用系统 Python 且位于 `ros2_ws`**:本机新终端会自动激活 conda base；实测 CMake 因此选中 Python 3.14 并缺少 `em`/`catkin_pkg`。必须用修订后的 `env_ros.sh`,确认 `python3=/usr/bin/python3`。运行控制节点则必须使用 `scripts/run_manus_wuji2.sh`，它固定调用 `wuji2` Python。
8. **Git LFS 必须拉下来**:`.gitattributes` 把 `*.so` 全走 LFS。不拉 → `.so` 是 134 字节指针 → 要么链接失败,要么 entrypoint 静默 `touch COLCON_IGNORE` 跳过整个 MANUS,**日志只有一行 WARN**。

### P2 — 安全 / 精度 / 资源

9. **固件软限位未启用**:`soft_limit_enabled = 0`,`soft_pos_min/max` 全 0。**固件不会拦截超程位置指令**,越界保护 100% 靠软件。`RetargetSession` 不能代替下发前的逐关节 clamp；没有 watchdog、速率限制和异常失能时不得 enable。
10. **一代手当前是坏的**:`~/.wuji/logs` 里 2026-09-02 / 09-04 / 09-17 反复出现 `USB I/O error: transport I/O error: hardware fault or protocol violation`。即使想退回一代手方案,那两只现在也连不上。
11. **`.mcal` 个人标定缺失**:只能先用仓库自带的通用标定,手型贴合度差。
12. **磁盘**:2026-09-18 复核时根分区约 **93%**,剩余约 138G。`ros-jazzy-ros-base` + `ros-dev-tools` 约 1.5GB,当前仍够用；该数字会变化,部署前以 `df -h /` 为准。
13. **多网卡发现不稳**:4 个接口(`enp132s0` / `wlp131s0` / `tun0` VPN / docker bridge),`wuji doctor` 已警告 `multiple interfaces may cause routing issues`,studio 日志有 `Unable to join multicast group 224.0.0.224 ... Address already in use`。兜底:全部命令用 `--address 192.168.1.111:7447` 绕过组播发现;`tun0` 的 VPN 若推全局默认路由可能吃掉 `192.168.1.0/24`。
14. **设备独占**:`wuji-studio` / `wuji-hand-hmi` / Windows 端 MANUS Core / 另一个 `manus_data_publisher` —— 任何一个开着都会抢设备。
15. **无图形会话**:当前 `DISPLAY` 为空、`XDG_SESSION_TYPE=tty`(桌面在 `:1`)。Monitor GUI / rviz 开不了,但本方案纯 CLI,不受影响。另外官方 Monitor 的 "Hand only" 预设**没传 `enable_camera:=false`**,你这台没相机会持续报错刷屏 —— 反正也用不上。
16. **sudo 需要密码**(`sudo -n` 失败),所有 `usermod` / udev 步骤必须在交互终端手动敲。
