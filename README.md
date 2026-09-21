# Manus2Wujihand2

用 MANUS Metagloves 遥操作**一对 Wuji Hand 2**（左右双手，只做灵巧手，不含机械臂）。

## 链路

```
MANUS Metagloves (USB dongle 3325:0049)
  → manus_ros2            C++，链 libManusSDK_Integrated.so
  → /manus_glove_0 | _1   ManusGlove.msg，含 raw_nodes 骨架
  → manus_input_py        转换成 21 点 MediaPipe
  → /hand_input_right     Float32MultiArray，63 floats = 21×3，约 120 Hz
    /hand_input_left      两只手各发各的，互不阻塞
  → scripts/04_manus_wuji2.py --side right|left
        wuji_sdk RetargetSession.for_hand(HandModel.WujiHand2, <side>)
        → 20 个关节角（firmware order）
  → Zenoh
  → Wuji Hand 2
```

两只手各跑一个 `04_manus_wuji2.py` 进程，互相独立 —— 一只手 fail-safe 不影响另一只。

| | 序列号 | 地址 |
|---|---|---|
| 右手 | `WH2KA01260818006` | `192.168.1.111:7447` |
| 左手 | `WH2JA01260813009` | `192.168.1.110:7447` |

两台的 `handedness` 都已向设备本身核实过（`h.handedness()`），不是从序列号推断的。

Hand 2 走 **RJ45 / 静态 IP / Zenoh**，不走 USB，因此对 Hand 本体零 udev 需求；
`config/99-manus-libusb.rules` 只用于 MANUS 手套的 dongle。

## 环境

| 组件 | 版本 / 说明 |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy，**装在系统里**（非 Docker、非 RoboStack） |
| Python | conda env `wuji2`，Python 3.12（与 Jazzy rclpy 同 ABI） |
| `wuji_sdk` | 2026.8.31，内置 Hand 2 的 retarget 配置与 URDF |
| MANUS SDK | **3.2.0**，闭源，`.so` 不入库，见下方「首次部署」 |

`wuji2` 与 ROS 2 用两套 shell 环境，互不混用：

- `source env.sh` —— 激活 conda `wuji2`，跑 `scripts/01..04`
- `source env_ros.sh` —— 把 conda 踢出 PATH，接系统 ROS 2，跑 `colcon build` 和 ROS 节点

两个脚本都从自身位置推导仓库根目录，仓库可以放在任意路径。
conda 前缀默认 `$HOME/miniconda3`，可用 `WUJI2_CONDA` 覆盖。

## 首次部署

```bash
# 1. 恢复 MANUS SDK 运行库（约 40 MB，不在 git 里）
#    来源是官方分发包的 "ROS2 package/ManusSDK/lib"
./tools/fetch_manus_sdk.sh /path/to/ManusSDK/lib

# 2. MANUS dongle 的 udev 规则
sudo cp config/99-manus-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# 3. 构建三个 ROS 2 包
source env_ros.sh
cd ros2_ws && colcon build --symlink-install \
  --packages-select manus_ros2_msgs manus_ros2 manus_input_py \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

> ⚠️ `-DPython3_EXECUTABLE=/usr/bin/python3` 不能省。CMake 的探测会挑中 conda 的
> `wuji2/bin/python3`，那个环境里没有 `empy`，`rosidl` 生成消息时会报
> `ModuleNotFoundError: No module named 'em'`。

完整的分步部署、验收点和排障表见 [DEPLOY.md](DEPLOY.md)。

## 运行

三个终端：

四个终端：

```bash
# A —— MANUS 取数
source env_ros.sh && ros2 run manus_ros2 manus_data_publisher

# B —— 转 21 点 MediaPipe（双手）
source env_ros.sh
ros2 run manus_input_py manus_input --config config/manus_input_both_hands.yaml

# C —— 双手（起两个独立进程）
./scripts/run_manus_wuji2.sh --side both
```

`--side both` 刻意起**两个进程**而不是一个进程管两只手：SDK 的连接/使能是阻塞调用，
同进程会让一只手卡住另一只的控制环；一只手 fail-safe 也会带走另一只。
脚本负责转发 Ctrl+C 并等两个子进程失能退出。

**一只手异常退出时另一只会继续运行**（遥操作中途停掉正常工作的那只，可能让它松开
已抓的东西），脚本会把这件事打出来。要全停就 Ctrl+C。

也可以分开起，那样能给每只手不同参数：

```bash
./scripts/run_manus_wuji2.sh --side right
./scripts/run_manus_wuji2.sh --side left --kp 3.0
```

只用一只手时，B 换成 `config/manus_input_right_only.yaml`。

**⚠️ C / D 默认就会连接并驱动实体手。** 只想看重定向数值而不动硬件，加 `--dry-run`：

```bash
./scripts/run_manus_wuji2.sh --side both --dry-run
```

## 安全

**这两只 Hand 2 的 `soft_limit_enabled = 0`，`soft_pos_min/max` 全为 0 —— 固件不拦截
超程位置指令，越界保护 100% 靠软件。**

### ⚠️ 默认即实体控制

`scripts/04_manus_wuji2.py` **默认就会连接、使能并驱动实体手**，`--dry-run` 才是
只打印不下发。早期版本反过来（默认 dry-run，实体控制需 `--control` 加确认短语）。

那道闸曾在三次故障中挡住实体动作：骨架取点错位一节、坐标系手性镜像、标定文件
手型写反。**它现在没有了**，所以换手套标定、改 `manus_input_py` 取点逻辑、升级
MANUS SDK 之后，务必先用 `--dry-run` 过一遍再直接跑。

剩下的保护都在运行时：

- 输入侧：BEST_EFFORT/depth=1、长度 63 校验、NaN/Inf 拒收、MediaPipe 米制尺度合理性检查
- 骨架侧：首帧自检腕→MCP 距离、MCP 展宽、指节长度与手性，不合常理即报错
- 输出侧：按 hand2 URDF 的 20 轴限位 clamp、每轴限速
- 时序：输入 watchdog、enable 后必须等到一帧新输入才下发
- 故障：按固件的四级 severity 分级处理 —— `Warning` 级（多为 AutoClear 的数据质量
  提示，如 `Enc1BitRate`）限流记录但不停机；`DeferredStop` / `ImmediateStop` /
  `Fatal` 以及无法识别的非零码一律 fail-safe
- 退出：所有异常与退出路径均失能并断开

> 早期版本把**任何**非零关节故障码都当致命错误，导致动作幅度稍大就停机 ——
> `Enc1BitRate`（编码器 bit 标志率偏高，`AutoClear`）在正常动作中会偶发，
> SDK 自己也只打 WARN。分级判断读 `WujiHand2.describe_error(code)["severity"]`，
> 不要自己解析故障码的 hex 位。

### 控制参数

默认值面向**实时跟随**，不是首次上电的保守值：

| 参数 | 默认 | 说明 |
|---|---|---|
| `--rate` | 120 Hz | 与 `/hand_input_*` 的发布率对齐，避免拍频 |
| `--max-speed` | 8.0 rad/s | 每轴限速。这是「跟随慢」的主因，不是 kp |
| `--kp` | 4.0 | 位置刚度（设备出厂值 1.5） |
| `--kd` | 0.02 | 阻尼。kp 抬高后 0.01 容易欠阻尼 |
| `--effort-limit` | 1.5 A | 官方建议上限 |
| `--watchdog` | 0.2 s | 120 Hz 下相当于连丢 24 帧 |

`--max-speed` 是限幅器：8 rad/s 时每 tick 允许走 3.8°，对正常手速几乎透明，
但仍能拦住手套跟丢造成的跳变。全行程（MCP 150°）约 0.33 s 走完。

调参方向：**发闷/跟不上** → 先抬 `--max-speed`，再抬 `--kp`；
**嗡嗡响/抖** → 降 `--kp` 或抬 `--kd`。

首次上电或换了标定，建议先用保守值试：

```bash
./scripts/run_manus_wuji2.sh --side right --max-speed 2.0 --kp 3.0 --kd 0.05 --effort-limit 0.5
```

## 已知问题

### 手套标定未完成（阻塞项）

摊平手时 MANUS 自算的 ergonomics 读数不合理 —— `IndexPIPStretch` 达 `+143°`
（人体极限约 110°），`ThumbMCPSpread` 达 `-46°`。MCP 一层正常，PIP/DIP 错得离谱。

**`.mcal` 文件修不好这一层。** 该文件只含手型几何（`cmcPosition`、`fingerLength`、
`wristPosition`、`wristRotationOffset`），不含弯曲传感器的原始值→角度映射；
后者在 MANUS Core 的用户配置里。必须用 MANUS Core Dashboard 重跑标定流程。

当前生效的是**官方默认标定**（别人的手），几何自洽但尺寸不对，只够验证链路。

两道校验：

```bash
python3 tools/check_mcal.py ros2_ws/src/manus_ros2/calibration/RightMetaglovePro.mcal
source env_ros.sh && python3 tools/check_glove_live.py    # 摊平手
```

> 曾有一份标定文件 `side` 标着 `right`、几何却是**左手**，外加 `141.5°` 的腕旋转偏移。
> `CoreSdk_SetGloveCalibration` **不校验**文件内几何属于哪一侧，静默接受后
> 下游表现为关节角离谱、手指朝腕部折回，极难追溯。`tools/check_mcal.py` 就是为
> 挡住这类文件写的 —— 每次标定完都跑一遍。

### 上游 `JointTypeToString` 错位（已在本仓库修正）

MANUS 官方 ROS 2 包（含 3.2.0）把 SDK 按**骨头**命名的 `FingerJointType` 翻译成按
**关节**命名的字符串时整体错了一位：

| SDK 枚举（骨头） | 节点实际位置 | 上游标成 | 应该是 |
|---|---|---|---|
| `_Metacarpal` | 掌骨根 ≈ 腕 | `"MCP"` | 腕/掌骨根 |
| `_Proximal` | 近节指骨根 | `"PIP"` | **`"MCP"`** |
| `_Intermediate` | 中节指骨根 | `"IP"` | **`"PIP"`** |
| `_Distal` | 远节指骨根 | `"DIP"` | `"DIP"` ✓ |
| `_Tip` | 指尖 | `"TIP"` | `"TIP"` ✓ |

骨骼绑定中节点位于骨头**根部**，「近节指骨的根」就是 MCP 关节。照字面取点会让每根
手指变成 `[掌骨根, MCP, PIP, DIP]`，而 MediaPipe 要 `[MCP, PIP, DIP, TIP]` ——
错一节、指尖丢失，retarget 输出四指同向外摆。

本仓库两处修正：`manus_ros2` 直接输出骨头名；`manus_input_py` 按 `(chain_type, 骨头)`
选点，并带旧标签兼容表。**升级 MANUS SDK 时这两处必须一起带走**，上游至今未修。

拇指链是 `[Metacarpal, Proximal, Distal, Tip]` —— 它没有中节指骨，唯一的 IP 关节在
远节指骨根部。`ManusSDKTypes.h` 里 `//thumb doesn't have it` 那句注释标在了 `Distal`
行上，是标错了行。

### 坐标系手性

`manus_input_py` 的 `_node_position` **不做** y 取反。这一处曾被反复改错两次，
根因都是早期判据在摊平的手上退化（拇指离掌面不足 1 mm，符号是浮点噪声）。
现用判据是：

```
u    = 腕 → 中指MCP            r = 小指MCP → 食指MCP
curl = 中指尖相对中指MCP、垂直于 u 的分量（屈曲时指向掌心）
值   = dot(curl, normalize(cross(u, r)))
```

**屈曲姿态下右手为负、左手为正**，大小对称（±6.82e−02 @ 45° 屈曲），基准取自
Hand 2 的 `right.urdf` / `left.urdf`。`manus_input` 启动时按手打印实测值并与
`msg.side` 比对，不符即报错。

手指伸直时该判据**退化**（掌面法向不确定），此时它报「无法判定」而不是给结论 ——
判手性必须弯曲手指。`tests/test_skeleton_mapping.py` 固化了左右两组基准点。

## 仓库结构

```
config/          manus_input 配置（单手/双手）、MANUS dongle udev 规则
scripts/         01..03 为 SDK 连通性检查，04 为主遥操节点
ros2_ws/src/
  ManusSDK/      MANUS SDK 头文件与运行库（lib/ 不入库）
  manus_ros2/    官方 ROS 2 节点 + 本地补丁，含 calibration/*.mcal
  manus_ros2_msgs/
  manus_input_py/  MANUS 骨架 -> 21 点 MediaPipe（非官方，见下）
tests/           实录 /hand_input 帧与取点逻辑测试
tools/           SDK 恢复脚本、标定与骨架检查工具
```

## 第三方代码来源

`ManusSDK/`、`manus_ros2/`、`manus_ros2_msgs/` 取自 **MANUS Core 3.2.0 SDK for Linux**
官方分发包的 `ROS2 package/`。

`manus_input_py/` 不是官方的：它来自 `wuji-technology/wuji-hand-teleop`
commit `3fa58481e971bf1588b57751ea44d99abe1d95b5`（经 `Fanlinfeng23/Wuji_Retargeting`
转手），官方 main 分支已删除该包。

vendored 代码首次提交为**逐字未改的副本**，此后所有本地修改都是独立提交 ——
`git log -p ros2_ws/src/` 即为相对上游的完整改动清单。`manus_ros2` 里的本地改动
都标了 `LOCAL PATCH` 注释，共两处：

1. `JointTypeToString` 输出骨头名（修上游的错位 bug，见「已知问题」）
2. `.mcal` 自动加载 —— 上游 3.2.0 删掉了这个功能，改由 MANUS Core 持有标定。
   本项目保留它，这样遥操作时不必开着 Core。

`ManusSDK/` 下的头文件与运行库版权归 MANUS 所有，使用需持有带 `SDK` feature 的
MANUS license key。官方分发包本身不入库（见 `.gitignore`）。
