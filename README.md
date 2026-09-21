# Manus2Wujihand2

用 MANUS Metagloves 遥操作 **Wuji Hand 2**（只做灵巧手，不含机械臂）。

## 链路

```
MANUS Metagloves (USB dongle 3325:0049)
  → manus_ros2            C++，链 libManusSDK_Integrated.so
  → /manus_glove_0 | _1   ManusGlove.msg，含 raw_nodes 骨架
  → manus_input_py        转换成 21 点 MediaPipe
  → /hand_input           Float32MultiArray，63 floats = 21×3，约 120 Hz
  → scripts/04_manus_wuji2.py
        wuji_sdk RetargetSession.for_hand(HandModel.WujiHand2, Right)
        → 20 个关节角（firmware order）
  → Zenoh 192.168.1.111:7447
  → Wuji Hand 2
```

Hand 2 走 **RJ45 / 静态 IP / Zenoh**，不走 USB，因此对 Hand 本体零 udev 需求；
`config/99-manus-libusb.rules` 只用于 MANUS 手套的 dongle。

## 环境

| 组件 | 版本 / 说明 |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy，**装在系统里**（非 Docker、非 RoboStack） |
| Python | conda env `wuji2`，Python 3.12（与 Jazzy rclpy 同 ABI） |
| `wuji_sdk` | 2026.8.31，内置 Hand 2 的 retarget 配置与 URDF |
| MANUS SDK | 闭源，两个 `.so` 不入库，见下方「首次部署」 |

`wuji2` 与 ROS 2 用两套 shell 环境，互不混用：

- `source env.sh` —— 激活 conda `wuji2`，跑 `scripts/01..04`
- `source env_ros.sh` —— 把 conda 踢出 PATH，接系统 ROS 2，跑 `colcon build` 和 ROS 节点

两个脚本都从自身位置推导仓库根目录，仓库可以放在任意路径。
conda 前缀默认 `$HOME/miniconda3`，可用 `WUJI2_CONDA` 覆盖。

## 首次部署

```bash
# 1. 恢复 MANUS SDK 运行库（约 250 MB，不在 git 里）
./tools/fetch_manus_sdk.sh /path/to/manus/sdk/lib

# 2. MANUS dongle 的 udev 规则
sudo cp config/99-manus-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# 3. 构建三个 ROS 2 包
source env_ros.sh
cd ros2_ws && colcon build --packages-select manus_ros2_msgs manus_ros2 manus_input_py
```

完整的分步部署、验收点和排障表见 [DEPLOY.md](DEPLOY.md)。

## 运行

三个终端：

```bash
# A —— MANUS 取数
source env_ros.sh && ros2 run manus_ros2 manus_data_publisher

# B —— 转 21 点 MediaPipe
source env_ros.sh
ros2 run manus_input_py manus_input --config config/manus_input_right_only.yaml

# C —— retarget（默认 dry-run，不连手、不下发）
./scripts/run_manus_wuji2.sh
```

## 安全

**这台 Hand 2 的 `soft_limit_enabled = 0`，`soft_pos_min/max` 全为 0 —— 固件不拦截超程
位置指令，越界保护 100% 靠软件。**

`scripts/04_manus_wuji2.py` 因此默认 dry-run，且实体控制需要显式确认短语。已实现的安全门：

- 输入侧：BEST_EFFORT/depth=1、长度 63 校验、NaN/Inf 拒收、MediaPipe 米制尺度合理性检查
- 输出侧：按 hand2 URDF 的 20 轴限位 clamp、每轴默认 0.6 rad/s 限速
- 时序：0.25 s 输入 watchdog、enable 后必须等到一帧新输入才下发
- 退出：所有异常与退出路径均失能并断开

首次上电建议保持默认的低限幅参数 `kp=3.0`、`kd=0.05`、`effort_limit=0.5 A`。

## 已知问题

**`/hand_input` 骨架整体错位一节（未修）。** 后果是 retarget 输出让除拇指外四指
同向外摆（abd 约 +20/+13/+10/+10 度）。

根因在 `manus_ros2/src/ManusDataPublisher.cpp` 的 `JointTypeToString`：MANUS SDK 的
`FingerJointType` 按**骨头**命名，该函数翻译成按**关节**命名的字符串时整体错了一位。

| SDK 枚举（骨头） | 节点实际位置 | 代码标成 | 应该是 |
|---|---|---|---|
| `_Metacarpal` | 掌骨根 ≈ 腕 | `"MCP"` | 腕/掌骨根 |
| `_Proximal` | 近节指骨根 | `"PIP"` | **`"MCP"`** |
| `_Intermediate` | 中节指骨根 | `"IP"` | **`"PIP"`** |
| `_Distal` | 远节指骨根 | `"DIP"` | `"DIP"` ✓ |
| `_Tip` | 指尖 | `"TIP"` | `"TIP"` ✓ |

骨骼绑定中节点位于骨头**根部**，「近节指骨的根」就是 MCP 关节。结果每根手指发布的
4 个点是 `[掌骨根, MCP, PIP, DIP]`，而 MediaPipe 要 `[MCP, PIP, DIP, TIP]`，指尖从未发布。

> ⚠️ `manus_input_py` 里的 `_convert_to_mediapipe_semantic` 看似是条正确的后备路径，
> 但它正是按这些已被错误标注的 `joint_type` 字符串选点的，**改走语义路径修不好**。

检查方式：

```bash
# 几何判据（离线跑已抓好的帧，或 --live 直接订阅）
python3 tools/check_hand_input.py tests/data/hand_input_straight.yaml --retarget

# 手套节点拓扑与每段骨长（需先 source env_ros.sh）
python3 tools/dump_manus_nodes.py
```

这个错误姿态在 ±40° 限位之内，**clamp 拦不住**。修复前请勿使用 `--control`。

`tests/data/` 下的两帧实录可复现此问题。注意它们抓于右手标定更新之前。

## 仓库结构

```
config/          manus_input 配置、MANUS dongle udev 规则
scripts/         01..03 为 SDK 连通性检查，04 为主遥操节点
ros2_ws/src/     三个 MANUS ROS 2 包（vendored，见下）
tests/data/      实录 /hand_input 帧
tools/           MANUS SDK 恢复脚本、骨架错位检查工具
```

## 第三方代码来源

`ros2_ws/src/` 下的 `manus_input_py`、`manus_ros2`、`manus_ros2_msgs` 取自
`wuji-technology/wuji-hand-teleop` commit `3fa58481e971bf1588b57751ea44d99abe1d95b5`，
经 `Fanlinfeng23/Wuji_Retargeting` 转手。

首次提交为**逐字未改的 vendored 副本**，此后所有本地修改都是独立提交 ——
`git log -p ros2_ws/src/` 即为相对上游的完整改动清单。

`ManusSDK/` 下的头文件与运行库版权归 MANUS 所有，按其授权条款使用。
