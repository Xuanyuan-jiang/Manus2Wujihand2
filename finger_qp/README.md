# 独立灵巧手触觉 QP 自适应关节调整插件

本目录提供一个可独立集成到其他机器人、控制框架或设备程序中的触觉 QP 插件：

```text
adaptive_finger_qp.py
```

插件只依赖 Python 标准库和 NumPy，不依赖 Isaac Sim、ROS、真机 SDK，也不引用 `DexTeleop-0` 的其他 Python 模块。复制这一个 `.py` 文件即可集成到别的工程。

## 1. 功能

插件根据以下输入计算关节位置调整量：

- 机器人手的 URDF；
- 当前各关节位置；
- 各指尖当前受力。

输出包括：

- 调整后的全部关节位置；
- 每个关节本次的调整量 `delta_q`；
- 当前生效的指尖；
- 每个指尖相对目标受力区间的误差；
- QP 求解迭代次数及目标函数值。

内部流程为：

1. 从 URDF 解析关节树、关节轴和关节限位；
2. 根据当前关节位置计算指尖正运动学和位置雅可比；
3. 计算超过目标受力区间的三轴力误差；
4. 将力误差转成期望指尖卸力位移；
5. 通过带 box 约束的 QP 求解各关节位置调整；
6. 同时满足 URDF 关节限位和单次最大调整量约束。

基础 QP 默认启用。多指 `Balance` 已包含在同一文件中，可通过参数启用。

## 2. 运行要求

- Python 3.10 或更高版本；
- NumPy。

安装 NumPy：

```bash
python3 -m pip install numpy
```

在 Piper 上已经使用 Python 3.10.12 和 NumPy 1.21.5 完成验证。

## 3. 单位与坐标系

| 数据 | 单位或约定 |
| --- | --- |
| 旋转关节位置 | 弧度 `rad` |
| 移动关节位置 | 米 `m` |
| 指尖力 | 牛顿 `N` |
| 力矩 | 牛顿米 `N·m` |
| URDF 长度 | 米 `m` |
| 默认力坐标系 | 对应指尖 link 的局部坐标系 |

三轴指尖力使用 `[Fx, Fy, Fz]`。力的正负方向必须与 URDF 指尖 link 的局部坐标轴一致。

如果只传入一个数字，例如 `8.0`，插件会将其解释为局部坐标系中的 `[0, 0, 8.0]`。

## 4. 最小 Python 示例

确保 `adaptive_finger_qp.py` 所在目录已经加入 `PYTHONPATH`，或者将文件复制到调用工程中。

```python
from adaptive_finger_qp import AdaptiveFingerQP

controller = AdaptiveFingerQP(
    "right_hand.urdf",
    tip_links={
        "thumb": "right_thumb_fingertip",
        "index": "right_index_fingertip",
        "middle": "right_middle_fingertip",
        "ring": "right_ring_fingertip",
        "pinky": "right_pinky_fingertip",
    },
)

# 键必须使用 URDF 中的独立可动关节名称，值为当前关节位置。
current_joint_positions = {
    name: read_current_joint_position(name)
    for name in controller.controlled_joint_names
}

# 默认认为力位于对应指尖 link 的局部坐标系。
fingertip_forces = {
    "thumb": [0.1, -0.2, 4.0],
    "index": [0.0, 0.1, 9.0],
    "middle": [0.0, 0.0, 6.0],
    "ring": [0.0, 0.0, 3.0],
    "pinky": [0.0, 0.0, 2.0],
}

result = controller.adjust(current_joint_positions, fingertip_forces)

# 可直接交给上层位置控制接口。
adjusted_joint_positions = result.joint_positions

# 本次相对输入位置的增量。
joint_adjustments = result.delta

send_joint_position_command(adjusted_joint_positions)
```

`AdaptiveFingerQP` 应在控制循环外创建一次，并在后续循环中重复使用。这样接触/释放迟滞和求解器 warm start 才能保持连续。

```python
controller = AdaptiveFingerQP(urdf_path, tip_links=tip_links)

while running:
    current_positions = read_joint_positions()
    current_forces = read_fingertip_forces()
    result = controller.adjust(current_positions, current_forces)
    send_joint_position_command(result.joint_positions)
```

需要清除接触状态和求解器状态时执行：

```python
controller.reset()
```

## 5. Sharpa 右手示例

Piper 当前右手 URDF：

```text
/home/pine/YUYAO/DexTeleop-0/real/scripts/SharpaWave_URDF_XML_USD_PINE_V3.0.4/src/right_sharpa_wave/right_sharpa_wave.urdf
```

初始化：

```python
from adaptive_finger_qp import AdaptiveFingerQP

urdf_path = (
    "/home/pine/YUYAO/DexTeleop-0/real/scripts/"
    "SharpaWave_URDF_XML_USD_PINE_V3.0.4/src/"
    "right_sharpa_wave/right_sharpa_wave.urdf"
)

controller = AdaptiveFingerQP(
    urdf_path,
    tip_links={
        "thumb": "right_thumb_fingertip",
        "index": "right_index_fingertip",
        "middle": "right_middle_fingertip",
        "ring": "right_ring_fingertip",
        "pinky": "right_pinky_fingertip",
    },
)
```

该 URDF 解析出的 22 个独立可动关节为：

```text
right_thumb_CMC_FE
right_thumb_CMC_AA
right_thumb_MCP_FE
right_thumb_MCP_AA
right_thumb_IP
right_index_MCP_FE
right_index_MCP_AA
right_index_PIP
right_index_DIP
right_middle_MCP_FE
right_middle_MCP_AA
right_middle_PIP
right_middle_DIP
right_ring_MCP_FE
right_ring_MCP_AA
right_ring_PIP
right_ring_DIP
right_pinky_CMC
right_pinky_MCP_FE
right_pinky_MCP_AA
right_pinky_PIP
right_pinky_DIP
```

左手使用相同方法，把 `right` 替换为 `left`，并加载左手 URDF。

## 6. 指尖力输入格式

### 6.1 简单三轴力

```python
fingertip_forces = {
    "index": [Fx, Fy, Fz],
}
```

### 6.2 只有法向力

下面会被解释为局部 `+Z` 方向 8 N：

```python
fingertip_forces = {
    "index": 8.0,
}
```

### 6.3 完整测量格式

```python
fingertip_forces = {
    "index": {
        "force": [0.2, -0.1, 8.0],
        "frame": "local",
        "contact_point_local": [0.0, 0.0, 0.006],
        "target_ranges": [
            [-5.0, 5.0],
            [-5.0, 5.0],
            [0.0, 5.0],
        ],
    }
}
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `force` | 三轴力 `[Fx, Fy, Fz]` |
| `frame` | `local`、`base` 或 `world`；`world` 在本插件中等同手基座坐标系 |
| `contact_point_local` | 接触点在指尖 link 局部坐标系中的位置；省略时使用 link 原点 |
| `target_ranges` | 当前指尖自己的三轴目标受力区间，可覆盖全局默认值 |

如果传入基座坐标系的力：

```python
fingertip_forces = {
    "index": {
        "force": [1.0, 0.0, 5.0],
        "frame": "base",
    }
}
```

## 7. 输出格式

`controller.adjust(...)` 返回 `QPResult`：

```python
result.joint_positions          # dict[str, float]，调整后的关节位置
result.delta                    # dict[str, float]，本次关节调整量
result.active_fingertips        # tuple[str, ...]，当前接触生效的指尖
result.force_errors_local_n     # dict，局部三轴力误差
result.solver_iterations        # QP迭代次数
result.objective                # QP目标函数值
result.to_dict()                # 转为可JSON序列化的字典
```

## 8. 启用多指 Balance

Balance会在基础 QP 目标上增加软约束，减少多指接触中的合力与合力矩残差。它默认关闭。

```python
from adaptive_finger_qp import AdaptiveFingerQP, QPConfig

config = QPConfig(
    enable_balance=True,
    force_balance_weight=1.0,
    torque_balance_weight=1.0,
    min_active_contacts_for_balance=2,
)

controller = AdaptiveFingerQP(
    "right_hand.urdf",
    tip_links=tip_links,
    config=config,
)

result = controller.adjust(current_joint_positions, fingertip_forces)
```

Balance 使用当前活跃指尖位置的中心作为力矩参考点，因此只用 URDF、当前关节位置和各指尖力即可运行，不要求仿真物体位姿。

启用后，同一个 QP 中会同时包含：

- 单指力超出目标区间后的卸力目标；
- 多指合力平衡目标；
- 多指合力矩平衡目标；
- 关节位置限位；
- 单次关节调整量限制。

## 9. 常用参数

```python
from adaptive_finger_qp import QPConfig

config = QPConfig(
    contact_threshold_n=0.5,
    contact_release_threshold_n=0.3,
    force_target_ranges_local_n=(
        (-5.0, 5.0),
        (-5.0, 5.0),
        (0.0, 5.0),
    ),
    force_gain_position_per_n_xyz=(
        0.0034906585,  # 0.2°/N
        0.0034906585,  # 0.2°/N
        0.0069813170,  # 0.4°/N
    ),
    max_joint_step=0.0087266463,  # 0.5°
    regularization=1.0e-3,
    max_solver_iterations=64,
    solver_tolerance=1.0e-7,
)
```

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `contact_threshold_n` | 0.5 N | 超过该力时进入接触状态 |
| `contact_release_threshold_n` | 0.3 N | 低于该力时释放接触状态 |
| `force_target_ranges_local_n` | X/Y ±5 N，Z 0～5 N | 不产生主动修正的目标受力区间 |
| `max_joint_step` | 0.5° | 每次调用允许的最大调整；旋转关节单位为弧度，移动关节单位为米 |
| `regularization` | `1e-3` | 抑制过大的关节调整 |
| `force_weight` | 1.0 | 基础力反馈目标权重 |
| `enable_balance` | `False` | 是否启用多指平衡 |
| `force_balance_weight` | 1.0 | 合力平衡权重 |
| `torque_balance_weight` | 1.0 | 合力矩平衡权重 |

如果传感器受力方向与期望调整方向相反，可调整：

```python
config = QPConfig(
    force_relief_direction_xyz=(-1.0, -1.0, -1.0),
)
```

不要在没有低速测试的情况下直接大幅提高 `max_joint_step` 或力反馈增益。

## 10. 关节限位和控制范围覆盖

默认控制 URDF 中全部独立可动关节，并使用 URDF 的 `<limit lower="..." upper="...">`。

仅控制指定关节：

```python
controller = AdaptiveFingerQP(
    "hand.urdf",
    controlled_joint_names=[
        "index_mcp",
        "index_pip",
        "index_dip",
    ],
)
```

覆盖关节限位和单步调整上限：

```python
controller = AdaptiveFingerQP(
    "hand.urdf",
    joint_limit_overrides={
        "index_mcp": [0.0, 1.2],
    },
    max_joint_step_overrides={
        "index_mcp": 0.004,
    },
)
```

URDF支持的关节类型：

- `fixed`
- `revolute`
- `continuous`
- `prismatic`
- 标准 URDF `mimic` 关节

不支持 `floating` 和 `planar` 关节。

## 11. JSON命令行接口

### 11.1 查看 URDF 信息

```bash
python3 adaptive_finger_qp.py \
  --urdf right_hand.urdf \
  --describe
```

输出会列出：

- 独立可动关节及顺序；
- 根 link；
- 叶子 link，可用于确认指尖 link 名称。

### 11.2 输入文件

`joints.json` 必须包含全部被控制关节：

```json
{
  "index_mcp": 0.20,
  "index_pip": 0.45,
  "index_dip": 0.30
}
```

`forces.json`：

```json
{
  "index_tip": [0.0, 0.0, 9.0]
}
```

如果 force 的键不是 URDF link 名称，可通过 `tip_links.json` 映射：

```json
{
  "index": "right_index_fingertip"
}
```

执行：

```bash
python3 adaptive_finger_qp.py \
  --urdf right_hand.urdf \
  --joints joints.json \
  --forces forces.json \
  --tip-map tip_links.json
```

启用 Balance 时，可提供 `config.json`：

```json
{
  "enable_balance": true,
  "force_balance_weight": 1.0,
  "torque_balance_weight": 1.0
}
```

```bash
python3 adaptive_finger_qp.py \
  --urdf right_hand.urdf \
  --joints joints.json \
  --forces forces.json \
  --tip-map tip_links.json \
  --config config.json
```

## 12. 接入其他设备或代码框架

集成层只需要完成三项适配：

1. 将设备当前关节位置整理为 `{URDF关节名: 位置}`；
2. 将触觉传感器输出整理为 `{指尖名: [Fx, Fy, Fz]}`；
3. 将 `result.joint_positions` 发送给设备的位置控制接口。

插件本身不打开串口、不订阅 ROS topic、不创建仿真对象，也不发送电机命令，因此可以嵌入：

- ROS/ROS 2节点；
- 独立 Python控制程序；
- 仿真控制循环；
- 网络控制服务；
- 厂商 SDK回调；
- 数据回放和离线分析程序。

## 13. 常见问题

### 提示缺少关节位置

```text
missing controlled joint positions
```

输入字典没有覆盖全部 `controller.controlled_joint_names`。可通过 `--describe` 或打印该属性查看完整列表。

### 提示找不到指尖 link

检查 `tip_links` 的值是否与 URDF 中的 link 名称完全一致，包括大小写。

### 有受力但关节没有调整

依次检查：

1. 力模长是否超过接触阈值；
2. 三轴力是否仍处于目标受力区间；
3. 受力坐标系是否正确；
4. 对应指尖雅可比在该方向是否有可控自由度；
5. 关节是否已经到达 URDF 限位；
6. `max_joint_step` 或 `force_weight` 是否被设为零。

### 调整方向相反

首先确认传感器报告的是“环境作用在手上的力”还是“手作用在环境上的力”。必要时修改 `force_relief_direction_xyz` 的符号。

## 14. 安全边界

此插件只提供运动学层面的关节位置修正，不替代以下设备安全功能：

- 电流和力矩限制；
- 速度、加速度限制；
- 自碰撞和环境碰撞检测；
- 通信超时保护；
- 急停；
- 驱动器故障处理。

首次接入新设备时，应在低速、低增益、可急停条件下逐指测试，并核对每个传感器坐标轴和每个关节的正方向。

## 15. 已完成验证

当前文件已经完成以下测试：

- Sharpa左手和右手 URDF均成功解析；
- 每侧22个可动关节和5个指尖均成功建立雅可比；
- 12 N测试受力能生成非零关节调整；
- 默认单次调整严格限制在0.5°以内；
- 输出严格限制在 URDF关节范围内；
- 受力释放后本次调整量归零；
- QP + Balance模式正常求解；
- 通用旋转关节与移动关节 URDF测试通过；
- Python API和 JSON命令行接口测试通过。
