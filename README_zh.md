# 仿真控制策略节点

[English](README.md) | [简体中文](README_zh.md)

独立 Forge Tool 策略节点，为 `robot-control-sim` 提供仿真场景控制：
在 `sim.control` Endpoint 上暴露机器人元数据查询和带延迟归位的场景复位 Action。

## Node、Endpoint 与 Tool API

| 项目 | 当前实现 |
| --- | --- |
| 生产 Node | 1：`sim_control_policy`；源码命令 `forge-sim-control-policy`，打包入口 `dist/sim_control_policy` |
| Tool 接入方式 | Dora `ToolMessage` provider；通过 Gateway 暴露调用入口 |
| 直接 HTTP Tool API | 0；本 Node 不监听 HTTP，调用地址属于 Gateway |
| Endpoint 数 | 1：`sim.control` |
| operation 数 | 2：`get_robot_info`（Query）、`reset`（Action，不可取消、可查状态；各自最大并发 1） |
| Gateway Tool 数 | 由 `tools.specs` 决定；robot-control-sim 为本 Endpoint 配置 2 个 Tool，见下表 |

| Tool ID（robot-control-sim 配置） | Endpoint / operation | provider Node |
| --- | --- | --- |
| `sim.get_robot_info` | `sim.control / get_robot_info` | `sim_control_policy` |
| `sim.reset` | `sim.control / reset` | `sim_control_policy` |

`reset` 经 `sim_command` 复位场景；配置 re-home 时，本 Node 同时作为 Tool caller，
经 `tool_request/tool_response` 调用 Gateway 中已配置的 Motion/Gripper Tool。
这些被调用的 operation 属于其他 provider，不能计入本仓库的 Endpoint/operation 数。
ToolSpec 的 `implementation_id` 是部署绑定标识，可不同于包名或 Node ID；以实际 profile 为准。

完整契约见 [TOOL_CONTRACTS.md](TOOL_CONTRACTS.md)，descriptor 见
[info.py](src/sim_control_policy_node/info.py)，reset 编排见
[control.py](src/sim_control_policy_node/control.py)。

兼容性：当前依赖声明为 `dora-rs==0.4.1`、`forge-msgs==1.0.1`；以 [pyproject.toml](pyproject.toml) 与 [uv.lock](uv.lock) 为准。 Tool 协议实现为仓库内 vendored 代码。 这些声明不构成跨版本互操作保证；使用制品时还需核对 build revision 和 Skill lock。

后续修改 Node 入口、Endpoint 或 Tool API 时，请同步更新本节及对应配置/契约，遵循 [Node README 规范](https://github.com/PhyAgentOS/forge/blob/master/CONTRIBUTING.md#node-readme-contract)。

## Tool 契约

| tool_id | implementation_id | endpoint_id | operation | 语义 |
| --- | --- | --- | --- | --- |
| `sim.get_robot_info` | `sim-control-policy` | `sim.control` | `get_robot_info` | query |
| `sim.reset` | `sim-control-policy` | `sim.control` | `reset` | action |

完整请求/响应、幂等性和错误码见 [`TOOL_CONTRACTS.md`](TOOL_CONTRACTS.md)。

## 设计目的

`move-arm-by-ee` 的运行链（gateway → motion policies → motion server → controllers → mujoco）
已经支持关节值与末端位姿运动。本仓库补充两项仿真控制能力，无需修改其他运行时节点：

- `get_robot_info`（Query）：返回校验后的机器人描述，包含关节名称/顺序、自由度、类型/限制、
  末端 site 和初始位姿，使 Agent 在运行时发现内置或用户提供的机器人，而非解析 SKILL.md。
- `reset`（Action）：经 `sim_command` 发送 `reset_scene`，并将机器人归位到 descriptor 的 `initial_qpos`。
  归位经 Gateway 公共调用通道完成：等待 `settle_ms` 后调用 `move_joints`/`set_opening`，
  聚合完成状态后才结束外层调用。因此归位模式下 `sim.reset` 的终态意味着场景和机器人都已回到初始状态，
  不会被 JTC 保持命令再次拉回。profile 不含 `reset:` 配置时，场景复位后即完成。

机器人差异通过声明配置表达；本节点读取 MJCF/URDF 资产旁的 `robot.yaml`，
增加同契约机器人无需修改本节点源码。

## 配置

```yaml
# profiles/<profile>/sim_control_policy.yaml
robot_descriptor_path: ../../robots/piper/robot.yaml  # relative to this file

# 可选归位阶段；省略时场景复位后即完成。
reset:
  motion:                       # 存在 reset 时必填
    tool_id: sim.set_joint_positions
    implementation_id: motion.integration
    endpoint_id: motion.server
    operation: move_joints
    arguments:
      group_name: piper_arm     # 并配置 velocity_scale / acceleration_scale
  gripper:                      # 可选；没有 Tool 夹爪时省略
    tool_id: sim.set_gripper_opening
    implementation_id: piper.gripper
    endpoint_id: gripper.controller
    operation: set_opening
  timeout_ms: 15000             # 归位总预算，默认值
  poll_interval_ms: 1000        # 结果轮询周期，默认值
  settle_ms: 500                # 运动前等待仿真状态传播，默认值
```

完整 schema 见 `tests/`；配置严格校验，拒绝未知键。

## Dora 接线

| 输入 | 用途 |
| --- | --- |
| `tool_in` | Gateway 发来的调用/注册消息 |
| `tool_response` | 内层归位调用的响应与事件 |
| `tick` | 驱动注册租约和归位计时器 |

| 输出 | 用途 |
| --- | --- |
| `tool_out` | 响应、租约注册/注销 |
| `tool_request` | 经 Gateway 公共调用通道发送内层 Tool 请求 |
| `sim_command` | 发给 MuJoCo 的 UTF-8 JSON `{"command": "reset_scene"}` |

## 开发

```bash
uv sync --frozen --all-groups
uv run pytest -q
uv run ruff check .
scripts/build_pyinstaller.sh   # single-file executable -> dist/sim_control_policy
```

`forge_tool` 内置于 `src/forge_tool/`；`sim_control_policy_node/__init__.py`
把它加入 `sys.path`，PyInstaller 下也适用。应与其他策略节点中的对应副本保持字节一致。

## 许可证

本仓库使用 Apache License 2.0，详见 [LICENSE](LICENSE)、[NOTICE](NOTICE) 与
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
