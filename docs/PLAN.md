# mujoco_robot_cli 开发计划

状态：设计草案，可作为后续实现与验收依据；本次仅制定计划，尚未实现服务端或 CLI。

## 1. 目标与首版范围

提供一个长期运行的 `mujoco_server`，维护唯一的仿真环境；提供一个一次性请求程序 `mujoco_cli`，每次启动接收参数、请求服务、输出结果后退出。CLI 退出不影响仿真，多个 CLI 调用操作同一个环境，未来可以直接作为 agent tool 的执行入口。

首版覆盖九类能力：关节位置控制、末端位置控制、夹爪开合控制、查询关节位置、查询末端位置、查询夹爪状态、查询图像、设置红色物块位姿、恢复环境。

默认选择参考仓库的 **Archer Y6 单臂场景**。参考仓库还包含 E3 Desktop 双臂场景；用户尚未指定型号，此处作为可修改的设计假设。首版不同时实现双臂，不将自动抓取、运动规划、避障或实时图像流纳入必需范围。

“末端位置控制”支持 XYZ；姿态可选，省略时保持动作开始时的姿态。位置和姿态一起给出时执行完整位姿控制。

## 2. 已核实的参考实现

参考路径：`/home/blank/GitCode/hexfellow_github_backup/hex_driver_mujoco`。
参考提交：`46d7de2c07d2d0af64ae52f91975f5ec072358a5`。后续移植以此提交为基线，不能仅依赖本机绝对路径。

| 内容 | 来源与事实 | 对本项目的影响 |
| --- | --- | --- |
| 单臂场景 | `hex_driver_mujoco/mujoco_archer_y6/model/` | 复用 MJCF、STL、关节限位、惯性、摩擦、夹爪联动和执行器参数 |
| 桌面 | `scene.xml`：table 位姿为 `(0.5, 0, 0.3)`，绕 Z 旋转约 90° | 保留网格与变换，不能用普通方盒替代 |
| 红色物块 | body `box`，free joint `red_box_joint`，geom `red_box`；半尺寸 `(0.02, 0.02, 0.02)` m，质量 0.05 kg | 初始中心 `(0.5, 0, 0.5)`，自然下落；不能误认为这是落稳后的桌面坐标 |
| 绿色目标区 | `tar_plane`：中心 `(0.6, 0, 0.301)`，半尺寸 `(0.04, 0.04, 0.001)` m | 保留颜色、尺寸、接触行为和位置 |
| 机械臂基座 | 外层 body 位于 world `(0, 0, 0.25)` | 驱动 FK 输出与 world 坐标不能直接混用 |
| 末端 | `ee_frame` 相对 `grip_base_link` 平移 `(0.187, 0, 0)` | 作为首版 TCP 定义，查询与控制统一使用它 |
| 手部相机 | `robot.xml`：`end_camera`，局部位置 `(0.066, 0, 0.105)`、四元数 `(0.8660254, 0, -0.5, 0)`、fovy 50° | 仅调整相机朝向；位置、视场角、机器人动力学保持基线 |
| 图像 | 驱动固定 224×224，颜色渲染后转换为 BGR；深度转 uint16 毫米 | 对外规范 RGB 语义和文件编码，禁止重复交换颜色通道 |
| 时序 | 基类默认 state_rate=1000、cam_rate=30；单臂示例使用 500/30；仿真频率为 `max(2 × state_rate, 1000)` | 首版默认对齐单臂示例：状态 500 Hz、相机 30 Hz、仿真 1000 Hz |
| 控制 | `set_arm_pos_cmd`、`set_arm_pose_cmd`、`set_grip_pos_cmd`；仿真进程只对更新的命令时间戳计算控制量 | 一次请求必须转为服务端持续闭环执行，不能仅发送一次驱动命令 |
| 状态 | `get_arm_motor`、`get_arm_end`、`get_grip_motor`、`get_obj_pose` | 可参考数据组织方式，但必须统一坐标系、时间和快照 |
| 重置与物块设置 | `reset()` 只置事件；无公开物块设置接口；IK 失败仅打印 | 补全完成确认、结构化错误、物块设置和重置隔离 |
| 依赖 | Python ≥3.10、MuJoCo ≥3.3、OpenCV ≥4.10、三个 `hex_util_*` 包；参考仓库使用 uv | 保持同一技术栈，首个里程碑验证兼容版本并生成锁文件 |

以上结论来自静态源码检查；尚未运行参考场景，渲染效果、依赖可安装性与控制精度在 M0/M1 验证。

## 3. 场景与相机约定

### 3.1 场景复用

将 Archer Y6 的完整模型资源纳入本仓库，保留上游版权、许可证及来源记录。首次移植建立资源清单与散列；除相机朝向外，不主动调整场景几何、颜色、光照、接触、初始姿态或控制参数。必要的兼容修复单独记录。

MuJoCo、`hex_util_robot`、`hex_util_urdf` 等依赖的具体版本以实际安装和 smoke test 通过后的锁文件为准，不将参考仓库的宽松版本范围当作可复现环境。

### 3.2 手部相机“正方向”

这句话存在“画面正立”和“沿末端前向观看”两种可能含义。计划暂定为：**保留原来朝向夹爪工作区的光轴，仅校正画面滚转，使画面在约定观察姿态下正立**。沿 TCP +X 观看属于另一种安装方式，不能在未经说明的情况下改变原有视野。

实施步骤：

1. 按 MuJoCo 相机约定核实局部 -Z 为观察方向、+Y 为画面上方，计算当前光轴、上方向在 TCP/world 中的表达。
2. 选定可重复的桌面观察姿态，放置临时方向标记；记录原始图、相机坐标轴和修正图。
3. 计算校正四元数并修改 `end_camera` 的局部旋转；不先猜测为绕某轴 180°，也不使用返回图像时翻转来掩盖安装方向。
4. 验收画面上/右与标记一致、红色物块与绿色目标区在约定观察姿态可见、RGB 与深度对齐。临时标记不进入默认场景。
5. 在 `docs/camera.md` 固化最终光轴、上方向、局部四元数、参考姿态和对照图。如用户意图是沿 TCP 前向观看，在 M0 更新此约定后再实现。

相机默认启用 RGB；深度作为同一接口的可选类型。提供 GUI 调试与 headless 两种启动方式；headless 仍需可用的离屏渲染后端，优先验证 EGL，并记录 OSMesa 备选环境。

## 4. 服务架构与执行模型

首版采用 **HTTP + JSON，FastAPI/Pydantic + Uvicorn**。其请求响应、超时、错误和 schema 易于一次性脚本及未来 tool 使用；图像通过二进制响应返回。默认绑定 `127.0.0.1:8000`。

```text
mujoco_cli / future tool
         │ HTTP
         ▼
API：参数校验、request_id、deadline、结果编码
         │ 有界命令队列 / 完成结果
         ▼
单一仿真 worker：执行命令、持续控制、mj_step、渲染、发布快照
         │
         └─ MjModel / MjData / Renderer（仅 worker 访问）
```

- API 进程管理一个仿真子进程；Uvicorn 固定单 worker，禁用会重复启动仿真的自动 reload。
- 保留参考实现的独立仿真进程思路；首版由本项目 worker 管理移植后的模型和控制逻辑，避免依赖上游私有属性完成物块设置、相机配置和重置。
- 复用并核实参考控制算法及 `hex_util_robot` 的 FK/IK、重力/摩擦补偿。有效目标每个控制周期重新计算力矩；采用按名称解析的 joint/body/actuator 地址，避免复制硬编码 qpos 切片。
- API 不直接修改 `MjData`。worker 在步进边界处理命令，发布带版本的不可变状态快照；图像编码/网络传输不得长时间阻塞物理循环。
- 首版任何时刻最多一个修改环境的动作，包括关节、末端、夹爪、物块设置与 reset；竞争请求立即返回 `BUSY`。查询可并行，返回当前快照。
- 默认请求等待动作结束。内部使用 operation_id 关联命令、完成结果和截止时间；暂不增加完整的异步任务 API。
- 到位后继续保持目标；CLI 进程退出不会解除控制。超时后取消目标、将当前实测关节位置设为保持目标，返回最终状态，不允许后台继续追赶已超时目标。
- 请求取消需传递给 worker，只有 worker 确认取消/完成后才释放动作槽位。客户端断开后依靠已传入的 deadline 有界终止，不宣称断开等于立即停止。
- 启动必须等待模型、状态与已启用相机就绪；初始化失败应返回具体错误并退出，不能无限等待初始化事件。关闭时释放 viewer、renderer、IPC 和子进程。

## 5. 数据与动作契约

### 5.1 公共数据

- 单位：长度 m、角度 rad、速度 rad/s、时间 s；四元数顺序固定 `[w, x, y, z]`，表示局部坐标系到指定参考系的旋转。
- 对外末端和物块位姿默认 `frame="world"`；末端还支持 `base`。明确输出 frame，控制输入用同一约定。转换使用实际基座变换，不只在代码里随意补一个高度常量。
- 关节数组严格按 `joint_1` 至 `joint_6`；查询同时返回名称、位置、速度、限位。
- 每个快照包括 `epoch`（每次 reset 增加）、`step`、`sim_time_s`、`captured_at_ns`。动作截止时间使用单调时钟；重置仿真时间不影响超时。
- 每个请求携带/生成 `request_id`；动作返回目标、实际状态、误差和耗时。设置/重置的结果必须属于执行后的新快照。
- 查询默认返回最新有效快照，附带年龄；超出配置的新鲜度上限或尚无数据时返回 `NOT_READY`，不能返回全零伪状态。
- 拒绝 NaN/Inf、错误维度、非法坐标系、零四元数、越限目标；非零四元数在合理误差内归一化，否则拒绝。关节越限明确报错，不静默裁剪。

### 5.2 九类业务接口

| 能力 | HTTP 接口 | 请求要点 | 完成/返回语义 |
| --- | --- | --- | --- |
| 关节位置控制 | `POST /v1/arm/joints` | `positions[6]`、`timeout_s` | 持续控制至关节误差和速度连续满足阈值，返回实测值 |
| 末端位置控制 | `POST /v1/arm/pose` | `position[3]`、可选 `quaternion[4]`、`frame`、`timeout_s` | 先转换坐标并求 IK，再跟踪关节；按实际 TCP 位姿判定到位；不可达返回 `IK_FAILED` |
| 夹爪开合 | `POST /v1/gripper` | `action=open/close` 或 `opening_ratio∈[0,1]`，二选一 | 0=闭合、1=张开；通过模型标定映射到原始关节，返回实际开度和运动状态 |
| 查询关节 | `GET /v1/arm/joints` | 无 | 返回关节快照 |
| 查询末端 | `GET /v1/arm/pose?frame=world` | frame | 返回 `ee_frame` 实测位姿，并与模型世界坐标交叉验证 |
| 查询夹爪 | `GET /v1/gripper` | 无 | 返回原始关节角、归一化开度、目标、`open/closed/partial/moving/blocked` |
| 查询图像 | `GET /v1/camera/image?camera=end&type=rgb` | RGB 或 depth | RGB 为 PNG；depth 为 uint16 毫米 PNG；携带 epoch、step、时间和图像尺寸元数据 |
| 设置物块 | `POST /v1/object/pose` | world `position[3]`、可选 quaternion、`settle_s` | 设置 `red_box_joint` 位姿并清零其 6 维速度，更新派生状态；返回放置与可选落稳后的实际位姿 |
| 恢复环境 | `POST /v1/environment/reset` | 可选有界 `settle_s` | 恢复 home、控制器与物块，清理旧命令及缓存、递增 epoch，等待新状态/图像后返回 |

辅助接口：`GET /healthz` 提供进程存活信息，`GET /readyz` 提供仿真和渲染就绪信息，`GET /v1/capabilities` 返回协议版本、型号、相机、TCP、限位、阈值与单位。可增加 `GET /v1/object/pose` 以便放置后的验证。

### 5.3 动作完成与特殊情况

- 初始建议阈值：各关节位置误差 ≤0.01 rad、速度 ≤0.02 rad/s；末端位置误差 ≤0.005 m、姿态误差 ≤0.05 rad；连续保持 0.2 s。姿态误差使用四元数最短旋转角，正确处理 q 与 -q 等价。
- 这些数值是验收起点，需在 M2 用实际动力学验证并写入配置，不承诺未经测试的精度。轨迹通过限速插值生成；接口不承诺笛卡尔直线或无碰撞路径。
- 夹爪夹住物块时可能无法到达完全闭合位置；通过持续低速度、剩余误差和接触信息判定 `blocked`。动作可正常结束并报告该状态，但不得将其解释为抓取成功。
- 物块设置是环境编辑；只修改物块 free joint，不修改机械臂。默认零速度即时放置；`settle_s` 指额外仿真等待时间，必须受总 timeout 限制。检查明显穿透桌面和非法位置；允许合理高度释放。
- reset 采用与初次启动相同的初始化路径，清理控制目标、积分/滤波状态、qvel、ctrl、外力、求解器相关状态及旧缓存；忙时遵循 `BUSY`，首版不提供强制抢占。

### 5.4 返回结果与错误

成功 JSON 示例（字段值仅示意）：

```json
{
  "ok": true,
  "request_id": "req-001",
  "data": {
    "positions": [0, 0, 0, 0, 0, 0],
    "epoch": 0,
    "step": 1234,
    "sim_time_s": 1.234
  },
  "error": null
}
```

失败使用同一外层结构：`ok=false`、`data` 可包含最终实测状态、`error={"code":"TIMEOUT","message":"...","details":{...}}`。错误覆盖 `INVALID_ARGUMENT`（422）、`BUSY`（409）、`IK_FAILED`（422）、`TIMEOUT`（504）、`NOT_READY`（503）、`CAMERA_DISABLED`（409）、`INTERNAL_ERROR`（500）。未知异常输出到服务日志，响应不混入 traceback。

图片成功为二进制，失败仍为上述 JSON；相机内参、安装变换和深度单位通过 capabilities/相机元数据提供。未来跨机器调用不依赖服务端文件路径。

首版不自动重试修改请求，避免网络超时后重复动作。响应丢失时 CLI 明确报告结果未知，查询状态后决定下一步；request_id 仅用于关联，不宣称已有持久化去重能力。

## 6. 一次性请求脚本与未来 tool

使用一个公共客户端库和一个 CLI 入口覆盖全部命令，避免九份脚本各自实现网络、错误与参数解析。提供 `scripts/request.py` 薄入口以及安装后的 `mujoco_cli` 命令；两者走同一逻辑。

规划调用方式如下，均为未来接口示例，当前尚不可执行：

```bash
mujoco_server --config configs/default.yaml --headless
mujoco_cli joints set --positions 0 -1.5 3 0 0 0 --timeout 15
mujoco_cli joints get
mujoco_cli pose set --position 0.3 0 0.55 --frame world --timeout 15
mujoco_cli pose get --frame world
mujoco_cli gripper open
mujoco_cli gripper close
mujoco_cli gripper set --opening-ratio 0.5
mujoco_cli gripper get
mujoco_cli image get --camera end --type rgb --output artifacts/end.png
mujoco_cli object set --position 0.5 0 0.34 --quaternion 1 0 0 0 --settle 0.5
mujoco_cli environment reset --settle 0.5
python scripts/request.py joints get
```

示例坐标需在 M2 验证可达性及桌面高度，不能直接当作抓取轨迹。

- 公共参数：`--server`（默认 localhost）、`--timeout`、`--request-id`。动作 timeout 传给服务端，客户端连接/读取超时另留有限通信余量。
- 支持 `--request-json` 输入规范 JSON 请求，便于未来 tool 直接传结构化参数；与普通业务参数互斥。输入 schema 与 HTTP 共用。
- stdout 恰好输出一个 JSON 对象，日志写 stderr。所有错误包括解析错误、连接失败、超时也遵循 JSON；`--help`/版本输出为常规文本例外。
- 建议退出码：0 成功、2 参数/业务失败、3 连接失败、4 超时、5 服务端异常。业务错误详情继续通过稳定 `error.code` 区分。
- 图像由 CLI 下载并原子写入客户端路径；stdout 返回绝对路径、MIME、尺寸、时间和 epoch。提供可选 base64 输出用于无法读取本机文件的 tool，默认不把大图嵌入 JSON。
- CLI 不启动服务、不 reset 环境、不重复加载模型。未来 tool schema 根据同一 Pydantic 模型导出；本期不要求实现 MCP 等特定 tool 框架。

## 7. 建议目录

```text
pyproject.toml
uv.lock
configs/default.yaml
src/mujoco_robot_cli/
  server.py             # 服务入口与生命周期
  api.py                # HTTP 路由与错误映射
  schemas.py            # 请求、结果与元数据
  simulation/
    worker.py           # 唯一仿真所有者、IPC、状态快照
    controller.py       # 持续闭环、插值、到位判定、IK
    scene.py            # 名称解析、位姿变换、物块设置与 reset
    camera.py           # 渲染与相机元数据
  client.py             # HTTP 客户端
  cli.py                # 一次性命令
  assets/archer_y6/      # 完整 MJCF 与 STL，纳入 wheel 资源
scripts/request.py
schemas/                # 导出的 tool JSON schema
THIRD_PARTY_NOTICES.md
docs/PLAN.md
docs/api.md
docs/camera.md
docs/validation.md
tests/
```

配置至少包括模型、GUI/headless、渲染后端、仿真/状态/相机频率、分辨率、动作限速、到位阈值、超时和监听地址。运行时覆盖不能悄悄修改“参考场景一致”的默认配置。

## 8. 分阶段任务与验收

按 M0 → M1 → M2 → M3 → M4 → M5 实施；每阶段留下可重复命令和实际结果后再进入下一阶段。

### M0：确定场景、环境与接口基线

- [ ] 明确默认采用 Archer Y6，记录相机“正方向”的暂定解释及坐标约定。
- [ ] 在隔离环境安装参考依赖，运行原始单臂示例，记录 Python/MuJoCo/渲染后端/依赖版本。
- [ ] 保存原始场景截图、home 状态、关节限位、TCP/base/world 对照与资源清单；核实开合方向及原始夹爪比例，不能将驱动的 `1.33/1.52` 当作归一化开度。
- [ ] 固化 HTTP/CLI schema 和默认配置，生成锁文件。

验收：参考场景实际运行并能取图；基线报告能区分源码事实、设计选择与实测结果。

### M1：场景复用、worker 与相机修正

- [ ] 移植模型资源，记录来源与许可，使用包资源路径加载。
- [ ] 实现 worker 生命周期、名称索引、GUI/headless、状态快照、就绪/启动失败超时。
- [ ] 校正相机方向，交付原图/修正图与坐标轴验证。
- [ ] 检查构建后的 wheel 包含全部资源，在非仓库工作目录可启动。

验收：默认场景与基线一致，唯一预期视觉差异是相机朝向；GUI 与已声明支持的 headless 后端均可运行、取图并干净退出。

### M2：控制与环境操作

- [ ] 实现持续关节/末端/夹爪控制、保持、限速和到位检测。
- [ ] 完成 world/base 转换，模型 TCP 与 FK 交叉验证，结构化返回 IK 失败。
- [ ] 实现物块设置、完整 reset、epoch 和缓存失效。
- [ ] 验证 timeout 后停止追踪旧目标以及夹爪 blocked 状态。

验收：九类底层能力可直接通过 Python 调用；控制单次提交后持续收敛；重复 reset 与初次初始化一致；无通过瞬移机械臂伪造到位的逻辑。

### M3：HTTP 服务

- [ ] 实现九类接口及辅助接口、统一错误、二进制图像。
- [ ] 实现动作互斥、deadline、worker 异常传播与新鲜度检查。
- [ ] 输出 OpenAPI 与示例请求；覆盖忙时查询、并发修改、渲染失败和初始化失败。

验收：外部进程可完成全部请求；成功控制响应意味着实测到位或明确返回夹爪 blocked；错误不导致服务挂死或释放仍在执行的动作槽位。

### M4：一次性 CLI 与 tool 契约

- [ ] 实现全部子命令、公共客户端和 `scripts/request.py`。
- [ ] 实现参数/JSON 输入、stdout/stderr 约定、退出码、图片落盘/base64。
- [ ] 导出 JSON schema，补充安装、启动、命令与错误示例。

验收：每个命令独立进程调用后退出；连续多次调用保持同一仿真状态；只依赖参数、JSON 和退出码即可编排；服务器未启动时有界失败。

### M5：端到端验证与交付

- [ ] 自动化验证：关节设定→查询、末端设定→查询、夹爪开合→查询、取图、设置物块→实际位姿、reset→基线。
- [ ] 失败验证：维度/单位约定、越限、非法四元数、不可达目标、相机禁用、服务不可达、deadline、并发修改、worker 崩溃。
- [ ] reset 后检查无旧 epoch 图像/状态返回、旧目标不会恢复；验证图像尺寸/通道、RGB/深度对齐和 TCP 坐标变换。
- [ ] 进行多轮调用及重置 smoke test，检查进程与资源释放；记录实测延迟、控制误差、实际仿真速率和运行环境。
- [ ] 提供一个只通过 CLI 串联的桌面交互示例：reset→取图→设置物块→移动到可达观察位→开合夹爪→再次取图。示例不宣称具备自主抓取能力。

验收：在干净环境按 README 可复现，九类请求均有成功与代表性失败结果；已知限制和相机对照图完整。

## 9. 实施时优先处理的风险

1. **持续控制**：参考示例依赖反复发送新时间戳；本项目必须把目标保持与力矩更新移入控制周期，先用有代表性的关节运动验证。
2. **坐标与 IK**：模型基座高于 world 原点，驱动末端状态来自 URDF FK；在开放末端接口前完成变换与模型对照，避免隐蔽的 0.25 m 偏差。
3. **相机含义**：“正方向”最终以 M0/M1 的轴定义和对照图落实；当前暂定解释不能当作用户已确认的方向。
4. **重置隔离**：仅还原 qpos 不足以恢复环境；控制状态、时间、命令和图像缓存必须一起重建或清理。
5. **可复现环境**：依赖安装、URDF 版本、离屏 OpenGL 与 wheel 资源必须实测；源码可读不代表当前机器能运行。

首个可用交付点为 M4；完整交付包含 M5 的验证证据。后续如扩展 E3 双臂，可在业务 schema 增加 arm/camera 标识并复用同一 CLI/服务契约。
