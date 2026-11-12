# API 与 CLI

服务默认监听 `127.0.0.1:8000`。所有 JSON 响应使用：

```json
{"ok": true, "request_id": "...", "data": {}, "error": null}
```

失败时 `ok=false`，`error.code` 为 `INVALID_ARGUMENT`、`BUSY`、`IK_FAILED`、`TIMEOUT`、`NOT_READY`、`CAMERA_DISABLED` 或 `INTERNAL_ERROR`。

所有对外绝对 pose（机械臂、物块、动作返回的 state、目标区位置）均以机械臂 `base_link` 为基准，`frame="base"`，位置单位米、角度单位弧度、四元数顺序 wxyz。`world` 输入返回参数错误。默认场景的 base 在 world `[0,0,0.25]`，因此旧 world 位置迁移到 base 时 Z 减 0.25；实现使用实际基座变换，也支持基座旋转。

## HTTP

- `POST /v1/arm/joints`：`{"positions":[6个弧度],"timeout_s":15}`
- `GET /v1/arm/joints`
- `POST /v1/arm/pose`：`{"position":[x,y,z],"quaternion":[w,x,y,z],"frame":"base"}`；frame 可省略；省略姿态时保持当前姿态。
- `GET /v1/arm/pose`：返回 base 下的 TCP 实测位姿，仅接受 `frame=base`。TCP 原点对标夹爪闭合时两指尖之间的接触位置：`+X` 指向指尖前方，`+Y` 为开合方向。张开时指尖离开该点，位姿原点不随开合移动。
- `POST /v1/arm/pose/delta`：`{"delta_position":[dx,dy,dz],"delta_rpy":[roll,pitch,yaw],"timeout_s":15}`；两项增量默认均为零。以仿真线程开始执行时的 TCP 坐标系为基准：+X 沿闭合指尖的前方，+Y 为夹爪开合方向，+Z 向上；绕 +X/+Y/+Z 分别为 roll/pitch/yaw，按右手规则，角度单位弧度。目标为 `T_base_tcp × Trans(delta_position) × Rz(yaw) × Ry(pitch) × Rx(roll)`；平移按开始时的朝向计算，旋转局部右乘。每次调用执行一次相对移动，连续调用可实现类似飞机/底盘的自身坐标系操控。到位、阻塞、IK 失败、超时语义与绝对 pose 一致。
- `POST /v1/gripper`：`{"action":"open"}`、`{"action":"close"}` 或 `{"opening_ratio":0.5}`。
- `GET /v1/gripper`
- `GET /v1/camera/image?camera=end|opposite|left&type=rgb|depth`：手部、桌对面或左侧固定相机，成功返回 PNG；深度为 uint16 毫米。省略 camera 时使用手部相机。
- `POST /v1/camera/point`：最简请求为 `{"camera":"end","pixel":[u,v],"depth_m":z}`，相机必填；相机内参和外参由服务器读取。省略末端位姿时，`end` 使用当前 TCP 位姿计算，`opposite` 和 `left` 使用各自固定相机位姿计算。可同时传入 `tcp_position_base` 和 `tcp_quaternion_base` 指定计算所用的末端位姿。需按特定图像帧精确反投影时，仍可传入该帧响应头提供的完整采集元数据（末端采集位姿、相机相对 TCP 外参和内参），此时不读取当前位姿或相机帧。返回 `base_link` 下 XYZ（米）。
- `GET /v1/objects`：返回三个物块的位姿和目标区状态，`data.objects` 以 `red`、`orange`、`yellow` 为键。
- `POST /v1/object/pose`：通过请求体的 `object_id` 选择物块，例如 `{"object_id":"orange","position":[0.5,0.12,0.11]}`；设置 base 坐标和可选姿态，仅支持 `frame="base"`（可省略）。
- `GET /v1/object/pose?object_id=yellow`：查询指定物块的 base 位置和姿态。
- `GET /v1/object/in-target?object_id=orange`：查询指定物块中心是否位于绿色目标区水平范围内，并已落在桌面高度。
- `POST /v1/environment/reset`
- `GET /healthz`、`GET /readyz`、`GET /v1/capabilities`

物块 `object_id` 可选 `red`（红）、`orange`（橙）、`yellow`（黄），省略时默认 `red`；非法值返回 422。物块边长均为 0.04 m、质量均为 0.05 kg，初始 world 位置分别为 `[0.5,0,0.5]`、`[0.45,0.17,0.5]`、`[0.45,-0.17,0.5]`。重置会恢复全部物块。动作返回的 `state.objects` 包含全部物块，原 `state.object` 保留为红色块；每个物块状态包含 `object_id`。`GET /v1/capabilities` 的 `objects` 列出支持的标识。

修改请求同时只允许一个。状态和图像通过带进程锁的共享内存最新快照传递；发布时覆盖旧数据，读取不消费数据，也不依赖客户端是否曾经查询。每次查询复制读取时最新发布的完整快照，业务字段与 `epoch`、`step`、时间戳来自同一份数据。超过新鲜度上限返回 `NOT_READY`。

“最新”指最近一次采样并发布的数据：默认状态发布 500 Hz、相机 30 Hz，实际频率受物理步进和渲染耗时影响；并非每次请求都会重新采样或渲染。指令及执行结果仍使用队列传递。

## CLI

```bash
mujoco_server --config configs/default.yaml --headless
mujoco_cli joints set --positions 0.1 -0.2 0.4 -0.2 0.1 0.05 --timeout 15
mujoco_cli pose get
mujoco_cli pose delta --delta-position 0.01 0 0 --delta-rpy 0 0 0.1
mujoco_cli image point --camera end --pixel 112 112
mujoco_cli gripper open
mujoco_cli image get --type rgb --output artifacts/end.png
mujoco_cli image get --camera opposite --type rgb --output artifacts/opposite.png
mujoco_cli image get --camera opposite --type depth --output artifacts/opposite_depth.png
mujoco_cli object list
mujoco_cli object get --object-id orange
mujoco_cli object in-target --object-id yellow
mujoco_cli object set --object-id orange --position 0.5 0.12 0.11 --settle 0.2
mujoco_cli object get
mujoco_cli object in-target
mujoco_cli object set --position 0.5 0 0.11 --settle 0.2
mujoco_cli environment reset
python scripts/request.py joints get
```

夹块 demo 只使用 `object get` 返回的位置，机械臂末端保持斜向下 45° 的固定朝向，流程见 [pick_place_demo.md](pick_place_demo.md)。`object set` 是环境设置操作，demo 的抓取流程不调用它。

stdout 恰好输出一个 JSON 对象。退出码：0 成功、2 参数或业务错误、3 连接失败、4 超时、5 服务端异常。`--request-json` 可直接传入与 HTTP 相同的请求体。

控制请求在误差达标后返回 `reached`。如果目标误差仍未达标，但关节、末端位姿或夹爪实际位置连续 `blocked_window_s`（默认 1 秒）变化范围小于 `blocked_delta`（默认 `0.001`），则返回 `ok=true`、`status=blocked` 和当前误差；请求不会继续等待到总 timeout。总 timeout 先到时仍返回 `TIMEOUT`。夹爪使用独立的 `gripper_blocked_delta=0.005 rad`，避免接触后的缓慢挤压反复重置停滞计时；机械臂继续使用 `blocked_delta=0.001`。夹爪返回 `blocked` 后继续保持闭合目标以维持夹持力；机械臂返回 `blocked` 后保持当前实测关节位置。
