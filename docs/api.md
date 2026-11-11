# API 与 CLI

服务默认监听 `127.0.0.1:8000`。所有 JSON 响应使用：

```json
{"ok": true, "request_id": "...", "data": {}, "error": null}
```

失败时 `ok=false`，`error.code` 为 `INVALID_ARGUMENT`、`BUSY`、`IK_FAILED`、`TIMEOUT`、`NOT_READY`、`CAMERA_DISABLED` 或 `INTERNAL_ERROR`。

## HTTP

- `POST /v1/arm/joints`：`{"positions":[6个弧度],"timeout_s":15}`
- `GET /v1/arm/joints`
- `POST /v1/arm/pose`：`{"position":[x,y,z],"quaternion":[w,x,y,z],"frame":"world"}`；省略姿态时保持当前姿态。
- `GET /v1/arm/pose?frame=world|base`
- `POST /v1/gripper`：`{"action":"open"}`、`{"action":"close"}` 或 `{"opening_ratio":0.5}`。
- `GET /v1/gripper`
- `GET /v1/camera/image?camera=end|left_rear&type=rgb|depth`：手部或固定左后方相机，成功返回 PNG；深度为 uint16 毫米。省略 camera 时使用手部相机。
- `POST /v1/object/pose`：设置红色物块的世界坐标和可选姿态。
- `GET /v1/object/pose`：查询红色物块的世界位置和姿态。
- `GET /v1/object/in-target`：查询物块中心是否位于绿色目标区水平范围内，并已落在桌面高度。
- `POST /v1/environment/reset`
- `GET /healthz`、`GET /readyz`、`GET /v1/capabilities`

修改请求同时只允许一个。状态和图像通过带进程锁的共享内存最新快照传递；发布时覆盖旧数据，读取不消费数据，也不依赖客户端是否曾经查询。每次查询复制读取时最新发布的完整快照，业务字段与 `epoch`、`step`、时间戳来自同一份数据。超过新鲜度上限返回 `NOT_READY`。

“最新”指最近一次采样并发布的数据：默认状态发布 500 Hz、相机 30 Hz，实际频率受物理步进和渲染耗时影响；并非每次请求都会重新采样或渲染。指令及执行结果仍使用队列传递。

## CLI

```bash
mujoco_server --config configs/default.yaml --headless
mujoco_cli joints set --positions 0.1 -0.2 0.4 -0.2 0.1 0.05 --timeout 15
mujoco_cli pose get --frame world
mujoco_cli gripper open
mujoco_cli image get --type rgb --output artifacts/end.png
mujoco_cli image get --camera left_rear --type rgb --output artifacts/left_rear.png
mujoco_cli image get --camera left_rear --type depth --output artifacts/left_rear_depth.png
mujoco_cli object get
mujoco_cli object in-target
mujoco_cli object set --position 0.5 0 0.36 --settle 0.2
mujoco_cli environment reset
python scripts/request.py joints get
```

夹块 demo 只使用 `object get` 返回的位置，机械臂末端保持斜向下 45° 的固定朝向，流程见 [pick_place_demo.md](pick_place_demo.md)。`object set` 是环境设置操作，demo 的抓取流程不调用它。

stdout 恰好输出一个 JSON 对象。退出码：0 成功、2 参数或业务错误、3 连接失败、4 超时、5 服务端异常。`--request-json` 可直接传入与 HTTP 相同的请求体。

控制请求在误差达标后返回 `reached`。如果目标误差仍未达标，但关节、末端位姿或夹爪实际位置连续 `blocked_window_s`（默认 1 秒）变化范围小于 `blocked_delta`（默认 `0.001`），则返回 `ok=true`、`status=blocked` 和当前误差；请求不会继续等待到总 timeout。总 timeout 先到时仍返回 `TIMEOUT`。夹爪使用独立的 `gripper_blocked_delta=0.005 rad`，避免接触后的缓慢挤压反复重置停滞计时；机械臂继续使用 `blocked_delta=0.001`。夹爪返回 `blocked` 后继续保持闭合目标以维持夹持力；机械臂返回 `blocked` 后保持当前实测关节位置。
