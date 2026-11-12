# 相机方向与像素坐标

手部相机 `end_camera` 位于 `grip_base_link` 局部坐标 `(0.066, 0, 0.105)`，视场角 50°。相机光轴平行于 TCP（`ee_frame`）的 **+X**，画面上方为 TCP **+Z**，画面右方为 TCP **−Y**。相机保留安装位置偏移，因此光轴与 TCP 前进方向夹角为零，但两个原点不重合。

MuJoCo 相机局部 −Z 是观察方向，+Y 是画面上方，因此模型使用 `xyaxes="0 -1 0 0 0 1"`。该关系随机械臂运动保持不变。

## RGB-D 与反投影

两路相机均输出 RGB PNG 与 uint16 毫米深度 PNG。深度表示沿光轴的距离，不是到相机原点的欧氏距离；0 表示无效深度。背景、非有限值、裁剪范围外和超出 uint16 可表达范围的深度均置零。关闭离屏多重采样，使深度对应像素中心，避免子像素深度采样引入反投影偏差。

`POST /v1/camera/point` 必须指定 `camera`、`pixel` 和该像素的 `depth_m`。默认服务端内参及外参用于反投影；省略 TCP 位姿时，`end` 使用当前 TCP 位姿，`left_rear` 使用固定相机位姿。可传入 `tcp_position_base` 与 `tcp_quaternion_base` 指定 TCP 位姿，必须成对提供。也可提供图像响应头中的全套采集元数据（TCP 位姿、相机相对 TCP 外参和内参），按该帧的标定精确反投影；这一路径不读取当前 TCP 状态或服务端图像。返回坐标为 `base_link` 下 XYZ（米）。选择当前姿态意味着坐标按请求处理时的状态计算，并不保证与更早获取的深度帧同步。越界像素或无效深度返回 `INVALID_ARGUMENT`。

`GET /v1/camera/image` 返回的图像响应头包含 base64url 编码的采集时 TCP pose、相机相对 TCP 外参和内参。`end` 和 `left_rear` 均可用，结果为 `base_link` 下 XYZ（米）；无效深度不应提交。

CLI 的 `image point` 需要将同一张 depth 图中的采样值、采集位姿、相机外参和内参通过 `--request-json` 传入 `/v1/camera/point` 请求体。

## 固定左后方相机

`left_rear`（模型名 `left_rear_camera`）固定于 world `[-0.65, 0.85, 1.15]`，朝向桌面中心 `[0.5, 0, 0.32]`，fovy 60°。机器人前方定义为 +X，左侧为 +Y，因此 −X/+Y 为左后方。两路相机通过共享快照提供最新已发布图像，依次渲染。

```bash
mujoco_cli image get --camera left_rear --output artifacts/left_rear.png
```
