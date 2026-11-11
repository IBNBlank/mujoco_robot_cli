# mujoco_robot_cli

常驻 MuJoCo Archer Y6 桌面仿真服务，以及面向未来 tool 调用的一次性请求 CLI。

## 安装

```bash
uv venv --python 3.10 .venv
source .venv/bin/activate
uv pip install -e .
```

`hex_util_robot`、`hex_util_runtime` 和 `hex_util_urdf` 当前不在公共索引的可复现锁定范围内。本仓库验证时分别使用本地源码和已有的 `hex_util_urdf==0.0.1a2` wheel；部署前需提供这三个包的相同版本。

## 运行

```bash
mujoco_server --config configs/default.yaml
mujoco_cli joints get
```

默认打开 MuJoCo 图形窗口。无窗口运行时显式指定：

```bash
mujoco_server --config configs/default.yaml --headless
```

场景包含机械臂、手部相机、桌面、绿色目标区和红色物块。服务进程持续维护唯一仿真；每次 CLI 调用结束即退出，不会重置或关闭仿真。

接口、相机校正和实测限制分别见：

- [docs/api.md](docs/api.md)
- [docs/camera.md](docs/camera.md)
- [docs/validation.md](docs/validation.md)
- [docs/PLAN.md](docs/PLAN.md)

## 验证

先将测试依赖安装到项目虚拟环境，再通过该环境的 Python 运行 pytest：

```bash
uv pip install --python .venv/bin/python 'pytest>=8'
PYTHONPATH=src MUJOCO_GL=egl .venv/bin/python -m pytest tests/test_api.py
```

测试显式使用无窗口模式，不影响服务默认的 GUI 模式。

当前自动化验证覆盖关节控制、状态查询、图像、参数错误、物块非法放置、环境重置，以及服务器不可达时 CLI 的退出码。
