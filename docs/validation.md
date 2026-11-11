# 验证记录

验证环境：Python 3.10.12、MuJoCo 3.17.0、`MUJOCO_GL=egl`。参考模型提交为 `46d7de2c07d2d0af64ae52f91975f5ec072358a5`。

## 实测控制

控制执行器已从力矩 motor 改为 MuJoCo 位置伺服，关节 kp 为 500–2500。控制请求保持阻塞，直到误差和速度连续达标，或达到 timeout。

实测代表性动作：

- 关节位置最大残差约 0.0016 rad。
- 末端综合误差约 0.0066 rad/m，满足 0.005 m 与 0.05 rad 的组合阈值。
- 夹爪可打开到约 0.002 rad 的剩余误差。
- 物块可设置到 `(0.5, 0, 0.36)`，reset 后 epoch 增加且关节恢复 home。

当前验收阈值为关节 0.01 rad、速度 0.05 rad/s、末端位置 0.005 m、姿态 0.05 rad。

## 已运行检查

- `PYTHONPATH=src MUJOCO_GL=egl python tests/smoke_worker.py`
- `PYTHONPATH=src MUJOCO_GL=egl pytest tests/test_api.py`
- 相机校正前后图像已保存到 `artifacts/`。

GUI 模式代码已提供，但本次自动化验证使用 EGL headless，未在图形界面中交互验证。
