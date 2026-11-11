# Third-party assets and dependencies

Archer Y6 MuJoCo model files under `src/mujoco_robot_cli/assets/archer_y6/` are copied from `hex_driver_mujoco` commit `46d7de2c07d2d0af64ae52f91975f5ec072358a5`. The upstream project is licensed under Apache-2.0. The only intentional model change is the `end_camera` quaternion, documented in `docs/camera.md`.

The implementation also depends on MuJoCo, FastAPI, Uvicorn, Pydantic, NumPy, OpenCV, Pillow, PyYAML, HTTPX, and the HEXFELLOW packages `hex_util_robot`, `hex_util_runtime`, and `hex_util_urdf`. Their licenses remain with their respective projects.
