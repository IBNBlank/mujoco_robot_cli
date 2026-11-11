import mujoco
import numpy as np

from mujoco_robot_cli.simulation.kinematics import solve_pose
from mujoco_robot_cli.simulation.scene import scene_path, quaternion_error


def test_model_ik_matches_tcp_without_mutating_live_state():
    model = mujoco.MjModel.from_xml_path(scene_path())
    live = mujoco.MjData(model)
    mujoco.mj_forward(model, live)
    original = live.qpos.copy()
    qids = [model.jnt_qposadr[model.joint(f"joint_{i}").id] for i in range(1, 7)]
    vids = [model.jnt_dofadr[model.joint(f"joint_{i}").id] for i in range(1, 7)]
    limits = np.array([model.joint(f"joint_{i}").range for i in range(1, 7)])
    tcp = model.body("ee_frame").id
    position = np.array([0.51057747, 0, 0.30924467])
    quaternion = np.array([np.cos(.45), 0, np.sin(.45), 0])
    solved = solve_pose(model, live, qids, vids, tcp, limits, position, quaternion, np.zeros(6))
    assert solved is not None
    np.testing.assert_array_equal(live.qpos, original)
    live.qpos[qids] = solved
    mujoco.mj_forward(model, live)
    assert np.linalg.norm(live.xpos[tcp] - position) < 0.0002
    assert quaternion_error(live.xquat[tcp], quaternion) < 0.002
    assert np.all(solved >= limits[:, 0]) and np.all(solved <= limits[:, 1])
