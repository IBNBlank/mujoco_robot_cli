"""Model-based bounded inverse kinematics, evaluated without changing live data."""
from __future__ import annotations

import mujoco
import numpy as np

from .scene import orientation_delta, quaternion_error


def solve_pose(model, data, joint_q, joint_v, tcp, limits, position, quaternion, seed):
    scratch = mujoco.MjData(model)
    scratch.qpos[:] = data.qpos
    jac = np.zeros((6, model.nv))
    # Analytic branches can differ; several deterministic seeds avoid the home singularity.
    seeds = [np.asarray(seed).copy()]
    seeds += [np.array([seed[0], a, b, c, 0., 0.]) for a, b, c in
              [(-0.6, 2.5, -0.5), (0.4, 1.5, 0.8), (1., 1., 0.5), (-0.8, 2., 1.)]]
    best = None
    for initial in seeds:
        q = np.clip(initial, limits[:, 0], limits[:, 1])
        for _ in range(200):
            scratch.qpos[joint_q] = q
            mujoco.mj_kinematics(model, scratch)
            mujoco.mj_comPos(model, scratch)
            current_quat = scratch.xquat[tcp]
            ep = np.asarray(position) - scratch.xpos[tcp]
            er = orientation_delta(current_quat, np.asarray(quaternion))
            if np.linalg.norm(ep) < 0.0002 and quaternion_error(current_quat, quaternion) < 0.002:
                if best is None or np.linalg.norm(q - seed) < np.linalg.norm(best - seed):
                    best = q.copy()
                break
            mujoco.mj_jacBody(model, scratch, jac[:3], jac[3:], tcp)
            j = jac[:, joint_v].copy()
            j[3:] *= 0.3
            e = np.concatenate([ep, er * 0.3])
            dq = j.T @ np.linalg.solve(j @ j.T + 0.0001 * np.eye(6), e)
            q_next = np.clip(q + np.clip(dq, -0.15, 0.15), limits[:, 0], limits[:, 1])
            if np.linalg.norm(q_next - q) < 1e-8:
                break
            q = q_next
    return best
