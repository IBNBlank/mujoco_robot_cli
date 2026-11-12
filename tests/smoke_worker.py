from mujoco_robot_cli.config import load_settings
from mujoco_robot_cli.simulation.worker import WorkerClient


def main() -> None:
    worker = WorkerClient(load_settings(overrides={"startup_timeout_s": 30}))
    try:
        worker.start()
        print("ready", worker.latest()["epoch"], worker.latest()["pose"]["position"])
        import time
        before = worker.latest()["step"]
        time.sleep(0.5)
        print("steps_per_0.5s", worker.latest()["step"] - before)
        for kind, payload, timeout in (
            ("joints", {"positions": [0.2, -0.4, 0.8, -0.4, 0.2, 0.1]}, 8),
            ("pose", {"position": [0.4, 0, 0.25], "quaternion": [1, 0, 0, 0], "frame": "base"}, 8),
            ("gripper", {"action": "open"}, 8),
            ("object", {"position": [0.5, 0, 0.36], "settle_s": 0.2}, 5),
            ("reset", {"settle_s": 0.1}, 5),
        ):
            print(kind, worker.submit(kind, payload, timeout), getattr(worker, '_unexpected', None))
            print("state", worker.latest()["joints"]["positions"], worker.latest()["pose"]["position"])
    finally:
        worker.close()


if __name__ == "__main__":
    main()
