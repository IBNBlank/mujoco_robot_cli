"""Measure actual close duration and joint drift at the demo grasp pose."""
import json
import threading
import time
from pathlib import Path

import numpy as np

from mujoco_robot_cli.config import load_settings
from mujoco_robot_cli.simulation.worker import WorkerClient
from mujoco_robot_cli.simulation.scene import rotate_vector


def main():
    worker = WorkerClient(load_settings(overrides={"headless": True, "render_backend": "egl"}))
    worker.start()
    records = []
    try:
        for trial in range(3):
            worker.submit("reset", {"settle_s": .3}, 10)
            worker.submit("gripper", {"action": "open"}, 10)
            snap = worker.latest()
            obj = np.array(snap["object"]["position"])
            quat = [float(np.cos(np.pi/8)), 0., float(np.sin(np.pi/8)), 0.]
            grasp = obj + rotate_vector(np.array(quat), np.array([.015, 0, .0016]))
            for position in ([grasp[0], grasp[1], .44252477], grasp.tolist()):
                result = worker.submit("pose", {"position": position, "quaternion": quat, "frame": "world"}, 20)
                if result.get("status") != "reached":
                    raise RuntimeError(result)
            samples = []
            output = []
            start = time.monotonic()
            thread = threading.Thread(target=lambda: output.append(worker.submit("gripper", {"action": "close"}, 20)))
            thread.start()
            while thread.is_alive():
                snap = worker.latest()
                samples.append({"t": time.monotonic()-start, "q": snap["gripper"]["raw_position"], "v": snap["gripper"]["velocity"], "step": snap["step"]})
                time.sleep(.01)
            thread.join()
            elapsed = time.monotonic()-start
            summary = {"trial": trial+1, "elapsed_s": elapsed, "result": output[0], "last_second_range_rad": max(s["q"] for s in samples if s["t"] > elapsed-1)-min(s["q"] for s in samples if s["t"] > elapsed-1)}
            print(json.dumps(summary), flush=True)
            for second in range(int(elapsed)+1):
                section = [s for s in samples if second <= s["t"] < second+1]
                if section:
                    print(f"  {second}-{second+1}s q={section[0]['q']:.6f}->{section[-1]['q']:.6f} range={max(s['q'] for s in section)-min(s['q'] for s in section):.6f}", flush=True)
            records.append({"summary": summary, "samples": samples})
    finally:
        worker.close()
        Path("artifacts").mkdir(exist_ok=True)
        Path("artifacts/gripper_timing.json").write_text(json.dumps(records, indent=2))


if __name__ == "__main__":
    main()
