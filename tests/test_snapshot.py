import multiprocessing as mp
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from mujoco_robot_cli.simulation.snapshot import LatestSnapshot


def publish_many(slot):
    for step in range(100):
        slot.publish({"step": step, "image": np.full((32, 32, 3), step, dtype=np.uint8)})


def test_shared_snapshot_overwrites_without_readers():
    ctx = mp.get_context("spawn")
    slot = LatestSnapshot(ctx, 16384)
    assert slot.read() is None
    process = ctx.Process(target=publish_many, args=(slot,))
    process.start()
    process.join(10)
    if process.is_alive():
        process.terminate()
        process.join()
        raise AssertionError("publisher blocked without readers")
    assert process.exitcode == 0
    with ThreadPoolExecutor(max_workers=8) as pool:
        copies = list(pool.map(lambda _: slot.read(), range(24)))
    for value in copies:
        assert value["step"] == 99
        assert np.all(value["image"] == 99)
    copies[0]["image"][:] = 0
    assert np.all(slot.read()["image"] == 99)
