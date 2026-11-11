"""Single-writer shared-memory snapshots, copied atomically by any reader."""
from __future__ import annotations

import pickle


class LatestSnapshot:
    def __init__(self, context, capacity: int) -> None:
        self._buffer = context.RawArray("B", capacity)
        self._length = context.RawValue("Q", 0)
        self._lock = context.Lock()
        self._capacity = capacity

    def publish(self, value: dict) -> None:
        payload = pickle.dumps(value, protocol=pickle.HIGHEST_PROTOCOL)
        if len(payload) > self._capacity:
            raise ValueError(f"snapshot requires {len(payload)} bytes; capacity is {self._capacity}")
        # Serialization is outside the critical section; readers only wait for a copy.
        with self._lock:
            memoryview(self._buffer).cast("B")[:len(payload)] = payload
            self._length.value = len(payload)

    def read(self) -> dict | None:
        with self._lock:
            length = self._length.value
            if not length:
                return None
            payload = bytes(memoryview(self._buffer).cast("B")[:length])
        return pickle.loads(payload)
