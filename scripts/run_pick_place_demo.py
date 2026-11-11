#!/usr/bin/env python3
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    address = f"http://127.0.0.1:{port}"
    server = subprocess.Popen([sys.executable, "-m", "mujoco_robot_cli.server", "--headless", "--port", str(port)], cwd=ROOT)
    try:
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if server.poll() is not None:
                return 3
            probe = subprocess.run([sys.executable, "-m", "mujoco_robot_cli.cli", "--server", address, "joints", "get"], cwd=ROOT, capture_output=True, text=True)
            if probe.returncode == 0:
                break
            time.sleep(0.5)
        else:
            print(probe.stderr, file=sys.stderr)
            return 3
        return subprocess.run([sys.executable, "scripts/pick_place_demo.py", "--server", address], cwd=ROOT).returncode
    finally:
        server.terminate()
        server.wait(10)


if __name__ == "__main__":
    raise SystemExit(main())
