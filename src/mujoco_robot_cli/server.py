from __future__ import annotations

import argparse
import os

import uvicorn

from .config import load_settings


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the persistent MuJoCo robot service")
    parser.add_argument("--config", default=None)
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--headless", action="store_true")
    mode.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    overrides = {key: value for key, value in {"host": args.host, "port": args.port}.items() if value is not None}
    if args.headless:
        overrides["headless"] = True
    if args.gui:
        overrides["headless"] = False
    settings = load_settings(args.config, overrides)
    if not settings.headless:
        settings.render_backend = "glfw"
    elif args.headless:
        settings.render_backend = "egl"
    # Select OpenGL before importing MuJoCo; spawn workers inherit this environment.
    os.environ["MUJOCO_GL"] = settings.render_backend
    from .api import create_app

    uvicorn.run(create_app(settings), host=settings.host, port=settings.port, workers=1, reload=False, log_level="info")


if __name__ == "__main__":
    main()
