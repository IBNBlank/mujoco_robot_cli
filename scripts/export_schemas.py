#!/usr/bin/env python3
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from mujoco_robot_cli.schemas import GripperCommand, JointCommand, ObjectCommand, PoseCommand, ResetCommand

root = Path(__file__).resolve().parents[1] / "schemas"
root.mkdir(exist_ok=True)
for model in (JointCommand, PoseCommand, GripperCommand, ObjectCommand, ResetCommand):
    (root / f"{model.__name__}.json").write_text(json.dumps(model.model_json_schema(), indent=2))
print(f"exported {len(list(root.glob('*.json')))} schemas")
