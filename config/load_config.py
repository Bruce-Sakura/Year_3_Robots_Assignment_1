from pathlib import Path
import yaml


def load_config():
    """Load robot geometry relative to this module, independent of cwd."""
    config_path = Path(__file__).resolve().parent / "robot.yaml"

    with open(config_path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    return data["robot_controller"]["ros__parameters"]