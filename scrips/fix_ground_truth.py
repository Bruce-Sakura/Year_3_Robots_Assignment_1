"""Repair Ground Truth publication and ROS 2 bridge for this Robots project.

Place this file in: <project root>/scrips/fix_ground_truth.py
Run: python scrips/fix_ground_truth.py

All paths are discovered from this script's location. It is safe to run more
than once and creates .bak copies before changing existing project files.
"""
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODEL_SDF = PROJECT_ROOT / "Round_Robot" / "models" / "round_robot" / "model.sdf"
START_SCRIPT = PROJECT_ROOT / "start_sim.sh"

POSE_PLUGIN = '''    <!-- Gazebo ground truth; evaluation only, never control -->
    <plugin filename="gz-sim-pose-publisher-system"
            name="gz::sim::systems::PosePublisher">
      <publish_model_pose>true</publish_model_pose>
      <publish_nested_model_pose>true</publish_nested_model_pose>
      <publish_link_pose>false</publish_link_pose>
      <use_pose_vector_msg>false</use_pose_vector_msg>
      <topic>/model/round_robot/ground_truth</topic>
      <update_frequency>50</update_frequency>
    </plugin>
'''

GZ_BRIDGE_LINE = (
    "    '/model/round_robot/ground_truth"
    "@geometry_msgs/msg/PoseStamped[gz.msgs.Pose' \\\n"
)
ROS_REMAP_LINE = "    -r /model/round_robot/ground_truth:=/ground_truth &"


def backup_and_write(path: Path, content: str) -> None:
    old = path.read_text(encoding="utf-8")
    if content == old:
        print(f"Already correct: {path.relative_to(PROJECT_ROOT)}")
        return
    backup = path.with_name(path.name + ".bak")
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_text(content, encoding="utf-8")
    print(f"Updated: {path.relative_to(PROJECT_ROOT)}; backup: {backup.name}")


def update_model() -> None:
    text = MODEL_SDF.read_text(encoding="utf-8")
    root = ET.fromstring(text)
    if root.tag != "sdf":
        raise ValueError("Expected an SDF document")
    model = root.find("model")
    if model is None or model.get("name") != "round_robot":
        raise ValueError("Expected model name round_robot")

    plugins = [
        el for el in model.findall("plugin")
        if el.get("name") == "gz::sim::systems::PosePublisher"
    ]
    if len(plugins) > 1:
        raise ValueError("Multiple PosePublisher plugins detected; review manually")
    if not plugins:
        pos = text.rfind("  </model>")
        if pos < 0:
            raise ValueError("Could not find closing </model>")
        text = text[:pos] + POSE_PLUGIN + text[pos:]
        ET.fromstring(text)  # Do not corrupt a large SDF file.
    else:
        plugin = plugins[0]
        expected = {
            "publish_model_pose": "true",
            "publish_nested_model_pose": "true",
            "use_pose_vector_msg": "false",
            "topic": "/model/round_robot/ground_truth",
        }
        mismatches = {
            tag: (plugin.findtext(tag), value)
            for tag, value in expected.items()
            if (plugin.findtext(tag) or "").strip() != value
        }
        if mismatches:
            raise ValueError(
                f"Existing PosePublisher settings need manual review: {mismatches}"
            )
    backup_and_write(MODEL_SDF, text)


def update_bridge() -> None:
    text = START_SCRIPT.read_text(encoding="utf-8")
    if "ground_truth@geometry_msgs/msg/PoseStamped[gz.msgs.Pose" not in text:
        anchor = "    --ros-args \\\n"
        if text.count(anchor) != 1:
            raise ValueError("Bridge --ros-args marker not found exactly once")
        text = text.replace(anchor, GZ_BRIDGE_LINE + anchor, 1)

    if "-r /model/round_robot/ground_truth:=/ground_truth" not in text:
        anchor = "    -r /model/round_robot/joint_states:=/joint_states &"
        if text.count(anchor) != 1:
            raise ValueError("Joint-state remap anchor not found exactly once")
        text = text.replace(anchor,
            "    -r /model/round_robot/joint_states:=/joint_states \\\n"
            + ROS_REMAP_LINE, 1)

    backup_and_write(START_SCRIPT, text)


def main() -> None:
    print("Project root:", PROJECT_ROOT)
    for path in (MODEL_SDF, START_SCRIPT):
        if not path.is_file():
            raise FileNotFoundError(path)
    update_model()
    update_bridge()
    print("\nRestart the running Gazebo and start_sim.sh processes to load these changes.")
    print("Check Gazebo: gz topic -l | grep ground_truth")
    print("Check ROS:    ros2 topic echo /ground_truth --once")


if __name__ == "__main__":
    main()
