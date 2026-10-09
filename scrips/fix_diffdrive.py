from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET


def fix_duplicate_diffdrive():
    model_path = Path(
        "/home/user/Desktop/Robots/"
        "Round_Robot/models/round_robot/model.sdf"
    )

    content = model_path.read_text(encoding="utf-8")

    # Find all DiffDrive plugin declarations
    pattern = re.compile(
        r'<plugin\b'
        r'(?=[^>]*filename="gz-sim-diff-drive-system")'
        r'(?=[^>]*name="gz::sim::systems::DiffDrive")'
        r'[^>]*>.*?</plugin>',
        re.DOTALL
    )

    matches = list(pattern.finditer(content))

    print(f"Found {len(matches)} DiffDrive plugins")

    if len(matches) <= 1:
        print("No duplicate DiffDrive found.")
        return

    # Check that duplicate configurations are equivalent
    def get_parameters(match):
        element = ET.fromstring(match.group())
        return {
            child.tag: (child.text or "").strip()
            for child in element
        }

    original_parameters = get_parameters(matches[0])

    for match in matches[1:]:
        if get_parameters(match) != original_parameters:
            raise ValueError(
                "DiffDrive configurations differ. "
                "Manual review required."
            )

    # Keep first plugin, remove duplicates
    for match in reversed(matches[1:]):
        content = content[:match.start()] + content[match.end():]

    # Validate XML before writing
    root = ET.fromstring(content)

    remaining = [
        plugin
        for plugin in root.iter("plugin")
        if plugin.get("name") == "gz::sim::systems::DiffDrive"
    ]

    assert len(remaining) == 1

    # Backup original model
    backup_path = model_path.with_suffix(".sdf.bak")
    shutil.copy2(model_path, backup_path)

    # Save fixed model
    model_path.write_text(content, encoding="utf-8")

    print("Duplicate DiffDrive removed successfully.")
    print(f"Remaining DiffDrive plugins: {len(remaining)}")
    print(f"Backup: {backup_path}")


if __name__ == "__main__":
    fix_duplicate_diffdrive()