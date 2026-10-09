import site
from pathlib import Path

conda_site = Path(site.getsitepackages()[0])
pth = conda_site / "ros2_jazzy.pth"

pth.write_text(
    "/opt/ros/jazzy/lib/python3.12/site-packages\n"
)

print("Created:", pth)
