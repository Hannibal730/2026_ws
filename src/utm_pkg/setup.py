import os
from glob import glob

from setuptools import find_packages, setup


package_name = "utm_pkg"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["README.md", "howtorun.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mando 2026 Team",
    maintainer_email="maintainer@example.com",
    description="ROS 2 UTM/ENU CSV mapping and protected dual-GNSS global yaw.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bag_to_enu_csv = utm_pkg.bag_to_enu_csv:main",
            "dual_gnss_map = utm_pkg.dual_gnss_map_node:main",
        ],
    },
)
