from glob import glob
from setuptools import find_packages, setup

package_name = "vla_data_collector"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="juhun",
    maintainer_email="juhun@example.com",
    description="OpenVLA data collection for Doosan A0509 and ZED 2i",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "waypoint_recorder = vla_data_collector.waypoint_recorder:main",
            "dataset_recorder = vla_data_collector.dataset_recorder:main",
            "make_actions = vla_data_collector.make_actions:main",
            "validate_episode = vla_data_collector.validate_episode:main",
        ],
    },
)
