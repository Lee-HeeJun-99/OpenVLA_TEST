from glob import glob
from setuptools import find_packages, setup

package_name = "openvla_doosan_runtime"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="OpenVLA runtime for Doosan A0509",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_adapter = openvla_doosan_runtime.camera_adapter_node:main",
            "openvla_inference = openvla_doosan_runtime.openvla_inference_node:main",
            "action_adapter = openvla_doosan_runtime.action_adapter_node:main",
            "doosan_bridge = openvla_doosan_runtime.doosan_bridge_node:main",
            "episode_manager = openvla_doosan_runtime.episode_manager_node:main",
        ],
    },
)
