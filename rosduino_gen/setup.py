# SPDX-License-Identifier: Apache-2.0
from pathlib import Path

from setuptools import find_packages, setup

package_name = "rosduino_gen"
readme = Path(__file__).with_name("README.md").read_text(encoding="utf-8")

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "generated"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "arduino_types.txt"]),
        ("share/" + package_name + "/doc", ["doc/arduino_due.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RosDuino Contributors",
    maintainer_email="maintainers@voltbro.com",
    description="Build-time generator of RosDuino C++ headers from ROS 2 .msg/.srv interfaces.",
    long_description=readme,
    long_description_content_type="text/markdown",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rosduino-gen = rosduino_gen.cli:main",
        ],
    },
)
